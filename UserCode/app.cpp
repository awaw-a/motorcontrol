#include "can.h"
#include "cmsis_os2.h"
#include "dji.hpp"
#include "gpio_driver.hpp"
#include "motor_pos_controller.hpp"
#include "motor_vel_controller.hpp"
#include "static_object.hpp"
#include "upper_uart.hpp"
#include "usart.h"
#include <algorithm>
#include <cmath>
#include <cstdint>

#ifndef USE_M2006_MOTOR
#define USE_M2006_MOTOR 0
#endif

namespace
{
constexpr uint8_t kMotorCanId = 1;
constexpr bool kMotorReverse = true;
#if USE_M2006_MOTOR
constexpr auto kMotorType = motors::DJIMotor::Type::M2006_C610;
constexpr float kMaxCurrent = 4000.0f;
constexpr float kVelKp = 25.0f, kVelKi = 0.4f, kVelKd = 0.0f;
constexpr float kPosKp = 2.2f, kPosKi = 0.0f, kPosKd = 0.5f;
#else
constexpr auto kMotorType = motors::DJIMotor::Type::M3508_C620;
constexpr float kMaxCurrent = 8000.0f;
constexpr float kVelKp = 35.0f, kVelKi = 0.6f, kVelKd = 0.0f;
constexpr float kPosKp = 2.0f, kPosKi = 0.0f, kPosKd = 0.4f;
#endif
constexpr float kVelTolRpm = 15.0f;
constexpr float kPosTolDeg = 5.0f;
constexpr uint32_t kSettleMs = 50;
constexpr uint32_t kMaxControlGapMs = 5;
constexpr bsp::gpio::GpioPinOutput kLed0 = GpioPinWithName(LED_0);

memory::StaticObject<UpperUart> g_uart_rx;
memory::StaticObject<motors::DJIMotor> g_motor;
memory::StaticObject<controllers::MotorVelController> g_vel_ctrl;
memory::StaticObject<controllers::MotorPosController> g_pos_ctrl;

struct StableCondition
{
    bool active = false;
    uint32_t since = 0;
    void reset() { active = false; }
    bool update(bool condition, uint32_t now, uint32_t duration)
    {
        if (!condition)
        {
            reset();
            return false;
        }
        if (!active)
        {
            active = true;
            since = now;
        }
        return now - since >= duration;
    }
} g_reached;

struct PresetState
{
    bool active = false;
    uint32_t action_id = 0;
    uint32_t start_tick = 0;
    size_t stage = 0;
    StableCondition hold{};
} g_preset;

struct SafetyState
{
    bool fault_latched = false;
    bool ever_running = false;
    bool stopped = false;
    bool tx_flushed = false;
    uint32_t fault_sequence = 0;
    uint32_t motor_wait_sequence = 0;
    uint32_t uart_faults = 0;
} g_safety;
UpperUart::Command g_last_command{};
bool g_have_last_command = false;
uint8_t g_status = 0xFF;
uint32_t g_deadline_misses = 0;

void setActiveController(controllers::IController* target)
{
    if (auto* current = g_motor->currentController(); current != target)
    {
        if (current)
            current->disable();
        g_reached.reset();
        if (target)
            target->enable();
    }
}

void stopSystem(bool latch)
{
    if (latch && !g_safety.fault_latched)
    {
        g_safety.fault_latched = true;
        g_safety.fault_sequence = g_uart_rx->sequence();
    }
    setActiveController(nullptr);
    g_motor->setCurrent(0.0f);
    g_vel_ctrl->reset();
    g_pos_ctrl->reset();
    g_reached.reset();
    g_preset = {};
    g_have_last_command = false;
    if (!g_safety.stopped || !g_safety.tx_flushed)
        g_safety.tx_flushed = CAN_DiscardPendingTx(&hcan1);
    g_safety.stopped = true;
}

bool atPosition(const motors::DJIMotor::Feedback& feedback, float target)
{
    return std::fabs(feedback.angle - target) <= kPosTolDeg &&
           std::fabs(feedback.velocity) <= kVelTolRpm;
}

uint8_t runPresetSpeed(uint32_t now, const motors::DJIMotor::Feedback& feedback)
{
    struct Step { uint32_t end_ms; float rpm; };
    static constexpr Step steps[] = {
        {1000, 60}, {2000, 120}, {3000, 180}, {5000, 240},
        {7000, 300}, {8000, 150}, {9000, 0},
    };
    setActiveController(g_vel_ctrl);
    const uint32_t elapsed = now - g_preset.start_tick;
    for (const auto& step : steps)
    {
        if (elapsed < step.end_ms)
        {
            g_vel_ctrl->setRef(step.rpm, kMaxCurrent);
            g_reached.reset();
            return 0xAA;
        }
    }
    g_vel_ctrl->setRef(0.0f, kMaxCurrent);
    return g_reached.update(std::fabs(feedback.velocity) <= kVelTolRpm, now, kSettleMs)
                   ? 0xAF : 0xAA;
}

uint8_t runPresetPosition(uint32_t now, const motors::DJIMotor::Feedback& feedback)
{
    struct Step { float deg; uint32_t hold_ms; };
    static constexpr Step steps[] = {{0, 0}, {720, 3000}, {360, 1000}, {0, 0}};
    constexpr size_t count = sizeof(steps) / sizeof(steps[0]);
    setActiveController(g_pos_ctrl);
    if (g_preset.stage < count)
    {
        const auto& step = steps[g_preset.stage];
        if (g_preset.hold.update(atPosition(feedback, step.deg), now,
                                 std::max(step.hold_ms, kSettleMs)))
        {
            ++g_preset.stage;
            g_preset.hold.reset();
        }
    }
    const bool done = g_preset.stage >= count;
    g_pos_ctrl->setRef(done ? 0.0f : steps[g_preset.stage].deg, 300.0f);
    return done && g_reached.update(atPosition(feedback, 0.0f), now, kSettleMs) ? 0xBF : 0xBA;
}

// 单次控制周期独立出来，便于使用实际业务代码进行故障注入测试。
void controlStep(uint32_t now, bool timing_fault)
{
    g_uart_rx->poll(now);
    const auto feedback = g_motor->getFeedback();
    const bool connected = g_uart_rx->isConnected(now);
    kLed0.write(connected ? GPIO_PIN_SET : GPIO_PIN_RESET);
    UpperUart::Command command{};
    const bool have_command = g_uart_rx->getCommand(command, now);
    const bool uart_fault = g_safety.uart_faults != g_uart_rx->faultCount();
    g_safety.uart_faults = g_uart_rx->faultCount();
    const bool motor_ready = feedback.connected && feedback.initialized && feedback.position_valid;
    const bool tx_fault = CAN_TakeTxError(&hcan1);

    if (!motor_ready)
        g_safety.motor_wait_sequence = g_uart_rx->sequence();
    g_status = motor_ready ? 0xAA : 0xFF;
    if (timing_fault || tx_fault || uart_fault || !motor_ready || !have_command)
    {
        stopSystem(g_safety.ever_running || timing_fault || tx_fault);
        g_status = 0xFF;
    }
    else if (g_safety.fault_latched)
    {
        g_status = 0xFF;
        // 兼容上位机急停按钮：00 模式、A=0、B=0。重连旧指令不能直接重启。
        if (command.isStop() && g_uart_rx->sequence() != g_safety.fault_sequence &&
            g_safety.tx_flushed)
        {
            g_safety.fault_latched = false;
            g_status = 0xAF;
        }
        stopSystem(false);
    }
    else if (command.isStop())
    {
        stopSystem(false);
        g_status = g_safety.tx_flushed ? 0xAF : 0xFF;
    }
    else if (g_uart_rx->sequence() == g_safety.motor_wait_sequence)
    {
        stopSystem(false); // 电机就绪后仍要求至少一帧新指令。
    }
    else if (g_safety.stopped && !CAN_DiscardPendingTx(&hcan1))
    {
        stopSystem(true);
    }
    else
    {
        g_safety.stopped = false;
        g_safety.tx_flushed = false;
        g_safety.ever_running = true;
        if (!g_have_last_command || !(command == g_last_command))
            g_reached.reset();
        g_last_command = command;
        g_have_last_command = true;
        if (command.mode == 0x00)
        {
            g_preset = {};
            setActiveController(g_vel_ctrl);
            const float limit = std::min(std::fabs(static_cast<float>(command.val_a)), kMaxCurrent);
            const float reference = command.val_b;
            g_vel_ctrl->setRef(reference, limit);
            g_status = g_reached.update(std::fabs(feedback.velocity - reference) <= kVelTolRpm,
                                        now, kSettleMs) ? 0xAF : 0xAA;
        }
        else if (command.mode == 0x11)
        {
            g_preset = {};
            setActiveController(g_pos_ctrl);
            g_pos_ctrl->setRef(command.val_b, std::fabs(static_cast<float>(command.val_a)));
            g_status = g_reached.update(atPosition(feedback, command.val_b), now, kSettleMs)
                               ? 0xBF : 0xBA;
        }
        else
        {
            if (!g_preset.active || g_preset.action_id != command.action_id)
            {
                g_preset = { true, command.action_id, now, 0, {} };
                g_reached.reset();
            }
            g_status = command.action_id == 0 ? runPresetSpeed(now, feedback)
                                               : runPresetPosition(now, feedback);
        }
        if (auto* controller = g_motor->currentController())
            controller->update();
    }
    if (motors::DJIMotor::SendIqCommand(&hcan1,
                motors::DJIMotor::IqSetCMDGroup::IqCMDGroup_1_4) == CAN_SEND_FAILED)
    {
        stopSystem(true);
        g_status = 0xFF;
    }
}

void ControlTask(void*)
{
    uint32_t next_tick = osKernelGetTickCount();
    uint32_t last_control_ms = HAL_GetTick();
    uint32_t last_tx_ms = last_control_ms;
    for (;;)
    {
        if (osDelayUntil(++next_tick) != osOK)
        {
            ++g_deadline_misses;
            next_tick = osKernelGetTickCount();
        }
        const uint32_t now = HAL_GetTick();
        controlStep(now, now - last_control_ms >= kMaxControlGapMs);
        last_control_ms = now;
        if (now - last_tx_ms >= 20)
        {
            last_tx_ms = now;
            const auto feedback = g_motor->getFeedback();
            const uint8_t status = feedback.connected && feedback.position_valid ? g_status : 0xFF;
            g_uart_rx->sendFeedback(status, feedback.velocity, feedback.angle, now);
        }
    }
}
} // namespace

extern "C" void Init(void*)
{
    INIT_OBJECT(g_motor, motors::DJIMotor::Config{
        .hcan = &hcan1, .type = kMotorType, .id1 = kMotorCanId,
        .auto_zero = true, .reverse = kMotorReverse, .reduction_rate = 1.0f,
        .current_limit = kMaxCurrent,
    });
    INIT_OBJECT(g_vel_ctrl, g_motor, controllers::MotorVelController::Config{
        .pid = { kVelKp, kVelKi, kVelKd, kMaxCurrent },
    });
    INIT_OBJECT(g_pos_ctrl, g_motor, controllers::MotorPosController::Config{
        .position_pid = { kPosKp, kPosKi, kPosKd, 400.0f },
        .velocity_pid = { kVelKp, kVelKi, kVelKd, kMaxCurrent },
        .pos_vel_freq_ratio = 1,
    });
    motors::DJIMotor::CAN_FilterInit(&hcan1, 0);
    CAN_RegisterCallback(&hcan1, motors::DJIMotor::CANBaseReceiveCallback);
    CAN_InitMainCallback(&hcan1);
    CAN_Start(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    INIT_OBJECT(g_uart_rx, &huart1);
    if (HAL_UART_RegisterCallback(&huart1, HAL_UART_ERROR_CB_ID,
                [](UART_HandleTypeDef*) { g_uart_rx->errorFromISR(); }) != HAL_OK)
    {
        stopSystem(false);
        Error_Handler();
    }
    if (!g_uart_rx->startReceive())
        g_uart_rx->errorFromISR(); // 由控制任务重试。
    const osThreadAttr_t attributes = {
        .name = "ControlTask", .stack_size = 1024 * 4,
        .priority = static_cast<osPriority_t>(osPriorityHigh),
    };
    if (osThreadNew(ControlTask, nullptr, &attributes) == nullptr)
    {
        stopSystem(false);
        motors::DJIMotor::SendIqCommand(&hcan1, motors::DJIMotor::IqSetCMDGroup::IqCMDGroup_1_4);
        Error_Handler();
    }
    osThreadExit();
}
