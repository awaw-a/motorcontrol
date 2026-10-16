#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <vector>
#include "crc.hpp"
#include "app.cpp"

GPIO_TypeDef stub_gpio;
CAN_HandleTypeDef hcan1;
DMA_HandleTypeDef rx_dma, tx_dma;
UART_HandleTypeDef huart1{ &rx_dma, &tx_dma };
namespace test
{
uint32_t now = 0;
uint8_t* rx = nullptr;
size_t rx_size = 0, rx_offset = 0;
bool fail_rx = false, fail_can = false, fail_abort = false;
bool auto_complete = true;
uint32_t free_mailboxes = 3;
unsigned uart_aborts = 0, can_aborts = 0, assertions = 0;
GPIO_PinState led = GPIO_PIN_RESET;
std::vector<std::array<uint8_t, 8>> sent;
std::array<void(*)(CAN_HandleTypeDef*), 8> can_callbacks{};
void check(bool condition, const char* message)
{
    ++assertions;
    if (!condition)
    {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(1);
    }
}
uint8_t crc(const uint8_t* data, size_t size)
{
    uint8_t value = 0;
    for (size_t i = 0; i < size; ++i)
    {
        value ^= data[i];
        for (int bit = 0; bit < 8; ++bit)
            value = value & 0x80 ? static_cast<uint8_t>((value << 1) ^ 7)
                                 : static_cast<uint8_t>(value << 1);
    }
    return value;
}
std::vector<uint8_t> command(uint8_t mode, int16_t a, int16_t b)
{
    std::vector<uint8_t> f{0xAA, 0xBB, mode, uint8_t(a), uint8_t(a >> 8),
                           uint8_t(b), uint8_t(b >> 8), 0};
    f[7] = crc(f.data(), 7);
    return f;
}
void deliver(const std::vector<uint8_t>& bytes)
{
    check(rx != nullptr, "RX DMA has been started");
    for (uint8_t byte : bytes)
    {
        rx[rx_offset++] = byte;
        if (rx_offset == rx_size)
            rx_offset = 0;
        rx_dma.counter = static_cast<uint32_t>(rx_size - rx_offset);
    }
}
int16_t lastCurrent()
{
    check(!sent.empty(), "CAN current frame was sent");
    return static_cast<int16_t>((sent.back()[0] << 8) | sent.back()[1]);
}
void feedback(int raw = 0, int rpm = 0)
{
    CAN_RxHeaderTypeDef h{};
    h.StdId = 0x201;
    h.DLC = 8;
    const uint8_t data[]{uint8_t(raw >> 8), uint8_t(raw), uint8_t(rpm >> 8),
                         uint8_t(rpm), 0, 0, 0, 0};
    motors::DJIMotor::CANBaseReceiveCallback(&hcan1, &h, data);
}
void step(bool motor = true, bool late = false)
{
    ++now;
    if (motor)
        feedback();
    controlStep(now, late);
}
void resetApp()
{
    fail_rx = fail_can = fail_abort = false;
    auto_complete = true;
    free_mailboxes = 3;
    ++now;
    feedback();
    g_motor->resetAngle(); // 隔离测试用例；生产程序不会在重连时悄悄改零点。
    g_vel_ctrl->disable();
    g_pos_ctrl->disable();
    g_uart_rx->startReceive();
    g_uart_rx->poll(now);
    g_safety = {};
    g_safety.uart_faults = g_uart_rx->faultCount();
    g_preset = {};
    g_reached.reset();
    g_have_last_command = false;
}
class MockMotor : public motors::IMotor
{
public:
    float current = 0;
    float getAngle() const override { return 0; }
    float getVelocity() const override { return 0; }
    void resetAngle() override {}
    bool supportsCurrent() const override { return true; }
    void setCurrent(float value) override { current = value; }
};
}

uint32_t HAL_GetTick() { return test::now; }
HAL_StatusTypeDef HAL_UART_Abort(UART_HandleTypeDef* uart)
{
    ++test::uart_aborts;
    test::rx = nullptr;
    uart->gState = HAL_UART_STATE_READY;
    uart->RxState = HAL_UART_STATE_READY;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef* uart, uint8_t* data, uint16_t size)
{
    if (test::fail_rx)
        return HAL_BUSY;
    test::rx = data;
    test::rx_size = size;
    test::rx_offset = 0;
    uart->hdmarx->counter = size;
    uart->RxState = HAL_UART_STATE_BUSY_RX;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef* uart, uint8_t*, uint16_t)
{
    uart->gState = HAL_UART_STATE_BUSY_TX;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_RegisterCallback(UART_HandleTypeDef*, uint32_t, UartCB) { return HAL_OK; }
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef*, const CAN_FilterTypeDef*) { return HAL_OK; }
HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef*, uint32_t)
{
    ++test::can_aborts;
    if (test::fail_abort)
        return HAL_ERROR;
    test::free_mailboxes = 3;
    return HAL_OK;
}
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef*) { return test::free_mailboxes; }
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef*, const CAN_TxHeaderTypeDef* header,
                                      const uint8_t* data, uint32_t* mailbox)
{
    if (test::fail_can || test::free_mailboxes == 0)
        return HAL_ERROR;
    test::check(header->TransmitGlobalTime == 0, "CAN header is initialized");
    std::array<uint8_t, 8> copy{};
    std::copy_n(data, 8, copy.begin());
    test::sent.push_back(copy);
    *mailbox = 1;
    if (!test::auto_complete)
        --test::free_mailboxes;
    return HAL_OK;
}
uint32_t HAL_CAN_GetRxFifoFillLevel(CAN_HandleTypeDef*, uint32_t) { return 0; }
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef*, uint32_t, CAN_RxHeaderTypeDef*, uint8_t*) { return HAL_ERROR; }
HAL_StatusTypeDef HAL_CAN_RegisterCallback(CAN_HandleTypeDef*, uint32_t id, void(*cb)(CAN_HandleTypeDef*))
{
    test::can_callbacks[id] = cb;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef*) { return HAL_OK; }
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef*, uint32_t) { return HAL_OK; }
void HAL_GPIO_WritePin(GPIO_TypeDef*, uint16_t, GPIO_PinState value) { test::led = value; }
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef*, uint16_t) { return test::led; }
void HAL_GPIO_TogglePin(GPIO_TypeDef*, uint16_t) {}
void Error_Handler() { test::check(false, "unexpected Error_Handler"); }
uint32_t osKernelGetTickCount() { return test::now; }
int osDelayUntil(uint32_t) { return osOK; }
osThreadId_t osThreadNew(void(*)(void*), void*, const osThreadAttr_t*) { return reinterpret_cast<void*>(1); }
void osThreadExit() {}

int main()
{
    using namespace test;
    Init(nullptr);
    deliver(command(0, 8000, 200));
    step(false);
    check(lastCurrent() == 0 && !g_motor->currentController(), "offline motor cannot receive torque");
    feedback(6000);
    check(g_motor->getAngle() == 0, "first valid feedback establishes zero");
    const float ratio = USE_M2006_MOTOR ? 36.0f : 3591.0f / 187.0f;
    for (int i = 1; i <= 60; ++i)
    {
        ++now;
        feedback((6000 + i * 100) % 8192);
    }
    check(std::fabs(g_motor->getAngle() + 6000.0f * 360 / 8192 / ratio) < .01f,
          "no frame-50 zero jump and angle unwrap uses gearbox ratio");

    resetApp();
    g_motor->setCurrent(50000);
    check(std::abs(g_motor->getIqCMD()) <= kMaxCurrent, "driver enforces the configured final current limit");
    deliver(command(0, -32768, 32767));
    step();
    check(std::abs(lastCurrent()) <= kMaxCurrent, "int16 minimum current parameter cannot bypass the limit");
    resetApp();
    deliver({0x55});
    auto valid = command(0, 3000, -200);
    deliver(valid);
    deliver(valid);
    const auto before = g_uart_rx->sequence();
    check(!g_uart_rx->isConnected(now), "ISR/DMA cannot publish an unparsed command");
    step();
    check(g_uart_rx->sequence() == before + 2, "noise prefix preserves both valid frames");
    UpperUart::Command decoded{};
    check(g_uart_rx->getCommand(decoded, now) && decoded.val_b == -200, "signed little-endian command");
    auto bad = valid;
    bad[7] ^= 1;
    deliver(bad);
    deliver(valid);
    step();
    check(g_uart_rx->sequence() == before + 3, "bad CRC is discarded and next frame is recovered");
    for (int i = 0; i < 200; ++i)
    {
        deliver({0, 0xAA, 0xAA});
        deliver(valid);
        step();
    }
    check(g_uart_rx->sequence() == before + 203, "DMA wrap and repeated overlapping headers");

    resetApp();
    const auto partial_sequence = g_uart_rx->sequence();
    deliver(std::vector<uint8_t>(valid.begin(), valid.begin() + 4));
    step();
    for (int i = 0; i < 21; ++i)
        step();
    deliver(std::vector<uint8_t>(valid.begin() + 4, valid.end()));
    step();
    check(g_uart_rx->sequence() == partial_sequence, "stale half-frame cannot be completed after reconnect delay");
    deliver(valid);
    step();
    check(g_uart_rx->sequence() == partial_sequence + 1, "new complete frame recovers after stale partial data");

    auto frame = UpperUart::encodeFeedback(0xAF, -200, 301);
    check(frame.size() == 8 && frame[0] == 0xBB && frame[1] == 0xAA && frame[2] == 0xAF &&
          frame[3] == 0x38 && frame[4] == 0xFF && frame[5] == 0x2D && frame[6] == 1 &&
          frame[7] == test::crc(frame.data(), 7), "feedback remains exactly 8-byte wire compatible");
    check(UpperUart::saturateInt16(40000) == 32767 && UpperUart::saturateInt16(-40000) == -32768 &&
          UpperUart::saturateInt16(std::numeric_limits<float>::quiet_NaN()) == 0, "feedback saturates without wrap");

    resetApp();
    deliver(command(0, 3000, 200));
    step();
    check(lastCurrent() != 0 && led == GPIO_PIN_SET, "valid command enables the online motor");
    for (int i = 0; i < 400; ++i)
        step();
    check(lastCurrent() == 0 && g_safety.fault_latched && led == GPIO_PIN_RESET,
          "400 ms UART timeout stops and latches");
    deliver(command(0, 3000, 200));
    step();
    check(lastCurrent() == 0, "reconnected old motion command cannot rearm");
    deliver(command(0, 0, 0));
    step();
    check(!g_safety.fault_latched && lastCurrent() == 0, "existing emergency-stop frame acknowledges recovery");
    deliver(command(0, 3000, 200));
    step();
    check(lastCurrent() != 0, "new motion command after acknowledgement runs");
    step(true, true);
    check(lastCurrent() == 0 && g_safety.fault_latched, "control deadline fault stops immediately");

    resetApp();
    deliver(command(0, 3000, 200));
    step();
    deliver(command(0x33, 0, 0));
    step();
    check(lastCurrent() == 0 && g_safety.fault_latched && g_status == 0xFF, "unknown mode cannot retain stale status or motion");
    resetApp();
    deliver(command(0x22, 1, 2));
    step();
    check(!g_uart_rx->isConnected(now) && lastCurrent() == 0, "unknown action ID is rejected");

    resetApp();
    deliver(command(0, 3000, 200));
    step();
    const auto aborts = uart_aborts;
    g_uart_rx->errorFromISR();
    check(uart_aborts == aborts, "UART ISR never executes blocking recovery");
    step();
    check(uart_aborts > aborts && lastCurrent() == 0 && g_safety.fault_latched,
          "task restores both UART directions and stops on DMA error");
    fail_rx = true;
    check(!g_uart_rx->startReceive(), "RX start failure is visible");
    for (int i = 0; i < 11; ++i)
        step();
    fail_rx = false;
    for (int i = 0; i < 11; ++i)
        step();
    deliver(command(0, 0, 0));
    step();
    check(g_uart_rx->isConnected(now), "failed RX start automatically retries");
    const auto faults = g_uart_rx->faultCount();
    check(g_uart_rx->sendFeedback(0xAF, 0, 0, now), "feedback DMA starts");
    for (int i = 0; i < 51; ++i)
        step();
    check(g_uart_rx->faultCount() > faults && huart1.gState == HAL_UART_STATE_READY,
          "stuck TX DMA times out and restores UART");
    deliver(valid);
    now += 11;
    feedback();
    controlStep(now, true);
    check(!g_uart_rx->isConnected(now) && lastCurrent() == 0, "ambiguous DMA backlog is discarded after a long poll gap");

    resetApp();
    g_preset = {true, 0x11111111, 0, 1, {}};
    const motors::DJIMotor::Feedback at720{720, 0, true, true, true};
    const motors::DJIMotor::Feedback away{600, 0, true, true, true};
    runPresetPosition(0, at720);
    runPresetPosition(2999, at720);
    runPresetPosition(3000, away);
    check(g_preset.stage == 1, "leaving 720 degrees resets hold");
    runPresetPosition(3001, at720);
    runPresetPosition(6000, at720);
    check(g_preset.stage == 1, "full uninterrupted hold is required");
    runPresetPosition(6001, at720);
    check(g_preset.stage == 2, "continuous 3 second hold advances");
    const motors::DJIMotor::Feedback at360{360, 0, true, true, true};
    runPresetPosition(7000, at360);
    runPresetPosition(7999, at360);
    check(g_preset.stage == 2, "360 degree hold does not finish early");
    runPresetPosition(8000, at360);
    check(g_preset.stage == 3, "continuous 1 second hold advances");
    g_preset = {true, 0, 0, 0, {}};
    const motors::DJIMotor::Feedback moving{0, 100, true, true, true};
    const motors::DJIMotor::Feedback resting{0, 0, true, true, true};
    check(runPresetSpeed(9000, moving) == 0xAA, "speed action cannot finish while still moving");
    check(runPresetSpeed(9100, resting) == 0xAA && runPresetSpeed(9150, resting) == 0xAF,
          "speed action finishes only after settling at zero");

    MockMotor motor;
    controllers::MotorVelController vel(&motor, {{35, .6f, 0, 8000}});
    vel.enable();
    vel.setRef(200, 1000);
    for (int i = 0; i < 1000; ++i)
        vel.update();
    check(std::fabs(motor.current) <= 1000, "dynamic current limit is enforced");
    vel.setRef(0, 1000);
    vel.update();
    check(motor.current == 0, "saturated speed command does not wind up behind the dynamic limit");
    vel.disable();
    controllers::MotorPosController pos(&motor, {{2, 0, .4f, 400}, {35, .6f, 0, 8000}, 1});
    pos.enable();
    pos.setRef(10);
    for (int i = 0; i < 100; ++i)
        pos.update();
    pos.disable();
    check(motor.current == 0, "controller disable clears cached current");
    pos.setRef(0);
    pos.enable();
    pos.update();
    pos.update();
    check(motor.current == 0, "position reenable clears both PID states");

    resetApp();
    deliver(command(0, 3000, 200));
    step();
    for (int i = 0; i < 20; ++i)
        step(false);
    check(lastCurrent() == 0 && g_safety.fault_latched, "motor feedback timeout stops torque");
    CAN_RxHeaderTypeDef header{};
    header.StdId = 0x201;
    header.DLC = 0;
    header.RTR = CAN_RTR_REMOTE;
    std::array<uint8_t, 8> data{};
    motors::DJIMotor::CANBaseReceiveCallback(&hcan1, &header, data.data());
    check(!g_motor->isConnected(), "remote/short CAN frame cannot refresh connection");
    header.RTR = CAN_RTR_DATA;
    header.DLC = 8;
    data[0] = 0x20;
    motors::DJIMotor::CANBaseReceiveCallback(&hcan1, &header, data.data());
    check(!g_motor->isConnected(), "invalid encoder range cannot refresh connection");
    feedback();
    check(g_motor->isConnected() && !g_motor->getFeedback().position_valid,
          "reconnection does not invent lost turns or silently redefine zero");

    resetApp();
    free_mailboxes = 0;
    CAN_TxHeaderTypeDef tx{};
    tx.DLC = 8;
    data.fill(0x7F);
    check(CAN_SendMessage(&hcan1, &tx, data.data()) != CAN_SEND_FAILED, "queued CAN frame is accepted");
    check(CAN_DiscardPendingTx(&hcan1), "emergency stop requests hardware abort");
    const auto sent_before = sent.size();
    can_callbacks[HAL_CAN_TX_MAILBOX0_COMPLETE_CB_ID](&hcan1);
    check(sent.size() == sent_before, "emergency stop removes queued stale current frames");
    fail_can = true;
    check(CAN_SendMessage(&hcan1, &tx, data.data()) == CAN_SEND_FAILED && CAN_TakeTxError(&hcan1),
          "CAN send error is reported to task without hanging");
    check(!CAN_TakeTxError(&hcan1), "CAN error latch is consumed once");
    fail_can = false;
    CAN_HandleTypeDef other;
    check(motors::DJIMotor::SendIqCommand(&other, motors::DJIMotor::IqSetCMDGroup::IqCMDGroup_1_4) == CAN_SEND_FAILED,
          "unregistered CAN handle cannot dereference a null motor map");

    resetApp();
    now = std::numeric_limits<uint32_t>::max() - 5;
    g_uart_rx->startReceive();
    deliver(command(0, 0, 0));
    g_uart_rx->poll(now);
    for (int i = 0; i < 10; ++i)
        g_uart_rx->poll(++now);
    check(g_uart_rx->isConnected(now), "UART connection timestamps survive uint32 wrap");
    std::cout << "PASS: " << assertions << " assertions (production parser, controllers, DJI, CAN and application)\n";
}
