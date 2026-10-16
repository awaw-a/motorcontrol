#pragma once

#include "dma_buffer.hpp"
#include "main.h"
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

// 所有解析和 HAL 恢复均由唯一的控制任务调用；ISR 只设置错误标志。
class UpperUart
{
public:
    struct Command
    {
        uint8_t mode{};
        int16_t val_a{};
        int16_t val_b{};
        uint32_t action_id{};
        bool operator==(const Command& other) const
        {
            return mode == other.mode && val_a == other.val_a && val_b == other.val_b &&
                   action_id == other.action_id;
        }
        bool isStop() const { return mode == 0x00 && val_a == 0 && val_b == 0; }
    };

    static constexpr size_t kFrameLen = 8;
    static constexpr size_t kRxCapacity = 256;
    static constexpr uint32_t kConnectionTimeoutMs = 400;
    // 115200/8N1 在 10 ms 内最多约 116 字节，留出 DMA 覆写余量。
    static constexpr uint32_t kMaxPollGapMs = 10;

    explicit UpperUart(UART_HandleTypeDef* uart) : uart_(uart) {}
    bool startReceive();
    void poll(uint32_t now_ms);
    void errorFromISR() { error_pending_.store(true, std::memory_order_relaxed); }
    bool isConnected(uint32_t now_ms) const;
    bool getCommand(Command& out, uint32_t now_ms) const;
    uint32_t sequence() const { return sequence_; }
    uint32_t faultCount() const { return faults_; }
    bool sendFeedback(uint8_t status, float velocity, float position, uint32_t now_ms);
    static int16_t saturateInt16(float value);
    static std::array<uint8_t, kFrameLen> encodeFeedback(uint8_t status, float velocity,
                                                        float position);

private:
    bool recover(uint32_t now_ms);
    void parseByte(uint8_t byte, uint32_t now_ms);
    void invalidate();
    size_t writePosition() const;

    UART_HandleTypeDef* uart_;
    memory::DMABuffer<kRxCapacity> rx_{};
    memory::DMABuffer<kFrameLen> tx_{};
    std::array<uint8_t, kFrameLen> frame_{};
    size_t read_pos_ = 0;
    size_t frame_used_ = 0;
    Command command_{};
    bool command_valid_ = false;
    bool started_ = false;
    bool attempted_ = false;
    bool tx_started_ = false;
    uint32_t last_valid_ms_ = 0;
    uint32_t last_byte_ms_ = 0;
    uint32_t last_poll_ms_ = 0;
    uint32_t last_attempt_ms_ = 0;
    uint32_t tx_start_ms_ = 0;
    uint32_t sequence_ = 0;
    uint32_t faults_ = 0;
    std::atomic_bool error_pending_{ false };
};
