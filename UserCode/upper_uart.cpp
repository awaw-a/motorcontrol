#include "upper_uart.hpp"
#include "crc.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>

namespace
{
using CRC8 = crc::CRCX<8, 0x07, 0x00, false, false, 0x00>;
int16_t readInt16(const uint8_t* p)
{
    const uint16_t value = static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
    return static_cast<int16_t>(value);
}
}

void UpperUart::invalidate()
{
    command_valid_ = false;
    frame_used_ = 0;
}

bool UpperUart::recover(uint32_t now_ms)
{
    invalidate();
    started_ = false;
    attempted_ = true;
    last_attempt_ms_ = now_ms;
    tx_started_ = false;
    if (uart_ == nullptr || uart_->hdmarx == nullptr || uart_->hdmatx == nullptr ||
        uart_->hdmarx->Init.Mode != DMA_CIRCULAR)
        return false;
    // 本工程 HAL 的 DMA 错误可能同时结束 RX/TX，在线程中统一恢复两侧。
    if (HAL_UART_Abort(uart_) != HAL_OK)
        return false;
    __HAL_UART_CLEAR_OREFLAG(uart_);
    read_pos_ = 0;
    if (HAL_UART_Receive_DMA(uart_, rx_, kRxCapacity) != HAL_OK)
        return false;
    started_ = true;
    last_poll_ms_ = now_ms;
    return true;
}

bool UpperUart::startReceive() { return recover(HAL_GetTick()); }

size_t UpperUart::writePosition() const
{
    return (kRxCapacity - __HAL_DMA_GET_COUNTER(uart_->hdmarx)) % kRxCapacity;
}

void UpperUart::poll(uint32_t now_ms)
{
    bool failed = error_pending_.exchange(false, std::memory_order_relaxed);
    if (started_)
    {
        failed |= uart_->RxState != HAL_UART_STATE_BUSY_RX;
        if (uart_->gState == HAL_UART_STATE_READY)
            tx_started_ = false;
        else if (tx_started_ && now_ms - tx_start_ms_ >= 50)
            failed = true;
    }
    if (failed)
    {
        invalidate();
        started_ = false;
        attempted_ = false;
        ++faults_;
    }
    if (!started_)
    {
        if (!attempted_ || now_ms - last_attempt_ms_ >= 10)
            recover(now_ms);
        return;
    }

    const size_t end = writePosition();
    if (now_ms - last_poll_ms_ > kMaxPollGapMs)
    {
        // 超过采样预算时不猜测 DMA 是否已绕圈，丢弃旧数据并要求新指令。
        read_pos_ = end;
        last_poll_ms_ = now_ms;
        invalidate();
        ++faults_;
        return;
    }
    last_poll_ms_ = now_ms;
    if (command_valid_ && now_ms - last_valid_ms_ >= kConnectionTimeoutMs)
        invalidate();
    if (frame_used_ != 0 && now_ms - last_byte_ms_ >= 20)
        frame_used_ = 0; // 不把拔插前的半帧与很久以后的字节拼接。
    std::array<uint8_t, kRxCapacity> copy{};
    const size_t count = (end + kRxCapacity - read_pos_) % kRxCapacity;
    const volatile uint8_t* dma_data = rx_.data();
    __DMB();
    for (size_t i = 0; i < count; ++i)
        copy[i] = dma_data[(read_pos_ + i) % kRxCapacity];
    read_pos_ = end;
    if (error_pending_.load(std::memory_order_relaxed))
        return;
    for (size_t i = 0; i < count; ++i)
        parseByte(copy[i], now_ms);
}

void UpperUart::parseByte(uint8_t byte, uint32_t now_ms)
{
    last_byte_ms_ = now_ms;
    frame_[frame_used_++] = byte;
    while (frame_used_ >= 2 && (frame_[0] != 0xAA || frame_[1] != 0xBB))
    {
        --frame_used_;
        std::memmove(frame_.data(), frame_.data() + 1, frame_used_);
    }
    if (frame_used_ != kFrameLen)
        return;
    if (CRC8::calc(frame_.data(), kFrameLen - 1) != frame_.back())
    {
        --frame_used_;
        std::memmove(frame_.data(), frame_.data() + 1, frame_used_);
        return;
    }
    Command next{};
    next.mode = frame_[2];
    if (next.mode == 0x22)
    {
        for (size_t i = 0; i < 4; ++i)
            next.action_id |= static_cast<uint32_t>(frame_[3 + i]) << (8 * i);
    }
    else
    {
        next.val_a = readInt16(&frame_[3]);
        next.val_b = readInt16(&frame_[5]);
    }
    const bool valid = next.mode == 0x00 || next.mode == 0x11 ||
                       (next.mode == 0x22 &&
                        (next.action_id == 0 || next.action_id == 0x11111111U));
    frame_used_ = 0;
    if (!valid)
    {
        invalidate();
        ++faults_;
        return;
    }
    command_ = next;
    command_valid_ = true;
    last_valid_ms_ = now_ms;
    ++sequence_;
}

bool UpperUart::isConnected(uint32_t now_ms) const
{
    return started_ && command_valid_ && !error_pending_.load(std::memory_order_relaxed) &&
           now_ms - last_valid_ms_ < kConnectionTimeoutMs;
}

bool UpperUart::getCommand(Command& out, uint32_t now_ms) const
{
    if (!isConnected(now_ms))
        return false;
    out = command_;
    return true;
}

int16_t UpperUart::saturateInt16(float value)
{
    if (!std::isfinite(value))
        return 0;
    return static_cast<int16_t>(std::lround(std::clamp(value, -32768.0f, 32767.0f)));
}

std::array<uint8_t, UpperUart::kFrameLen>
UpperUart::encodeFeedback(uint8_t status, float velocity, float position)
{
    std::array<uint8_t, kFrameLen> frame{ 0xBB, 0xAA, status };
    const uint16_t speed = static_cast<uint16_t>(saturateInt16(velocity));
    const uint16_t angle = static_cast<uint16_t>(saturateInt16(position));
    frame[3] = static_cast<uint8_t>(speed);
    frame[4] = static_cast<uint8_t>(speed >> 8);
    frame[5] = static_cast<uint8_t>(angle);
    frame[6] = static_cast<uint8_t>(angle >> 8);
    frame[7] = CRC8::calc(frame.data(), kFrameLen - 1);
    return frame;
}

bool UpperUart::sendFeedback(uint8_t status, float velocity, float position, uint32_t now_ms)
{
    if (!started_ || uart_->gState != HAL_UART_STATE_READY)
        return false;
    const auto frame = encodeFeedback(status, velocity, position);
    std::copy(frame.begin(), frame.end(), tx_.data());
    if (HAL_UART_Transmit_DMA(uart_, tx_, kFrameLen) != HAL_OK)
    {
        error_pending_.store(true, std::memory_order_relaxed);
        return false;
    }
    tx_started_ = true;
    tx_start_ms_ = now_ms;
    return true;
}
