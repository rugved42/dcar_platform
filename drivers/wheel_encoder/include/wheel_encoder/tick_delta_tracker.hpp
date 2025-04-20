#pragma once

#include <cstdint>

namespace wheel_encoder {

class WheelEncoderTickDeltaTracker {
public:
    explicit WheelEncoderTickDeltaTracker(uint32_t max_tick_before_reset = 100'000'000)
        : last_tick_count_(0), max_tick_before_reset_(max_tick_before_reset) {}

    uint32_t update(uint32_t current_tick) {
        uint32_t delta = current_tick - last_tick_count_; 
        last_tick_count_ = current_tick;

        if (last_tick_count_ > max_tick_before_reset_) {
            last_tick_count_ = 0;
        }

        return delta;
    }

    void reset() {
        last_tick_count_ = 0;
    }

private:
    uint32_t last_tick_count_;
    uint32_t max_tick_before_reset_;
};

} // namespace wheel_encoder
