#pragma once
#include <atomic>
#include <cstdint>

namespace wheel_encoder {

class WheelEncoder {
public:
    explicit WheelEncoder(int gpio_pin);
    ~WheelEncoder();

    bool initialize();
    uint32_t getTickCount() const;
    uint32_t getDeltaTicks();
    void resetTickCount();

private:
    int gpio_pin_;
    std::atomic<uint32_t> tick_count_;
    std::atomic<uint32_t> last_tick_count_;
    std::atomic<uint32_t> ticks_since_reset_;
    bool initialized_;

    void checkAndResetIfNeeded();
};

}  // namespace wheel_encoder
