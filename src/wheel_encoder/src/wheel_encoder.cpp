#include "wheel_encoder/wheel_encoder.hpp"
#include <gpiod.h>
#include <iostream>
#include <thread>

namespace wheel_encoder {

constexpr uint32_t RESET_THRESHOLD = 100'000'000;

WheelEncoder::WheelEncoder(int gpio_pin)
    : gpio_pin_(gpio_pin),
      tick_count_(0),
      last_tick_count_(0),
      ticks_since_reset_(0),
      initialized_(false) {}

WheelEncoder::~WheelEncoder() {}

bool WheelEncoder::initialize() {
    gpiod_chip* chip = gpiod_chip_open_by_name("gpiochip0");
    if (!chip) {
        std::cerr << "Failed to open gpiochip0\n";
        return false;
    }

    gpiod_line* line = gpiod_chip_get_line(chip, gpio_pin_);
    if (!line) {
        std::cerr << "Failed to get line for GPIO " << gpio_pin_ << "\n";
        gpiod_chip_close(chip);
        return false;
    }

    if (gpiod_line_request_both_edges_events(line, "wheel_encoder") < 0) {
        std::cerr << "Failed to request event on GPIO " << gpio_pin_ << "\n";
        gpiod_chip_close(chip);
        return false;
    }

    std::thread([this, line]() {
        while (true) {
            gpiod_line_event event;
            if (gpiod_line_event_wait(line, nullptr) == 1 &&
                gpiod_line_event_read(line, &event) == 0) {
                tick_count_++;
                ticks_since_reset_++;
                checkAndResetIfNeeded();
            }
        }
    }).detach();

    initialized_ = true;
    return true;
}

uint32_t WheelEncoder::getTickCount() const {
    return tick_count_.load();
}

uint32_t WheelEncoder::getDeltaTicks() {
    uint32_t current = tick_count_.load();
    uint32_t previous = last_tick_count_.exchange(current);

    // Handle overflow using unsigned arithmetic
    return current - previous;
}

void WheelEncoder::resetTickCount() {
    tick_count_ = 0;
    last_tick_count_ = 0;
    ticks_since_reset_ = 0;
}

void WheelEncoder::checkAndResetIfNeeded() {
    if (ticks_since_reset_.load() >= RESET_THRESHOLD) {
        std::cout << "[WheelEncoder] Soft resetting tick counters." << std::endl;
        resetTickCount();
    }
}

}  // namespace wheel_encoder
