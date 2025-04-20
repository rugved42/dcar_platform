#include "drive_base.hpp"
#include "wheel_encoder/wheel_encoder.hpp"
#include "wheel_encoder/tick_delta_tracker.hpp"

#include <unistd.h> // for sleep
#include <iostream>
#include <chrono>
#include <thread>

int main() {
    DriveBase drive;
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // Initialize encoder
    wheel_encoder::WheelEncoder encoder(13);
    if (!encoder.initialize()) {
        std::cerr << "Failed to initialize encoder." << std::endl;
        return 1;
    }

    wheel_encoder::WheelEncoderTickDeltaTracker tracker;
    tracker.update(encoder.getTickCount()); // initialize baseline

    std::cout << "Monitoring wheel encoder ticks..." << std::endl;
    drive.setSpeed(SAFE_FORWARD_SPEED);
    sleep(2.0);
    for (int i = 0; i < 10; ++i) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        int current_ticks = encoder.getTickCount();
        int delta = tracker.update(current_ticks);
        std::cout << "[Tick " << i << "] Total: " << current_ticks << ", Delta: " << delta << std::endl;
    }
    drive.stop();
    sleep(2.0);
    // std::cout << "Starting keyboard control mode...\n";
    // drive.keyboardControl();

    return 0;
}
