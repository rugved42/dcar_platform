// drive_base.hpp
#pragma once

#include <string>
#include "pca9685_driver/pca9685_driver.hpp"
#include <chrono>

// Can be mentioned in a calibration file
const int STEERING_CHANNEL = 1;
const int SPEED_CHANNEL = 0;
// Calibrated forward and reverse speed
const float SAFE_FORWARD_SPEED = 0.7f;
const float SAFE_REVERSE_SPEED = -1.38f;
// steering
const float STEERING_ACTUAL_CENTER = 1.425f;
const float STEERING_ACTUAL_RIGHT = 1.0f;
const float STEERING_ACTUAL_LEFT = 1.9f;

class DriveBase {
public:
    DriveBase();

    bool setSpeed(float mps);       // m/s input
    bool setSteering(float angle);  // normalized input: -1 (left) to 1 (right)

    void turnLeft();
    void turnRight();

    void goStraight();
    void goReverse();

    void stop();
    void setNeutral();
    void centerSteering();

    float mapSteering(float value);
    void calibrateSteering();

    void keyboardControl();
    ~DriveBase();

private:
    int fd_;
    float ms_per_tick_;
    float steering_center_ms_ = STEERING_ACTUAL_CENTER;
    float steering_min_ms = STEERING_ACTUAL_RIGHT;     // Full right
    float steering_max_ms = STEERING_ACTUAL_LEFT;     // Full left
    float previous_speed_mps;
    std::chrono::steady_clock::time_point last_speed_change_ms;
    PCA9685Driver* p_pca_driver;
    void setSpeedPWM(float ms);
    void setSteeringPWM(float ms);
    bool isOppositeDirection(float curr, float last);
    char helperGetch();

};
