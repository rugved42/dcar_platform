// drive_base.hpp
#pragma once

#include <string>
#include "pca9685_driver/pca9685_driver.hpp"

const int STEERING_CHANNEL = 1;
const int SPEED_CHANNEL = 0;

class DriveBase {
public:
    DriveBase();

    void setSpeed(float mps);       // m/s input
    void setSteering(float angle);  // normalized input: -1 (left) to 1 (right)

    void goLeft();
    void goRight();
    void goStraight();

    ~DriveBase();

private:
    int fd_;
    float ms_per_tick_;
    PCA9685Driver* p_pca_driver;

    void setSpeedPWM(float ms);
    void setSteeringPWM(float ms);

};
