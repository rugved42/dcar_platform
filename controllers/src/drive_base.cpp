// drive_base.cpp
#include "drive_base.hpp"
#include <iostream>
#include <cmath>
#include <stdexcept>
#include <unistd.h> // sleep


DriveBase::DriveBase()
    : p_pca_driver(new PCA9685Driver) 
    {
      if (!p_pca_driver->initialize())
      {
        std::string err = "Could not initalize the motor driver";
        printf(err.c_str()); 
        throw std::runtime_error(err.c_str());
      }
    }

void DriveBase::setSpeed(float mps) {
    // Map speed in m/s to PWM pulse width
    float ms = 1.5 + (mps * 0.5);
    setSpeedPWM(ms);
}

void DriveBase::setSteering(float angle) {
    float ms = 1.5 + (angle * 0.3);
    setSteeringPWM(ms);
}

void DriveBase::goLeft() {
    std::cout << "turn left" << std::endl;
    setSteeringPWM(2.0);
}

void DriveBase::goRight() {
    std::cout << "turn right" << std::endl;
    setSteeringPWM(1.1);
}

void DriveBase::goStraight() {
  std::cout << "Going straight" << std::endl;
  setSteeringPWM(1.5);
  sleep(2);
  setSpeedPWM(1.6);

}

void DriveBase::stop() {
  std::cout << "Stopping" << std::endl;
  setSpeedPWM(0.0);
}

void DriveBase::goReverse() {
  setSpeedPWM(0.220);
}

void DriveBase::setSpeedPWM(float ms) {
    p_pca_driver->setPulseMs(SPEED_CHANNEL, ms);
}

void DriveBase::setSteeringPWM(float ms) {
  p_pca_driver->setPulseMs(STEERING_CHANNEL, ms);
}

DriveBase::~DriveBase()
{
  delete p_pca_driver;
}