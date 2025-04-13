// drive_base.cpp
#include "drive_base.hpp"
#include <iostream>
#include <cmath>
#include <stdexcept>
#include <unistd.h> // sleep
#include <thread>
#include <termios.h>
#include <unistd.h>


DriveBase::DriveBase()
    : p_pca_driver(new PCA9685Driver),
      previous_speed_mps{0.0},
      last_speed_change_ms{std::chrono::steady_clock::now()}
{
  if (!p_pca_driver->initialize())
  {
    std::string err = "Could not initalize the motor driver";
    printf(err.c_str()); 
    throw std::runtime_error(err.c_str());
  }
  // Center the steering
  p_pca_driver->setPulseMs(STEERING_CHANNEL, steering_center_ms_);
}

void DriveBase::centerSteering()
{
  p_pca_driver->setPulseMs(STEERING_CHANNEL, steering_center_ms_);
}


char DriveBase::helperGetch() {
  struct termios oldt, newt;
  char ch;
  tcgetattr(STDIN_FILENO, &oldt);          // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);        // disable buffering
  tcsetattr(STDIN_FILENO, TCSANOW, &newt); // apply new settings
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt); // restore old settings
  return ch;
}

bool DriveBase::isOppositeDirection(float curr, float last) {
  return (curr < 0 && last > 0.05) || (curr > 0 && last < -0.05);
}


bool DriveBase::setSpeed(float speed_mps) 
{
  bool reversal_requested = false;
  if (speed_mps < 0.0 || isOppositeDirection(speed_mps, previous_speed_mps))
  {
    reversal_requested = true;
  }
  float mps = std::max(-1.5f, std::min(5.0f, speed_mps));
  float ms = 1.5 + (mps * 0.1);
  if (reversal_requested)
  {
    setNeutral();
    usleep(400000);
  }
  setSpeedPWM(ms); // reverse
  last_speed_change_ms = std::chrono::steady_clock::now();
  previous_speed_mps = mps;
  usleep(300000);
  return true;
}

void DriveBase::setNeutral()
{
  setSpeedPWM(1.5f);
  setSteeringPWM(1.5f);
}

float DriveBase::mapSteering(float value) {
  // Clamp input to [-1.0, 1.0]
  value = std::max(-1.0f, std::min(1.0f, value));

  // Interpolate
  float pulse = steering_center_ms_ + (value * (steering_max_ms - steering_center_ms_));
  return pulse;
}

bool DriveBase::setSteering(float normalized_value) 
{
  float pulse = mapSteering(normalized_value);
  setSteeringPWM(pulse);
  usleep(300000);
  return true;
}

void DriveBase::turnLeft() {
    std::cout << "turn left" << std::endl;
    setSteeringPWM(2.0);
}

void DriveBase::turnRight() 
{
  std::cout << "turn right" << std::endl;
  setSteeringPWM(1.0);
}

void DriveBase::goStraight() 
{
  std::cout << "Going straight" << std::endl;
  setSpeed(0.7f);
}

void DriveBase::stop() 
{
  std::cout << "Stopping" << std::endl;
  setSpeedPWM(1.5f);
  usleep(300000);
}

void DriveBase::goReverse() {
  std::cout << "Going reverse" << std::endl;
  setSpeed(-1.38f);
}

void DriveBase::setSpeedPWM(float ms) {
    p_pca_driver->setPulseMs(SPEED_CHANNEL, ms);
}

void DriveBase::setSteeringPWM(float ms) {
  p_pca_driver->setPulseMs(STEERING_CHANNEL, ms);
}

void DriveBase::calibrateSteering()
{
  float ms = 1.5f;  // Starting from assumed center
  float step = 0.005f; // ~5 microseconds per step
  std::cout << "Use ← (a) and → (d) to adjust center. Press (q) to quit." << std::endl;

  while (true) {
      std::cout << "Sending pulse: " << ms << " ms\r" << std::flush;
      p_pca_driver->setPulseMs(1, ms); // Assuming channel 1 is steering

      char c = helperGetch();
      if (c == 'q') break;
      else if (c == 'a') ms -= step;     // Left
      else if (c == 'd') ms += step;     // Right

      // Clamp between 1.0 and 2.0 ms
      if (ms < 1.0f) ms = 1.0f;
      if (ms > 2.0f) ms = 2.0f;
  }
  std::cout << "\nFinal center ms: " << ms << std::endl;
}

void DriveBase::keyboardControl() {
  float speed = 0.0f;
  float steering = 0.0f;
  const float speed_step = 0.05f;
  const float steer_step = 0.1f;

  std::cout << "Use ↑ (w) and ↓ (s) for speed, ← (a) and → (d) for steering, (q) to quit.\n";

  while (true) {
      char c = helperGetch();
      if (c == 'q') break;
      else if (c == 'w') speed += speed_step;
      else if (c == 's') speed -= speed_step;
      else if (c == 'a') steering -= steer_step;
      else if (c == 'd') steering += steer_step;

      // Clamp values
      if (speed > 5.0f) speed = 5.0f;
      if (speed < -1.5f) speed = -1.5f;
      if (steering > 1.0f) steering = 1.0f;
      if (steering < -1.0f) steering = -1.0f;

      std::cout << "\rSpeed: " << speed << " m/s, Steering: " << steering << "   " << std::flush;

      setSpeed(speed);
      setSteering(steering);
  }

  setSpeed(0.0f);
  setSteering(0.0f);
  std::cout << "\nStopped.\n";
}

DriveBase::~DriveBase()
{
  delete p_pca_driver;
}