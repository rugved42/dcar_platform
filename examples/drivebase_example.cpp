#include "drive_base.hpp"
#include <unistd.h> // sleep
#include <iostream>

int main() {
    DriveBase drive;

    drive.setSteering(0.0f);  // Center
    sleep(2);

    // drive.goLeft();
    // sleep(2);

    // drive.goRight();
    // sleep(2);

    // drive.goStraight();  // Center
    // sleep(2);

    for (double i = 1.0; i < 2.0; i+=0.1)
    {
        drive.setSpeedPWM(i);
        sleep(2);
        drive.setSpeedPWM(1.5f);
        sleep(2);
    }


    // drive.stop();     // Slow forward
    // sleep(2);

    drive.goReverse();     // Stop
    sleep(2);

    drive.stop();     // Fast forward
    sleep(2);
}
