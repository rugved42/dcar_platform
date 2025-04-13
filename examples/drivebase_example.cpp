#include "drive_base.hpp"
#include <unistd.h> // sleep
#include <iostream>

int main() {
    DriveBase drive;
    sleep(2.0);
    // drive.setSteering(-1.0);
    // // sleep(1.5);
    // drive.setSpeed(SAFE_REVERSE_SPEED); // -1.38 is the appropriate speed
    // sleep(2); // hold

    // drive.setSpeed(SAFE_FORWARD_SPEED); // 0.7 is the appropriate speed
    // drive.setSteering(0.6);
    // sleep(2);

    // drive.stop();     // Fast forward

    // drive.calibrateSteering();
    
    

    // for (float i = -1.0; i < 1.0; i+=0.1)
    // {
    //     drive.setSteering(i);
    // }

    // drive.centerSteering();
    // sleep(1.0);
    // drive.stop();

    drive.keyboardControl();

    return 0;
}
