#include "drive_base.hpp"
#include <unistd.h> // sleep

int main() {
    DriveBase drive;

    drive.setSteering(0.0f);  // Center
    sleep(2);

    drive.goLeft();
    sleep(2);

    drive.goRight();
    sleep(2);

    drive.setSteering(0.0f);  // Center
    sleep(2);

    drive.setSpeed(1.6f);     // Slow forward
    sleep(2);

    drive.setSpeed(0.0f);     // Stop
    sleep(2);

    drive.setSpeed(2.0f);     // Fast forward
    sleep(2);

    drive.setSpeed(0.0f);     // Stop
    return 0;
}
