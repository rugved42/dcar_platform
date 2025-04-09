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

    drive.goStraight();  // Center
    sleep(2);

    // drive.stop();     // Slow forward
    // sleep(2);

    // drive.goReverse();     // Stop
    // sleep(2);

    drive.stop();     // Fast forward
    sleep(2);
}
