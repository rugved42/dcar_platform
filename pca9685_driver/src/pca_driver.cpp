// pca9685_driver.cpp
#include <iostream>
#include <fcntl.h>        // open()
#include <unistd.h>       // read(), write(), usleep(), close(), sleep()
#include <linux/i2c-dev.h> // I2C definitions
#include <sys/ioctl.h>    // ioctl()
#include <cerrno>         // errno
#include <cstring>        // strerror()
#include <cstdint>        // uint8_t, etc.
#include <cmath>          // round(), floor()

#define I2C_DEV "/dev/i2c-1"
#define PCA9685_ADDR 0x40

// PCA9685 Registers
#define MODE1 0x00
#define PRESCALE 0xFE
#define LED0_ON_L 0x06

int writeReg(int fd, uint8_t reg, uint8_t data) {
    uint8_t buffer[2] = {reg, data};
    if (write(fd, buffer, 2) != 2) {
        std::cerr << "Failed to write to I2C device: " << strerror(errno) << std::endl;
        return -1;
    }
    return 0;
}

int calculatePulse(float ms, float ms_per_tick) {
    std::cout << ms << " " << ms_per_tick << std::endl;
    return static_cast<int>((ms * 1000) / ms_per_tick);
}

int setPWM(int fd, int channel, int on, int off) {
    writeReg(fd, LED0_ON_L + 4 * channel, on & 0xFF);
    writeReg(fd, LED0_ON_L + 4 * channel + 1, on >> 8);
    writeReg(fd, LED0_ON_L + 4 * channel + 2, off & 0xFF);
    writeReg(fd, LED0_ON_L + 4 * channel + 3, off >> 8);
    return 0;
}

void setSteering(int fd, float ms, float ms_per_tick) {
    int pulse = calculatePulse(ms, ms_per_tick);
    std::cout << "[Steering] ms: " << ms << ", pulse: " << pulse << std::endl;
    setPWM(fd, 1, 0, pulse);
}

void setSpeed(int fd, float ms, float ms_per_tick) {
    int pulse = calculatePulse(ms, ms_per_tick);
    std::cout << "[Speed] ms: " << ms << ", pulse: " << pulse << std::endl;
    setPWM(fd, 0, 0, pulse);
}

int main() {
    int fd = open(I2C_DEV, O_RDWR);
    if (fd < 0) {
        std::cerr << "Failed to open I2C bus: " << strerror(errno) << std::endl;
        return 1;
    }

    if (ioctl(fd, I2C_SLAVE, PCA9685_ADDR) < 0) {
        std::cerr << "Failed to connect to PCA9685: " << strerror(errno) << std::endl;
        close(fd);
        return 1;
    }

    // Reset PCA9685
    writeReg(fd, MODE1, 0x00);
    usleep(10000);

    // Set PWM frequency to 50Hz
    float freq = 50.0;
    float prescaleval = 25000000.0; // 25MHz
    prescaleval /= 4096.0;
    prescaleval /= freq;
    prescaleval -= 1.0;
    uint8_t prescale = static_cast<uint8_t>(std::floor(prescaleval + 0.5));

    uint8_t oldmode;
    uint8_t reg = MODE1;
    if (write(fd, &reg, 1) != 1 || read(fd, &oldmode, 1) != 1) {
        std::cerr << "Failed to read mode: " << strerror(errno) << std::endl;
        close(fd);
        return 1;
    }
    uint8_t sleepmode = (oldmode & 0x7F) | 0x10; // sleep
    writeReg(fd, MODE1, sleepmode);
    writeReg(fd, PRESCALE, prescale);
    writeReg(fd, MODE1, oldmode);
    usleep(5000);
    writeReg(fd, MODE1, oldmode | 0x80);

    // Pulse conversion (ms per tick)
    int pulse_length = 1000000 / 50;
    float ms_per_tick = pulse_length / static_cast<float>(4096);

    // Perform all actions sequentially
    std::cout << "Center steering...\n";
    setSteering(fd, 1.5, ms_per_tick);
    sleep(2);

    std::cout << "Turn left...\n";
    setSteering(fd, 1.2, ms_per_tick);
    sleep(2);

    std::cout << "Turn right...\n";
    setSteering(fd, 1.8, ms_per_tick);
    sleep(2);

    std::cout << "Center steering...\n";
    setSteering(fd, 1.5, ms_per_tick);
    sleep(2);

    std::cout << "Set speed: slow...\n";
    setSpeed(fd, 1.6, ms_per_tick);
    sleep(2);

    std::cout << "Stopping...\n";
    setSpeed(fd, 0.0, ms_per_tick); // stop
    sleep(2);

    std::cout << "Set speed: fast...\n";
    setSpeed(fd, 2.0, ms_per_tick);
    sleep(2);

    std::cout << "Stopping...\n";
    setSpeed(fd, 0.0, ms_per_tick); // stop
    sleep(2);

    // Stop all channels
    for (int ch = 0; ch < 16; ++ch) {
        setPWM(fd, ch, 0, 0);
    }

    writeReg(fd, MODE1, 0x06);
    close(fd);
    return 0;
}

