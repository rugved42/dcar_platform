#include "pca9685_driver/pca9685_driver.hpp"
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <cerrno>
#include <cstring>
#include <cmath>

#define MODE1 0x00
#define PRESCALE 0xFE
#define LED0_ON_L 0x06

PCA9685Driver::PCA9685Driver(const std::string& i2c_device, int address) : address_(address), ms_per_tick_(0.0f) {
    fd_ = open(i2c_device.c_str(), O_RDWR);
    if (fd_ < 0 || ioctl(fd_, I2C_SLAVE, address_) < 0) {
        std::cerr << "Failed to open I2C: " << strerror(errno) << std::endl;
        fd_ = -1;
    }
}

PCA9685Driver::~PCA9685Driver() {
    if (fd_ >= 0) close(fd_);
}

bool PCA9685Driver::initialize(float frequency) {
    if (fd_ < 0) return false;

    writeReg(MODE1, 0x00);
    usleep(10000);

    float freq = 50.0;
    float prescaleval = 25000000.0; // 25MHz from the datasheet
    prescaleval /= 4096.0;
    prescaleval /= freq;
    prescaleval -= 1.0;
    uint8_t prescale = static_cast<uint8_t>(std::floor(prescaleval + 0.5f));

    uint8_t oldmode;
    uint8_t reg = MODE1;
    if (write(fd_, &reg, 1) != 1 || read(fd_, &oldmode, 1) != 1) return false;

    writeReg(MODE1, (oldmode & 0x7F) | 0x10); // sleep
    writeReg(PRESCALE, prescale);
    writeReg(MODE1, oldmode);
    usleep(5000);
    writeReg(MODE1, oldmode | 0x80); // restart, auto-increment

    int pulse_length = 1000000 / frequency;
    ms_per_tick_ = pulse_length / 4096.0f;
    std::cout << "Driver initialized. Sending neutral for ESC calibration." << std::endl;
    setPulseMs(0, 1.5f);
    usleep(1000000);  // Hold neutral 1s before any commands
    return true;
}

int PCA9685Driver::writeReg(uint8_t reg, uint8_t data) {
    uint8_t buffer[2] = {reg, data};
    return (write(fd_, buffer, 2) == 2) ? 0 : -1;
}

void PCA9685Driver::setPWM(int channel, int on, int off) {
    writeReg(LED0_ON_L + 4 * channel, on & 0xFF);
    writeReg(LED0_ON_L + 4 * channel + 1, on >> 8);
    writeReg(LED0_ON_L + 4 * channel + 2, off & 0xFF);
    writeReg(LED0_ON_L + 4 * channel + 3, off >> 8);
}

void PCA9685Driver::setPulseMs(int channel, float ms) {
    int pulse = static_cast<int>((ms * 1000) / ms_per_tick_);
    std::cout << "pulse " <<  pulse << std::endl;
    setPWM(channel, 0, pulse);
}

float PCA9685Driver::getMsPerTick() const {
    return ms_per_tick_;
}
