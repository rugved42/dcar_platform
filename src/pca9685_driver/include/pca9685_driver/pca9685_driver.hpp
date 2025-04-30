#ifndef PCA9685_DRIVER_HPP
#define PCA9685_DRIVER_HPP

#include <string>

class PCA9685Driver {
public:
    explicit PCA9685Driver(const std::string& i2c_device = "/dev/i2c-1", int address = 0x40);
    ~PCA9685Driver();

    bool initialize(float frequency = 50.0f);
    void setPWM(int channel, int on, int off);
    void setPulseMs(int channel, float ms);
    float getMsPerTick() const;

private:
    int fd_;
    int address_;
    float ms_per_tick_;

    int writeReg(uint8_t reg, uint8_t data);
};

#endif
