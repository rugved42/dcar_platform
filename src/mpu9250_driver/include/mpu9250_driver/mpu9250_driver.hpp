#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <stdexcept>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <cmath>

namespace mpu9250_driver
{

struct ImuData
{
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float mag_x;
    float mag_y;
    float mag_z;
};

class Calibrator
{
public:
    void addSample(const ImuData& data)
    {
        samples_.push_back(data);
    }

    void computeBias()
    {
        if (samples_.empty())
            return;

        float sum_gx = 0, sum_gy = 0, sum_gz = 0;
        for (const auto& s : samples_)
        {
            sum_gx += s.gyro_x;
            sum_gy += s.gyro_y;
            sum_gz += s.gyro_z;
        }

        gyro_bias_x_ = sum_gx / samples_.size();
        gyro_bias_y_ = sum_gy / samples_.size();
        gyro_bias_z_ = sum_gz / samples_.size();

        // (Optional) could also estimate accel bias if needed
    }

    void apply(ImuData& data) const
    {
        data.gyro_x -= gyro_bias_x_;
        data.gyro_y -= gyro_bias_y_;
        data.gyro_z -= gyro_bias_z_;
    }

private:
    std::vector<ImuData> samples_;
    float gyro_bias_x_ = 0.0f;
    float gyro_bias_y_ = 0.0f;
    float gyro_bias_z_ = 0.0f;
};

class MPU9250Driver
{
public:
    MPU9250Driver(const std::string& i2c_device = "/dev/i2c-6", uint8_t mpu_addr = 0x68);
    ~MPU9250Driver();

    void initialize();
    ImuData readImu();

    void calibrate();  // <-- new public function

private:
    int i2c_fd_;
    uint8_t mpu_addr_;
    uint8_t mag_addr_ = 0x0C;

    static constexpr float ACCEL_SCALE = 16384.0f;
    static constexpr float GYRO_SCALE = 131.0f;
    static constexpr float MAG_SCALE = 0.15e-6f;

    void writeRegister(uint8_t addr, uint8_t reg, uint8_t data);
    uint8_t readRegister(uint8_t addr, uint8_t reg);
    int16_t readWord(uint8_t addr, uint8_t reg);

    Calibrator calibrator_;
};

} // namespace mpu9250_driver
