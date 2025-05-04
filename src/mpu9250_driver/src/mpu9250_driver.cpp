#include "mpu9250_driver/mpu9250_driver.hpp"
#include <iostream>
#include <thread>

namespace mpu9250_driver
{

MPU9250Driver::MPU9250Driver(const std::string& i2c_device, uint8_t mpu_addr)
: mpu_addr_(mpu_addr)
{
    i2c_fd_ = open(i2c_device.c_str(), O_RDWR);
    if (i2c_fd_ < 0)
        throw std::runtime_error("Failed to open I2C device");
}

MPU9250Driver::~MPU9250Driver()
{
    if (i2c_fd_ >= 0)
        close(i2c_fd_);
}

void MPU9250Driver::writeRegister(uint8_t addr, uint8_t reg, uint8_t data)
{
    if (ioctl(i2c_fd_, I2C_SLAVE, addr) < 0)
        throw std::runtime_error("Failed to select I2C device");

    uint8_t buf[2] = {reg, data};
    if (write(i2c_fd_, buf, 2) != 2)
        throw std::runtime_error("Failed to write to I2C device");
}

uint8_t MPU9250Driver::readRegister(uint8_t addr, uint8_t reg)
{
    if (ioctl(i2c_fd_, I2C_SLAVE, addr) < 0)
        throw std::runtime_error("Failed to select I2C device");

    if (write(i2c_fd_, &reg, 1) != 1)
        throw std::runtime_error("Failed to write register address");

    uint8_t data;
    if (read(i2c_fd_, &data, 1) != 1)
        throw std::runtime_error("Failed to read from I2C device");

    return data;
}

int16_t MPU9250Driver::readWord(uint8_t addr, uint8_t reg)
{
    uint8_t high = readRegister(addr, reg);
    uint8_t low = readRegister(addr, reg + 1);
    return (int16_t)((high << 8) | low);
}

void MPU9250Driver::initialize()
{
    writeRegister(mpu_addr_, 0x6B, 0x00);
    usleep(10000);
    writeRegister(mpu_addr_, 0x37, 0x02);
    usleep(10000);
}

ImuData MPU9250Driver::readImu()
{
    ImuData data;

    data.accel_x = readWord(mpu_addr_, 0x3B) * (9.80665f / ACCEL_SCALE);
    data.accel_y = readWord(mpu_addr_, 0x3D) * (9.80665f / ACCEL_SCALE);
    data.accel_z = readWord(mpu_addr_, 0x3F) * (9.80665f / ACCEL_SCALE);

    data.gyro_x = (M_PI / 180.0f) * readWord(mpu_addr_, 0x43) / GYRO_SCALE;
    data.gyro_y = (M_PI / 180.0f) * readWord(mpu_addr_, 0x45) / GYRO_SCALE;
    data.gyro_z = (M_PI / 180.0f) * readWord(mpu_addr_, 0x47) / GYRO_SCALE;

    uint8_t st1 = readRegister(mag_addr_, 0x02);
    if (st1 & 0x01)
    {
        data.mag_x = readWord(mag_addr_, 0x03) * MAG_SCALE;
        data.mag_y = readWord(mag_addr_, 0x05) * MAG_SCALE;
        data.mag_z = readWord(mag_addr_, 0x07) * MAG_SCALE;
    }
    else
    {
        data.mag_x = data.mag_y = data.mag_z = 0.0f;
    }

    // Apply calibration
    calibrator_.apply(data);

    return data;
}

void MPU9250Driver::calibrate()
{
    std::cout << "[Calibration] Collecting gyro bias data, keep robot still..." << std::endl;
    const int num_samples = 500;

    for (int i = 0; i < num_samples; ++i)
    {
        ImuData data;

        data.gyro_x = (M_PI / 180.0f) * readWord(mpu_addr_, 0x43) / GYRO_SCALE;
        data.gyro_y = (M_PI / 180.0f) * readWord(mpu_addr_, 0x45) / GYRO_SCALE;
        data.gyro_z = (M_PI / 180.0f) * readWord(mpu_addr_, 0x47) / GYRO_SCALE;

        calibrator_.addSample(data);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    calibrator_.computeBias();
    std::cout << "[Calibration] Gyro bias computed." << std::endl;
}

} // namespace mpu9250_driver
