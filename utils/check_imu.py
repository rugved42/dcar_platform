import smbus2
import time
import json
import numpy as np
import argparse

# === CONFIGURATION ===
I2C_BUS_NUMBER = 6  # Use your correct bus, e.g., /dev/i2c-6
MPU_ADDRESS = 0x68
MAG_ADDRESS = 0x0C

NUM_SAMPLES_STATIC = 5000
NUM_SAMPLES_MAG = 5000

# === REGISTERS ===
PWR_MGMT_1 = 0x6B
INT_PIN_CFG = 0x02
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43
MAG_ST1 = 0x02
MAG_XOUT_L = 0x03

# === FUNCTIONS ===

def read_word(bus, addr, reg):
    high = bus.read_byte_data(addr, reg)
    low = bus.read_byte_data(addr, reg + 1)
    value = (high << 8) | low
    if value > 32767:
        value -= 65536
    return value

def setup_mpu(bus):
    bus.write_byte_data(MPU_ADDRESS, PWR_MGMT_1, 0x00)  # Wake up MPU
    time.sleep(0.1)
    bus.write_byte_data(MPU_ADDRESS, INT_PIN_CFG, 0x02)  # Enable bypass to access magnetometer
    time.sleep(0.1)

    # Reset magnetometer
    bus.write_byte_data(MAG_ADDRESS, 0x0B, 0x01)  # CNTL2: Soft reset
    time.sleep(0.1)
    bus.write_byte_data(MAG_ADDRESS, 0x0A, 0x00)
    time.sleep(0.1)
    bus.write_byte_data(MAG_ADDRESS, 0x0A, 0x16)
    time.sleep(0.1)
    mode = bus.read_byte_data(0x0C, 0x0A)
    print("Magnetometer status register (0x02):", hex(mode))
    time.sleep(0.05)
    # mag_data = [
    # bus.read_byte_data(0x0C, 0x03),
    # bus.read_byte_data(0x0C, 0x04),
    # bus.read_byte_data(0x0C, 0x05),
    # bus.read_byte_data(0x0C, 0x06),
    # bus.read_byte_data(0x0C, 0x07),
    # bus.read_byte_data(0x0C, 0x08)
    # ]
    # print("First raw magnetometer data:", mag_data)

    # Set magnetometer to continuous measurement mode 2 (100Hz, 16-bit output)
    #bus.write_byte_data(MAG_ADDRESS, 0x0A, 0x16)  # CNTL1: 0x16 = 16-bit output, 100Hz
    #time.sleep(0.05)

def read_imu(bus):
    accel = [
        read_word(bus, MPU_ADDRESS, ACCEL_XOUT_H),
        read_word(bus, MPU_ADDRESS, ACCEL_XOUT_H + 2),
        read_word(bus, MPU_ADDRESS, ACCEL_XOUT_H + 4)
    ]
    gyro = [
        read_word(bus, MPU_ADDRESS, GYRO_XOUT_H),
        read_word(bus, MPU_ADDRESS, GYRO_XOUT_H + 2),
        read_word(bus, MPU_ADDRESS, GYRO_XOUT_H + 4)
    ]
    mag = [0, 0, 0]
    st1 = bus.read_byte_data(MAG_ADDRESS, MAG_ST1)
    if st1 & 0x01:
        data = bus.read_i2c_block_data(MAG_ADDRESS, MAG_XOUT_L, 7)
        x = (data[1] << 8) | data[0]
        y = (data[3] << 8) | data[2]
        z = (data[5] << 8) | data[4]

        if x > 32767:
            x -= 65536
        if y > 32767:
            y -= 65536
        if z > 32767:
            z -= 65536

        mag = [x, y, z]

    return accel, gyro, mag

# === MAIN SCRIPT ===

def main():
    parser = argparse.ArgumentParser(description="IMU 9-DOF calibration script")
    parser.add_argument("--save_raw", action="store_true", help="Save raw accel/gyro/mag samples to imu_raw_data.npz")
    args = parser.parse_args()

    bus = smbus2.SMBus(I2C_BUS_NUMBER)
    setup_mpu(bus)

    print("\n[STEP 1/2] Static Gyro + Accel Bias Calibration")
    print(">> Keep the robot absolutely still...")
    time.sleep(2)

    accel_data = []
    gyro_data = []
    mag_data_static = []
    timestamps_static = []

    for i in range(NUM_SAMPLES_STATIC):
        accel, gyro, mag = read_imu(bus)
        timestamp = time.time()  # seconds since epoch (float)
        accel_data.append(accel)
        gyro_data.append(gyro)
        mag_data_static.append(mag)
        timestamps_static.append(timestamp)
        time.sleep(0.01)

    accel_data = np.array(accel_data)
    gyro_data = np.array(gyro_data)
    mag_data_static = np.array(mag_data_static)
    timestamps_static = np.array(timestamps_static)

    accel_bias = np.mean(accel_data, axis=0)
    gyro_bias = np.mean(gyro_data, axis=0)

    # Adjust accel bias to expect gravity on Z-axis
    gravity_raw = 16384.0  # ±2g range
    accel_bias[2] -= gravity_raw

    print("\nGyro bias (raw units):", gyro_bias)
    print("Accel bias (raw units):", accel_bias)

    print("\n[STEP 2/2] Magnetometer Calibration (Move robot slowly in all directions!)")
    print(">> Start moving the robot or IMU gently now...")
    time.sleep(2)

    mag_data_moving = []
    timestamps_moving = []

    for i in range(NUM_SAMPLES_MAG):
        _, _, mag = read_imu(bus)
        timestamp = time.time()
        mag_data_moving.append(mag)
        timestamps_moving.append(timestamp)
        print("mag", mag)
        time.sleep(0.01)

    mag_data_moving = np.array(mag_data_moving)
    timestamps_moving = np.array(timestamps_moving)

    # Simple hard-iron calibration
    mag_bias = np.mean(mag_data_moving, axis=0)

    print("\nMagnetometer bias (hard-iron correction):", mag_bias)

    # Save calibration result
    calib = {
        "gyro_bias": gyro_bias.tolist(),
        "accel_bias": accel_bias.tolist(),
        "mag_bias": mag_bias.tolist()
    }

    with open("imu_calibration.json", "w") as f:
        json.dump(calib, f, indent=4)

    print("✅ Saved imu_calibration.json")

    # Save full raw data if asked
    if args.save_raw:
        np.savez_compressed(
            "imu_raw_data.npz",
            accel_static=accel_data,
            gyro_static=gyro_data,
            mag_static=mag_data_static,
            timestamps_static=timestamps_static,
            mag_moving=mag_data_moving,
            timestamps_moving=timestamps_moving
        )
        print("✅ Saved raw IMU data to imu_raw_data.npz")

    bus.close()

if __name__ == "__main__":
    main()
