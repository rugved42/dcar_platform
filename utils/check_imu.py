from smbus2 import SMBus
import time

# Constants
MPU9250_ADDR = 0x68  # Default I2C address for MPU9250
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43
WHO_AM_I_REG = 0x75

# Open I2C Bus 6
bus = SMBus(6)

who_am_i = bus.read_byte_data(MPU9250_ADDR, WHO_AM_I_REG)

print(f"WHO_AM_I register: 0x{who_am_i:02X}") # if this print 0x71 then the MPU9250 is connected correctly

# Sensitivity scale factors (default ±2g for Accel, ±250°/s for Gyro)
ACCEL_SCALE = 16384.0  # LSB/g
GYRO_SCALE = 131.0     # LSB/(°/s)

def read_word(bus, addr, reg):
    """Read two bytes and combine them into a signed word."""
    high = bus.read_byte_data(addr, reg)
    low = bus.read_byte_data(addr, reg + 1)
    val = (high << 8) + low
    if val >= 0x8000:
        val = -((65535 - val) + 1)
    return val

# Step 1: Wake up MPU9250
bus.write_byte_data(MPU9250_ADDR, PWR_MGMT_1, 0x00)
time.sleep(0.1)

print("Starting MPU9250 data reading... (Press Ctrl+C to stop)")

try:
    while True:
        # Read accelerometer data
        accel_x = read_word(bus, MPU9250_ADDR, ACCEL_XOUT_H) / ACCEL_SCALE
        accel_y = read_word(bus, MPU9250_ADDR, ACCEL_XOUT_H + 2) / ACCEL_SCALE
        accel_z = read_word(bus, MPU9250_ADDR, ACCEL_XOUT_H + 4) / ACCEL_SCALE

        # Read gyroscope data
        gyro_x = read_word(bus, MPU9250_ADDR, GYRO_XOUT_H) / GYRO_SCALE
        gyro_y = read_word(bus, MPU9250_ADDR, GYRO_XOUT_H + 2) / GYRO_SCALE
        gyro_z = read_word(bus, MPU9250_ADDR, GYRO_XOUT_H + 4) / GYRO_SCALE

        # Display the results
        print(f"Accel [g]: X={accel_x:.2f} Y={accel_y:.2f} Z={accel_z:.2f} | "
              f"Gyro [°/s]: X={gyro_x:.2f} Y={gyro_y:.2f} Z={gyro_z:.2f}")

        time.sleep(0.5)

except KeyboardInterrupt:
    print("\nExiting...")
    bus.close()
