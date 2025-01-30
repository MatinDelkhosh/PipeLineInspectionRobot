import smbus

# MPU-6050 Registers and Address
MPU6050_ADDR = 0x69  # I2C address of the MPU-6050
PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
ACCEL_CONFIG = 0x1C
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43

# Initialize the MPU-6050
def mpu_init():
    # Write to sample rate register
    bus.write_byte_data(MPU6050_ADDR, SMPLRT_DIV, 7)
    # Write to power management register
    bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 1)
    # Write to Configuration register
    bus.write_byte_data(MPU6050_ADDR, CONFIG, 0)
    # Write to Gyroscope configuration register
    bus.write_byte_data(MPU6050_ADDR, GYRO_CONFIG, 0)
    # Write to Accelerometer configuration register
    bus.write_byte_data(MPU6050_ADDR, ACCEL_CONFIG, 0)

# Read raw data from MPU-6050
def read_raw_data(addr):
    # Read high and low bytes and combine them
    high = bus.read_byte_data(MPU6050_ADDR, addr)
    low = bus.read_byte_data(MPU6050_ADDR, addr + 1)
    # Combine high and low bytes
    value = (high << 8) | low
    # Convert to signed value
    if value > 32768:
        value -= 65536
    return value

# I2C bus setup
bus = smbus.SMBus(1)  # Use I2C bus 1
mpu_init()

def read_imu():
    try:
            # Read Accelerometer data
            acc_x = read_raw_data(ACCEL_XOUT_H)
            acc_y = read_raw_data(ACCEL_XOUT_H + 2)
            acc_z = read_raw_data(ACCEL_XOUT_H + 4)

            # Read Gyroscope data
            gyro_x = read_raw_data(GYRO_XOUT_H)
            gyro_y = read_raw_data(GYRO_XOUT_H + 2)
            gyro_z = read_raw_data(GYRO_XOUT_H + 4)

            # Convert raw data to proper units
            ax = acc_x / 16384.0  # Accelerometer sensitivity scale factor
            ay = acc_y / 16384.0
            az = acc_z / 16384.0

            gx = gyro_x / 131.0  # Gyroscope sensitivity scale factor
            gy = gyro_y / 131.0
            gz = gyro_z / 131.0

            print(f"Accelerometer: X={ax:.2f}g, Y={ay:.2f}g, Z={az:.2f}g")
            print(f"Gyroscope: X={gx:.2f}°/s, Y={gy:.2f}°/s, Z={gz:.2f}°/s")
            print("")

            return ax, ay, az, gz

    except KeyboardInterrupt:
        print("Measurement stopped by user")
