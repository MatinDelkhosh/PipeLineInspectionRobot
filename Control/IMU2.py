import smbus
import time

# MPU6050 Registers
MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43

# Initialize I2C bus
bus = smbus.SMBus(1)

# Wake up MPU6050
bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)

def read_raw_data(addr):
    """Read raw data from the sensor."""
    high = bus.read_byte_data(MPU6050_ADDR, addr)
    low = bus.read_byte_data(MPU6050_ADDR, addr+1)
    value = (high << 8) | low
    
    if value > 32768:
        value -= 65536
    return value

while True:
    # Read Accelerometer raw values
    accel_x = read_raw_data(ACCEL_XOUT_H)
    accel_y = read_raw_data(ACCEL_XOUT_H + 2)
    accel_z = read_raw_data(ACCEL_XOUT_H + 4)

    # Read Gyroscope raw values
    gyro_x = read_raw_data(GYRO_XOUT_H)
    gyro_y = read_raw_data(GYRO_XOUT_H + 2)
    gyro_z = read_raw_data(GYRO_XOUT_H + 4)

    # Convert raw values to g-forces and degrees/sec
    Ax = accel_x / 16384.0
    Ay = accel_y / 16384.0
    Az = accel_z / 16384.0

    Gx = gyro_x / 131.0
    Gy = gyro_y / 131.0
    Gz = gyro_z / 131.0

    print(f"Ax={Ax:.2f}g Ay={Ay:.2f}g Az={Az:.2f}g | Gx={Gx:.2f}°/s Gy={Gy:.2f}°/s Gz={Gz:.2f}°/s")
    
    time.sleep(1)
