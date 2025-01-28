import smbus
import time

MPU6050_ADDR = 0x68  # Default I2C address
bus = smbus.SMBus(1)

# Check if device is connected by reading WHO_AM_I register
try:
    who_am_i = bus.read_byte_data(MPU6050_ADDR, 0x75)  # WHO_AM_I register
    print(f"MPU-6050 WHO_AM_I: {who_am_i}")
except Exception as e:
    print(f"Error reading from MPU-6050: {e}")
