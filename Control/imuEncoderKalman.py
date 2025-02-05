import numpy as np
import smbus2
import time
from gpiozero import RotaryEncoder

k = 10  
dt = 0.1  

x, y, theta = 0, 0, 0  

# (MPU6050)
MPU_ADDR = 0x68
bus = smbus2.SMBus(1)
bus.write_byte_data(MPU_ADDR, 0x6B, 0)   

def read_imu():
    gyro_z = bus.read_byte_data(MPU_ADDR, 0x47) - 128  
    acc_x = bus.read_byte_data(MPU_ADDR, 0x3B) - 128  
    acc_y = bus.read_byte_data(MPU_ADDR, 0x3D) - 128  
    return gyro_z, acc_x, acc_y

# Instantiate the RotaryEncoder
encoder = RotaryEncoder(20, 21, max_steps=0)

def read_encoder():
    angle = 360 / 334. * encoder.steps  # Assuming ppr = 300.8
    distance = angle * 0.01  # Convert angle to distance
    return distance, 0.0 

class KalmanFilter:
    def __init__(self, process_variance, measurement_variance):
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        self.estimate = 0
        self.error_estimate = 1

    def update(self, measurement):
        kalman_gain = self.error_estimate / (self.error_estimate + self.measurement_variance)
        self.estimate = self.estimate + kalman_gain * (measurement - self.estimate)
        self.error_estimate = (1 - kalman_gain) * self.error_estimate + self.process_variance
        return self.estimate

kf_gyro = KalmanFilter(0.01, 0.1)
kf_acc_x = KalmanFilter(0.01, 0.1)
kf_acc_y = KalmanFilter(0.01, 0.1)

while True:
    encoder_dx, encoder_dy = read_encoder()
    encoder_dtheta = 0.01  

    gyro_z_raw, acc_x_raw, acc_y_raw = read_imu()
    gyro_z = kf_gyro.update(gyro_z_raw)
    acc_x = kf_acc_x.update(acc_x_raw)
    acc_y = kf_acc_y.update(acc_y_raw)

    w_imu = abs(gyro_z) / (abs(gyro_z) + k)
    w_enc = 1 - w_imu

    imu_dx = acc_x * dt ** 2 / 2  
    imu_dy = acc_y * dt ** 2 / 2

    x += w_enc * encoder_dx + w_imu * imu_dx
    y += w_enc * encoder_dy + w_imu * imu_dy
    theta += w_enc * encoder_dtheta + w_imu * (gyro_z * dt)

    print(f"X: {x:.3f}, Y: {y:.3f}, Theta: {theta:.3f}")
    time.sleep(dt)
