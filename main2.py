import cv2
import time
import RPi.GPIO as GPIO
import numpy as np
from threading import Thread
from gpiozero import RotaryEncoder
from picamera2 import Picamera2
import smbus2
from math import cos, sin

# Initialize the camera
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)}))
picam2.start()

# Initialize the IMU
MPU_ADDR = 0x68
bus = smbus2.SMBus(1)
bus.write_byte_data(MPU_ADDR, 0x6B, 0)

def read_imu():
    gyro_z = bus.read_byte_data(MPU_ADDR, 0x47) - 128  
    acc_x = bus.read_byte_data(MPU_ADDR, 0x3B) - 128  
    acc_y = bus.read_byte_data(MPU_ADDR, 0x3D) - 128  
    return gyro_z, acc_x, acc_y

# Kalman Filter
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

# Encoder setup
encoderR = RotaryEncoder(25, 8, max_steps=0)
encoderL = RotaryEncoder(9, 11, max_steps=0)
wheel_diameter = 0.06

def read_encoder(enc):
    angle = 360 / 334. * enc.steps
    distance = angle * wheel_diameter  # Convert angle to distance
    return distance

# Update points based on encoder and IMU data
x, y, theta = 0, 0, 0
points_3d = []

def Update_points():
    global x, y, theta, points_3d
    k = 10  
    dt = 1
    print("Starting to update points...")
    while True:
        encoderR_d = read_encoder(encoderR)
        encoderL_d = read_encoder(encoderL)
        encoder_d = (encoderR_d + encoderL_d) / 2
        encoder_dtheta = (encoderR_d - encoderL_d)

        gyro_z_raw, acc_x_raw, acc_y_raw = read_imu()
        gyro_z = kf_gyro.update(gyro_z_raw)
        acc_x = kf_acc_x.update(acc_x_raw)
        acc_y = kf_acc_y.update(acc_y_raw)

        w_imu = abs(gyro_z) / (abs(gyro_z) + k)
        w_enc = 1 - w_imu

        imu_dx = acc_x * dt ** 2 / 2  
        imu_dy = acc_y * dt ** 2 / 2

        x += w_enc * encoder_d * cos(theta) + w_imu * imu_dx
        y += w_enc * encoder_d * sin(theta) + w_imu * imu_dy
        theta += w_enc * encoder_dtheta + w_imu * (gyro_z * dt)

        points_3d.append((x, y, 0))
        print(f"Updated position: x={x}, y={y}, theta={theta}")
        time.sleep(dt)

# Motor pins setup
MOTOR_LEFT_ENA = 24
MOTOR_LEFT_IN1 = 27
MOTOR_LEFT_IN2 = 18
MOTOR_RIGHT_ENA = 23
MOTOR_RIGHT_IN1 = 22
MOTOR_RIGHT_IN2 = 17

GPIO.setmode(GPIO.BCM)
GPIO.setup(MOTOR_LEFT_ENA, GPIO.OUT)
GPIO.setup(MOTOR_LEFT_IN1, GPIO.OUT)
GPIO.setup(MOTOR_RIGHT_ENA, GPIO.OUT)
GPIO.setup(MOTOR_RIGHT_IN1, GPIO.OUT)
GPIO.setup(MOTOR_RIGHT_IN2, GPIO.OUT)
GPIO.setup(MOTOR_LEFT_IN2, GPIO.OUT)

motor_left_pwm = GPIO.PWM(MOTOR_LEFT_ENA, 100)
motor_right_pwm = GPIO.PWM(MOTOR_RIGHT_ENA, 100)
motor_left_pwm.start(0)
motor_right_pwm.start(0)

def control_motors(left_speed, right_speed):
    """Control motor speeds."""
    if left_speed > 0:
        GPIO.output(MOTOR_LEFT_IN1, GPIO.HIGH)
        motor_left_pwm.ChangeDutyCycle(left_speed)
    else:
        GPIO.output(MOTOR_LEFT_IN1, GPIO.LOW)
        motor_left_pwm.ChangeDutyCycle(0)

    if right_speed > 0:
        GPIO.output(MOTOR_RIGHT_IN1, GPIO.HIGH)
        motor_right_pwm.ChangeDutyCycle(right_speed)
    else:
        GPIO.output(MOTOR_RIGHT_IN1, GPIO.LOW)
        motor_right_pwm.ChangeDutyCycle(0)

# Camera Circle Detection
def detect_strongest_circle(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.equalizeHist(gray)
    blurred = cv2.GaussianBlur(gray, (9, 9), 2)
    circles = cv2.HoughCircles(
        blurred, 
        cv2.HOUGHT_GRADIENT, 
        dp=1.5, 
        minDist=30, 
        param1=80, 
        param2=30, 
        minRadius=20, 
        maxRadius=200
    )

    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        strongest_circle = circles[0]
        x, y, r = strongest_circle
        height, width = frame.shape[:2]
        center_x, center_y = width // 2, height // 2
        relative_x = x - center_x
        relative_y = y - center_y

        return (relative_x, relative_y), r, frame

    return (0, 0), 0, frame

# Ultrasonic Sensor
def measure_distance(trig, echo):
    GPIO.output(trig, False)
    time.sleep(0.2)

    GPIO.output(trig, True)
    time.sleep(0.00001)
    GPIO.output(trig, False)

    while GPIO.input(echo) == 0:
        pulse_start = time.time()

    while GPIO.input(echo) == 1:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150
    distance = round(distance, 2)

    if distance > 800: 
        distance = 2
    return distance

TRIG_LEFT = 6
ECHO_LEFT = 19
TRIG_RIGHT = 5
ECHO_RIGHT = 13

GPIO.setup(TRIG_LEFT, GPIO.OUT)
GPIO.setup(ECHO_LEFT, GPIO.IN)
GPIO.setup(TRIG_RIGHT, GPIO.OUT)
GPIO.setup(ECHO_RIGHT, GPIO.IN)

# Set default speed and start motor
baseSpeed = 10
SENSITIVITY_THRESHOLD = 4
TURN_FACTOR = 0.5
Points_updater = Thread(target=Update_points)

try:
    Points_updater.start()
    control_motors(baseSpeed, baseSpeed)
    
    while True:
        distance_left = measure_distance(TRIG_LEFT, ECHO_LEFT)
        distance_right = measure_distance(TRIG_RIGHT, ECHO_RIGHT)

        print(f"Left distance: {distance_left} cm, Right distance: {distance_right} cm")

        # Capture frame from the camera
        frame = picam2.capture_array()

        # Process the frame for center detection
        center_offset, radius, output_frame = detect_strongest_circle(frame)

        print(f"Circle detected at offset: {center_offset}, radius: {radius}")

        # Save the location points to a file
        with open("location_points.txt", "a") as f:
            for point in points_3d:
                f.write(f"{point[0]},{point[1]},{point[2]}\n")
                
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Program interrupted.")

finally:
    GPIO.cleanup()
    Points_updater.join()
    picam2.stop()
    print("GPIO cleaned up, camera stopped.")
