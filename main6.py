import cv2
import smbus2
import time
import RPi.GPIO as GPIO
from threading import Thread
import subprocess
import numpy as np
from gpiozero import RotaryEncoder
from picamera2 import Picamera2
import tempfile
from math import cos, sin, radians
import os

# Initialize the camera
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)}))
picam2.start()

################################ Used Functions ####################################

def detect_strongest_circle(frame):
    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Enhance contrast
    gray = cv2.equalizeHist(gray)

    # Apply Gaussian Blur to reduce noise
    blurred = cv2.GaussianBlur(gray, (9, 9), 2)

    # Detect circles using HoughCircles
    circles = cv2.HoughCircles(
        blurred,
        cv2.HOUGH_GRADIENT,
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

        # Get frame dimensions
        height, width = frame.shape[:2]
        center_x, center_y = width // 2, height // 2

        # Calculate position relative to center
        relative_x = x - center_x
        relative_y = y - center_y

        return (relative_x, relative_y), r, frame

    return (0, 0), 0, frame

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

# IMU
MPU_ADDR = 0x69
bus = smbus2.SMBus(1)
#bus.write_byte_data(MPU_ADDR, 0x6B, 0)

def read_imu():
    # Read gyroscope data
    gyro_z_high = bus.read_byte_data(MPU_ADDR, 0x47)
    gyro_z_low = bus.read_byte_data(MPU_ADDR, 0x48)
    gyro_z = (gyro_z_high << 8) | gyro_z_low
    gyro_z = gyro_z if gyro_z < 32768 else gyro_z - 65536

    # Read accelerometer data
    acc_x_high = bus.read_byte_data(MPU_ADDR, 0x3B)
    acc_x_low = bus.read_byte_data(MPU_ADDR, 0x3C)
    acc_x = (acc_x_high << 8) | acc_x_low
    acc_x = acc_x if acc_x < 32768 else acc_x - 65536

    acc_y_high = bus.read_byte_data(MPU_ADDR, 0x3D)
    acc_y_low = bus.read_byte_data(MPU_ADDR, 0x3E)
    acc_y = (acc_y_high << 8) | acc_y_low
    acc_y = acc_y if acc_y < 32768 else acc_y - 65536

    return gyro_z, acc_x, acc_y

kf_gyro = KalmanFilter(0.01, 0.1)
kf_acc_x = KalmanFilter(0.01, 0.1)
kf_acc_y = KalmanFilter(0.01, 0.1)

# Encoder
encoderR = RotaryEncoder(25, 8, max_steps=0)
encoderL = RotaryEncoder(9, 11, max_steps=0)

wheel_diameter = 0.06

def read_encoder(enc):
    angle = 360 / 334. * enc.steps
    distance = angle * wheel_diameter  # Convert angle to distance
    return distance

# Calculate movement
x, y, theta = 0, 0, 0
points_3d = []

def Update_points():
    k = 10
    dt = 1
    global x, y, theta, points_3d
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

        x += w_enc * encoder_d * cos(radians(theta)) + w_imu * imu_dx
        y += w_enc * encoder_d * sin(radians(theta)) + w_imu * imu_dy
        theta += w_enc * encoder_dtheta + w_imu * (gyro_z * dt)

        points_3d.append((x, y, 0))
        time.sleep(0.1)

# UltraSonic Sensors
def measure_distance(trig, echo):
    # Ensure the trigger pin is low
    GPIO.output(trig, False)
    time.sleep(0.2)

    # Send a 10µs pulse to trigger pin
    GPIO.output(trig, True)
    time.sleep(0.00001)
    GPIO.output(trig, False)
    # Wait for the echo pin to go high
    while GPIO.input(echo) == 0:
        pulse_start = time.time()

    # Wait for the echo pin to go low
    while GPIO.input(echo) == 1:
        pulse_end = time.time()

    # Calculate the distance
    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150  # Speed of sound: 343m/s or 17150cm/s
    distance = round(distance, 2)

    if distance > 800: distance = 2
    return distance

TRIG_LEFT = 6
ECHO_LEFT = 19
TRIG_RIGHT = 5
ECHO_RIGHT = 13

#GPIO.cleanup()
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG_LEFT, GPIO.OUT)
GPIO.setup(ECHO_LEFT, GPIO.IN)
GPIO.setup(TRIG_RIGHT, GPIO.OUT)
GPIO.setup(ECHO_RIGHT, GPIO.IN)

################################# Motor Pins #######################################

MOTOR_LEFT_ENA = 24
MOTOR_LEFT_IN1 = 27
MOTOR_LEFT_IN2 = 18
MOTOR_RIGHT_ENA = 23
MOTOR_RIGHT_IN1 = 17
MOTOR_RIGHT_IN2 = 22

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
        GPIO.output(MOTOR_LEFT_IN2, GPIO.LOW)
        motor_left_pwm.ChangeDutyCycle(left_speed)
    elif left_speed < 0:
        GPIO.output(MOTOR_LEFT_IN1, GPIO.LOW)
        GPIO.output(MOTOR_LEFT_IN2, GPIO.HIGH)
        motor_left_pwm.ChangeDutyCycle(abs(left_speed))
    else:
        GPIO.output(MOTOR_LEFT_IN1, GPIO.LOW)
        GPIO.output(MOTOR_LEFT_IN2, GPIO.LOW)
        motor_left_pwm.ChangeDutyCycle(0)

    if right_speed > 0:
        GPIO.output(MOTOR_RIGHT_IN1, GPIO.HIGH)
        GPIO.output(MOTOR_RIGHT_IN2, GPIO.LOW)
        motor_right_pwm.ChangeDutyCycle(right_speed)
    elif right_speed < 0:
        GPIO.output(MOTOR_RIGHT_IN1, GPIO.LOW)
        GPIO.output(MOTOR_RIGHT_IN2, GPIO.HIGH)
        motor_right_pwm.ChangeDutyCycle(abs(right_speed))
    else:
        GPIO.output(MOTOR_RIGHT_IN1, GPIO.LOW)
        GPIO.output(MOTOR_RIGHT_IN2, GPIO.LOW)
        motor_right_pwm.ChangeDutyCycle(0)

#################################### CNN ###########################################

# Call the virtual environment script using subprocess and pass the image file
def run_inference(image_path):
    # Path to the script in your virtual environment
    script_path = 'Detection/Detection.py'

    # Run subprocess and pass the image path to the script
    result = subprocess.run(
        ['python3', script_path, image_path],
        capture_output=True,
        text=True
    )
    return result.stdout  # This will capture the output from the subprocess

def save_image(frame):
    _, temp_path = tempfile.mkstemp(suffix='.jpg')
    cv2.imwrite(temp_path, frame)
    return temp_path

################################# Main Loop ########################################

baseSpeed = 0  # Initialize to 0, PC will set it
SENSITIVITY_THRESHOLD = 4
TURN_FACTOR = 0.5
#Points_updater = Thread(target=Update_points)
try:
    #Points_updater.start()
    while True:
        distance_left = measure_distance(TRIG_LEFT, ECHO_LEFT)
        #distance_right = measure_distance(TRIG_RIGHT, ECHO_RIGHT)

        # Capture frame from the camera
        frame = picam2.capture_array()

        frame_preprocessed = cv2.resize(frame, (224, 224)) / 255

        # Optionally, you can convert the image to float32 and normalize it if required by your model
        # frame_normalized = frame_resized.astype(np.float32) / 255.0

        image_path = save_image(frame)
        print('ANN here:')
        output = run_inference(image_path)
        print('ANN:',output)

        # Clean up the temporary image file
        os.remove(image_path)

        # Process the frame for center detection (return grayscale)
        center_offset, radius, output_frame = detect_strongest_circle(frame)
        x_offset, y_offset = center_offset

        # Print the x, y, z coordinates
        if points_3d:
            x, y, z = points_3d[-1]
            print(f"X: {x}, Y: {y}, Z: {z}")
        else:
            print("No 3D points available yet.")
        # Print model output and circle center info
        print(f"Model Output: {output}")
        print(f"Circle Center X Offset: {x_offset}, Y Offset: {y_offset}")

        time.sleep(0.1)  # added to avoid busy loop

except KeyboardInterrupt:
    print("Streaming stopped")

finally:
    GPIO.cleanup()
    #Points_updater.join()
    picam2.stop()