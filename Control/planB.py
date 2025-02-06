import RPi.GPIO as GPIO
from gpiozero import RotaryEncoder
from time import sleep
from picamera2 import Picamera2
import cv2
import numpy as np
import threading
import socket
import pickle
import struct
import smbus2
from math import cos,sin,atan2

# Set pin numbering mode
GPIO.setmode(GPIO.BCM)  # Use GPIO numbers instead of physical pin numbers

client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
host_ip = '192.168.171.250'  # Replace with your PC's IP address
port = 9999
client_socket.connect((host_ip, port))
print('Connected!')

# Initialize the camera
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)}))
picam2.start()

#function to send data
def send_data(data, data_type):
        """Helper function to send data with a type identifier."""
        data_packet = pickle.dumps((data_type, data))
        message_size = struct.pack("L", len(data_packet))
        client_socket.sendall(message_size + data_packet)

def detect_strongest_circle(frame):
    frame = frame.copy()
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

        frame = cv2.circle(frame, (x, y), r, (0, 255, 0), 4)
        frame = cv2.circle(frame, (x, y), 2, (0, 0, 255), 3)

        return (relative_x, relative_y), r, frame

    return (0,0), 0, frame

# Pin setup
MOTOR_LEFT_ENA = 24
MOTOR_LEFT_IN1 = 27
MOTOR_LEFT_IN2 = 18
MOTOR_RIGHT_ENA = 23
MOTOR_RIGHT_IN1 = 22
MOTOR_RIGHT_IN2 = 17

GPIO.setup(MOTOR_LEFT_ENA, GPIO.OUT)
GPIO.setup(MOTOR_LEFT_IN1, GPIO.OUT)
GPIO.setup(MOTOR_RIGHT_ENA, GPIO.OUT)
GPIO.setup(MOTOR_RIGHT_IN1, GPIO.OUT)
GPIO.setup(MOTOR_RIGHT_IN2, GPIO.OUT)
GPIO.setup(MOTOR_LEFT_IN2, GPIO.OUT)

# Set up PWM for speed control
pwm1 = GPIO.PWM(MOTOR_LEFT_ENA, 1000)  # Frequency = 1kHz
pwm1.start(0)  # Start with 0% duty cycle
pwm2 = GPIO.PWM(MOTOR_RIGHT_ENA, 1000) 
pwm2.start(0)

def Drive_Motor(Center):

    GPIO.output(MOTOR_RIGHT_IN1, GPIO.HIGH)
    GPIO.output(MOTOR_RIGHT_IN2, GPIO.LOW)
    GPIO.output(MOTOR_LEFT_IN1, GPIO.HIGH)
    GPIO.output(MOTOR_LEFT_IN2, GPIO.LOW)

    leftspeed = 80 + Center
    rightspeed = 80 - Center

    leftspeed = min(leftspeed,100)
    rightspeed = min(rightspeed,100)
    leftspeed = max(leftspeed,0)
    rightspeed = max(rightspeed,0)

    pwm1.ChangeDutyCycle(leftspeed)  # Set speed to 50%
    pwm2.ChangeDutyCycle(rightspeed)

def Stop_Motor():
    GPIO.output(MOTOR_RIGHT_IN1, GPIO.LOW)
    GPIO.output(MOTOR_RIGHT_IN2, GPIO.LOW)
    GPIO.output(MOTOR_LEFT_IN1, GPIO.LOW)
    GPIO.output(MOTOR_LEFT_IN2, GPIO.LOW)
    pwm1.ChangeDutyCycle(0)
    pwm2.ChangeDutyCycle(0)

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
# MPU6050 Registers
MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43

bus = smbus2.SMBus(1)
# Wake up MPU6050
bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)

def read_imu():
    gyro_z = bus.read_byte_data(MPU6050_ADDR, 0x47) - 128
    acc_x = bus.read_byte_data(MPU6050_ADDR, 0x3B) - 128
    acc_y = bus.read_byte_data(MPU6050_ADDR, 0x3D) - 128
    return gyro_z, acc_x, acc_y

kf_gyro = KalmanFilter(0.01, 0.1)
kf_acc_x = KalmanFilter(0.01, 0.1)
kf_acc_y = KalmanFilter(0.01, 0.1)
kf_x = KalmanFilter(0.01, 10)
kf_y = KalmanFilter(0.01, 10)
kf_theta = KalmanFilter(0.01, 10)

# Encoder
encoderR = RotaryEncoder(25, 8, max_steps=0)
encoderL = RotaryEncoder(9, 11, max_steps=0)

wheel_diameter = 0.065

def read_encoder(enc):
    angle = 360 / 334. * enc.steps
    distance = angle * wheel_diameter / 1000 # Convert angle to distance
    return distance

# Calculate movement
points_3d = []

def Update_points(k=0.8, dt=1):
    global points_3d
    running = True  # Control flag for stopping the loop
    x, y, theta = 0, 0, 0
    
    while running:
        try:
            # Read sensors
            encoderR_d = read_encoder(encoderR)
            encoderL_d = read_encoder(encoderL)
            encoder_d = (encoderR_d + encoderL_d) / 2
            encoder_dtheta = atan2(encoderR_d - encoderL_d , encoder_d) / 3.141 if encoder_d != 0 else 0

            gyro_z_raw, acc_x_raw, acc_y_raw = read_imu()
            gyro_z = kf_gyro.update(gyro_z_raw / 131.0 )
            acc_x = kf_acc_x.update(acc_x_raw / 16384.0 * 9.78)
            acc_y = kf_acc_y.update(acc_y_raw / 16384.0 * 9.78)

            # Compute weight factors
            w_imu = abs(gyro_z) / (abs(gyro_z) + k) if abs(gyro_z) + k != 0 else 0.5
            w_enc = 1 - w_imu

            # Compute motion updates
            imu_dx = acc_x * (dt ** 2) / 2
            imu_dy = acc_y * (dt ** 2) / 2

            
            x += w_enc * encoder_d * cos(theta) + w_imu * imu_dx
            y += w_enc * encoder_d * sin(theta) + w_imu * imu_dy
            theta += w_enc * encoder_dtheta + w_imu * (gyro_z * dt)

            x = kf_x.update(x)
            y = kf_y.update(y)
            theta = kf_theta.update(theta)
            points_3d.append((x, y, 0))
            
            # Send the updated points_3d data to the server
            send_data((x, y, 0), "points_3d")

            sleep(dt)  # 100ms delay for real-time update
            print(f'\rpoints calcd {acc_x:.2f}, {x*100:.2f}, {y*100:.2f}, {theta:.1f}, {w_enc}',end='')
        
        except Exception as e:
            print(f"Error in Update_points: {e}")
            running = False  # Stop the loop if an error occurs

# Global variable for motor control
motor_running = False

def listen_for_server_commands():
    global motor_running
    counter = 1
    while True:
        if counter<1000:counter+=1
        try:
            command = client_socket.recv(1024).decode('utf-8')
            if command == "STOP":
                motor_running = False
                Stop_Motor()
            elif command == "START":
                motor_running = True
        except Exception as e:
            print(f"Error receiving command: {e}")
            break

# Start a separate thread for listening to the server
command_listener = threading.Thread(target=listen_for_server_commands, daemon=True)
command_listener.start()
Points_updater = threading.Thread(target=Update_points)
Points_updater.start()

try:
    while True:
        frame = picam2.capture_array()

        center_offset, radius, output_frame = detect_strongest_circle(frame)

        send_data(output_frame, "image")

        if motor_running: Drive_Motor(center_offset[0])

finally:
    Stop_Motor()
    GPIO.cleanup()
    command_listener.join()
    Points_updater.join()
