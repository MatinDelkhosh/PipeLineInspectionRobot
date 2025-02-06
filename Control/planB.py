import RPi.GPIO as GPIO
from time import sleep
from picamera2 import Picamera2
import cv2
import numpy as np
from threading import Thread

# Set pin numbering mode
GPIO.setmode(GPIO.BCM)  # Use GPIO numbers instead of physical pin numbers

# Initialize the camera
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)}))
picam2.start()


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

    leftspeed = 80 + Center/2
    rightspeed = 80 - Center/2

    leftspeed = min(leftspeed,100)
    rightspeed = min(rightspeed,100)
    leftspeed = max(leftspeed,0)
    rightspeed = max(rightspeed,0)

    pwm1.ChangeDutyCycle(leftspeed)  # Set speed to 50%
    pwm2.ChangeDutyCycle(rightspeed)


try:
    while True:
        frame = picam2.capture_array()

        center_offset, radius, output_frame = detect_strongest_circle(frame)

        Drive_Motor(center_offset[0])

finally:
    pwm1.stop()
    pwm2.stop()
    GPIO.cleanup()
