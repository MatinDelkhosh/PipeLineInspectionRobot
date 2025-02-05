import RPi.GPIO as GPIO
from time import sleep
from picamera2 import Picamera2
import cv2
import numpy as np

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

try:
    while True:
        print('moving')
        # Move motor forward
        GPIO.output(MOTOR_RIGHT_IN1, GPIO.HIGH)
        GPIO.output(MOTOR_RIGHT_IN2, GPIO.LOW)
        GPIO.output(MOTOR_LEFT_IN1, GPIO.HIGH)
        GPIO.output(MOTOR_LEFT_IN2, GPIO.LOW)
        
        # Capture frame from the camera
        frame = picam2.capture_array()

        center_offset, radius, output_frame = detect_strongest_circle(frame)

        pwm1.ChangeDutyCycle(100 + center_offset[0]/100)  # Set speed to 50%
        pwm2.ChangeDutyCycle(100 - center_offset[0]/100)
        sleep(0.1)
finally:
    pwm1.stop()
    pwm2.stop()
    GPIO.cleanup()
