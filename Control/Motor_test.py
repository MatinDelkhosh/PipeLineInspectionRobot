import RPi.GPIO as GPIO
from time import sleep
GPIO.cleanup()

# Set pin numbering mode
GPIO.setmode(GPIO.BCM)  # Use GPIO numbers instead of physical pin numbers

# Pin setup
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

# Set up PWM for speed control
pwm1 = GPIO.PWM(MOTOR_LEFT_ENA, 1000)  # Frequency = 1kHz
pwm1.start(0)  # Start with 0% duty cycle
pwm2 = GPIO.PWM(MOTOR_RIGHT_ENA, 1000) 
pwm2.start(0)

try:
    print('moving')
    # Move motor forward
    GPIO.output(MOTOR_RIGHT_IN1, GPIO.HIGH)
    GPIO.output(MOTOR_RIGHT_IN2, GPIO.LOW)
    GPIO.output(MOTOR_LEFT_IN1, GPIO.HIGH)
    GPIO.output(MOTOR_LEFT_IN2, GPIO.LOW)
    pwm1.ChangeDutyCycle(100)  # Set speed to 50%
    pwm2.ChangeDutyCycle(100)
    sleep(10)
finally:
    pwm1.stop()
    pwm2.stop()
    GPIO.cleanup()
