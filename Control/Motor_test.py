import RPi.GPIO as GPIO
from time import sleep

# Set pin numbering mode
GPIO.setmode(GPIO.BCM)  # Use GPIO numbers instead of physical pin numbers

# Pin setup
IN1 = 17  # GPIO pin connected to IN1
IN2 = 27  # GPIO pin connected to IN2
ENA = 22  # GPIO pin connected to ENA (PWM)

GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)

# Set up PWM for speed control
pwm = GPIO.PWM(ENA, 1000)  # Frequency = 1kHz
pwm.start(0)  # Start with 0% duty cycle

try:
    # Move motor forward
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    pwm.ChangeDutyCycle(50)  # Set speed to 50%
    sleep(5)

    # Stop motor
    pwm.ChangeDutyCycle(0)
    sleep(2)

    # Move motor backward
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    pwm.ChangeDutyCycle(75)  # Set speed to 75%
    sleep(5)

finally:
    pwm.stop()
    GPIO.cleanup()
