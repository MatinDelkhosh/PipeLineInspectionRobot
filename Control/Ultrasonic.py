import RPi.GPIO as GPIO
import time

# Set GPIO mode
GPIO.setmode(GPIO.BCM)

# Define GPIO pins
TRIG = 23  # Trigger pin
ECHO = 24  # Echo pin

# Set up pins
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

def measure_distance():
    # Ensure the trigger pin is low
    GPIO.output(TRIG, False)
    time.sleep(0.2)

    # Send a 10µs pulse to trigger pin
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    # Wait for the echo pin to go high
    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()

    # Wait for the echo pin to go low
    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()

    # Calculate the distance
    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150  # Speed of sound: 343m/s or 17150cm/s
    distance = round(distance, 2)

    return distance

try:
    while True:
        dist = measure_distance()
        print(f"Distance: {dist} cm")
        time.sleep(1)  # Wait for 1 second before next measurement

except KeyboardInterrupt:
    print("Measurement stopped by user")
    GPIO.cleanup()
