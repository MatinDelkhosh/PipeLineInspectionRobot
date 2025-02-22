import RPi.GPIO as GPIO
import time

'''# Set GPIO mode
GPIO.setmode(GPIO.BCM)

# Define GPIO pins
TRIG = 5  # Trigger pin
ECHO = 13  # Echo pin

# Set up pins
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)'''

def measure_distance(trig,echo):
    # Ensure the trigger pin is low
    GPIO.output(trig, False)
    time.sleep(0.2)

    # Send a 10µs pulse to trigger pin
    GPIO.output(trig, True)
    time.sleep(0.00001)
    GPIO.output(trig, False)
    print('hey dist')
    # Wait for the echo pin to go high
    while GPIO.input(echo) == 0:
        print('hey1')
        pulse_start = time.time()

    # Wait for the echo pin to go low
    while GPIO.input(echo) == 1:
        print('hey2')
        pulse_end = time.time()

    # Calculate the distance
    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150  # Speed of sound: 343m/s or 17150cm/s
    distance = round(distance, 2)

    if distance > 800: distance = 2
    print(f'distance: {distance}')
    return distance

'''try:
    while True:
        dist = measure_distance(TRIG, ECHO)
        print(f"Distance: {dist} cm")
        time.sleep(1)  # Wait for 1 second before next measurement

except KeyboardInterrupt:
    print("Measurement stopped by user")
    GPIO.cleanup()'''
