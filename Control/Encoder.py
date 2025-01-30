import RPi.GPIO as GPIO
from time import sleep

# Define encoder pins
ENCODER_A = 20  # Channel A
ENCODER_B = 21  # Channel B

# Setup GPIO
GPIO.cleanup()
GPIO.setmode(GPIO.BCM)
GPIO.setup(ENCODER_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(ENCODER_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Global variables
encoder_count = 0

# Interrupt handler for encoder
def encoder_callback(channel):
    global encoder_count
    if GPIO.input(ENCODER_B) == GPIO.input(ENCODER_A):  # Determine rotation direction
        encoder_count += 1  # Clockwise
    else:
        encoder_count -= 1  # Counterclockwise
    print(f"Encoder Count: {encoder_count}")

# Attach interrupt to channel A
GPIO.add_event_detect(ENCODER_A, GPIO.RISING, callback=encoder_callback)

try:
    print("Reading encoder...")
    while True:
        sleep(0.1)  # Small delay to reduce CPU usage
except KeyboardInterrupt:
    print("\nStopping...")
finally:
    GPIO.cleanup()  # Clean up GPIO on exit
