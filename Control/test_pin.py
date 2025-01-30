import RPi.GPIO as GPIO
from time import sleep

GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.OUT)
GPIO.setup(27, GPIO.OUT)
GPIO.setup(23, GPIO.OUT) #ENA

GPIO.output(17, GPIO.HIGH)
GPIO.output(27, GPIO.LOW)
pwm = GPIO.PWM(23, 1000)  # Frequency = 1kHz

pwm.start(100)  # Start with 0% duty cycle
pwm.ChangeDutyCycle(80)
sleep(10)


GPIO.cleanup()