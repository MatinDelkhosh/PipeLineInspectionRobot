import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.OUT)
GPIO.setup(27, GPIO.OUT)
GPIO.setup(23, GPIO.OUT) #ENA

GPIO.output(17, GPIO.HIGH)
GPIO.output(27, GPIO.LOW)
GPIO.output(23, GPIO.HIGH)
from time import sleep
sleep(10)


GPIO.cleanup()