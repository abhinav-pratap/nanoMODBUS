import Jetson.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)
GPIO.setup(31, GPIO.OUT)
GPIO.output(31, GPIO.LOW)

while True:
    GPIO.output(31, GPIO.LOW)
    time.sleep(1)