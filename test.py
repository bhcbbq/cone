import Jetson.GPIO as GPIO
import time

TRIG = 12

GPIO.setmode(GPIO.BOARD)
GPIO.setup(TRIG, GPIO.OUT, initial=GPIO.LOW)

try:
    for i in range(10):
        GPIO.output(TRIG, GPIO.HIGH)
        print("TRIG HIGH")
        time.sleep(0.00003)

        GPIO.output(TRIG, GPIO.LOW)
        print("TRIG LOW")

        time.sleep(0.1)

finally:
    GPIO.cleanup()