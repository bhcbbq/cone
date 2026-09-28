import Jetson.GPIO as GPIO
import time

TRIG = 12

GPIO.setmode(GPIO.BOARD)
GPIO.setup(TRIG, GPIO.OUT, initial=GPIO.LOW)

print("TRIG 테스트 시작")

for i in range(10):
    GPIO.output(TRIG, GPIO.HIGH)
    print("HIGH")
    time.sleep(0.00001)

    GPIO.output(TRIG, GPIO.LOW)
    print("LOW")
    time.sleep(0.1)

GPIO.cleanup()

print("테스트 종료")