import Jetson.GPIO as GPIO
import time

TRIG = 12

GPIO.setmode(GPIO.BOARD)
GPIO.setup(TRIG, GPIO.OUT, initial=GPIO.LOW)

print("TRIG LOW 상태")
time.sleep(2)

print("TRIG HIGH 상태 - 5초 유지")
GPIO.output(TRIG, GPIO.HIGH)
time.sleep(5)

print("TRIG LOW 상태")
GPIO.output(TRIG, GPIO.LOW)
time.sleep(2)

GPIO.cleanup()
print("테스트 종료")