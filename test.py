import Jetson.GPIO as GPIO
import time

TRIG = 12

GPIO.setmode(GPIO.BOARD)
GPIO.setup(TRIG, GPIO.OUT, initial=GPIO.LOW)

print("TRIG 테스트 시작")
print("10번의 트리거 신호를 보냅니다.")

try:
    for i in range(10):
        print("TRIG 신호 {} / 10".format(i + 1))

        GPIO.output(TRIG, GPIO.LOW)
        time.sleep(0.000002)

        GPIO.output(TRIG, GPIO.HIGH)
        time.sleep(0.000020)

        GPIO.output(TRIG, GPIO.LOW)

        time.sleep(0.1)

finally:
    GPIO.output(TRIG, GPIO.LOW)
    GPIO.cleanup()

print("테스트 종료")