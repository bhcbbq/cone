import Jetson.GPIO as GPIO
import time

TRIG = 12
ECHO = 16

GPIO.setmode(GPIO.BOARD)
GPIO.setup(TRIG, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(ECHO, GPIO.IN)

try:
    for i in range(10):
        print(f"\n--- 테스트 {i+1} ---")

        # TRIG 30us
        GPIO.output(TRIG, GPIO.HIGH)
        time.sleep(0.00003)
        GPIO.output(TRIG, GPIO.LOW)

        # 100ms 동안 ECHO 상태를 계속 확인
        start = time.time()

        while time.time() - start < 0.1:
            value = GPIO.input(ECHO)

            if value == GPIO.HIGH:
                print("ECHO HIGH 감지!")
                break
        else:
            print("ECHO 없음")

        time.sleep(0.1)

finally:
    GPIO.cleanup()