import Jetson.GPIO as GPIO
import time

TRIG = 12
ECHO = 16

GPIO.setmode(GPIO.BOARD)

GPIO.setup(TRIG, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(ECHO, GPIO.IN)

try:
    for i in range(10):

        print(f"\n--- {i+1}번째 측정 ---")

        # ECHO 현재 상태 확인
        print("측정 전 ECHO =", GPIO.input(ECHO))

        # TRIG 100us
        GPIO.output(TRIG, GPIO.HIGH)
        time.sleep(0.0001)
        GPIO.output(TRIG, GPIO.LOW)

        # ECHO HIGH 대기
        start_wait = time.perf_counter()

        while GPIO.input(ECHO) == GPIO.LOW:

            if time.perf_counter() - start_wait >= 0.02:
                print("ECHO 없음")
                break

        else:
            print("ECHO HIGH 감지")

            # ECHO HIGH 시간 측정
            start_echo = time.perf_counter()

            while GPIO.input(ECHO) == GPIO.HIGH:

                if time.perf_counter() - start_echo >= 0.02:
                    print("ECHO 너무 김")
                    break

            else:
                end_echo = time.perf_counter()

                duration = (end_echo - start_echo) * 1_000_000

                distance = duration * 0.0343 / 2

                print(f"ECHO 시간 = {duration:.0f} us")
                print(f"거리 = {distance:.2f} cm")

        time.sleep(0.5)

finally:
    GPIO.cleanup()