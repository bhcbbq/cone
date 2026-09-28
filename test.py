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

        # TRIG 100us
        GPIO.output(TRIG, GPIO.HIGH)
        time.sleep(0.0001)
        GPIO.output(TRIG, GPIO.LOW)

        # ECHO가 HIGH가 될 때까지 기다림
        timeout_start = time.perf_counter()

        while GPIO.input(ECHO) == GPIO.LOW:
            if time.perf_counter() - timeout_start > 0.02:
                print("ECHO 없음")
                break

        else:
            echo_start = time.perf_counter()

            # ECHO가 LOW가 될 때까지 기다림
            while GPIO.input(ECHO) == GPIO.HIGH:
                if time.perf_counter() - echo_start > 0.02:
                    print("ECHO 너무 김")
                    break

            else:
                echo_end = time.perf_counter()

                duration = (echo_end - echo_start) * 1_000_000
                distance = duration * 0.0343 / 2

                print(f"ECHO 감지: {duration:.0f} us")
                print(f"거리: {distance:.2f} cm")

        time.sleep(0.5)

finally:
    GPIO.cleanup()