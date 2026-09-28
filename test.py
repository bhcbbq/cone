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

        # 센서 안정화
        time.sleep(0.05)

        # TRIG 100 us
        GPIO.output(TRIG, GPIO.HIGH)
        time.sleep(0.0001)
        GPIO.output(TRIG, GPIO.LOW)

        # ECHO 상승 에지 대기
        start = time.perf_counter()

        result = GPIO.wait_for_edge(
            ECHO,
            GPIO.RISING,
            timeout=100
        )

        if result is None:
            print("ECHO 상승 에지 없음")
            continue

        echo_start = time.perf_counter()

        # ECHO 하강 에지 대기
        result = GPIO.wait_for_edge(
            ECHO,
            GPIO.FALLING,
            timeout=100
        )

        if result is None:
            print("ECHO 하강 에지 없음")
            continue

        echo_end = time.perf_counter()

        duration = (echo_end - echo_start) * 1_000_000

        distance = duration * 0.0343 / 2

        print(f"ECHO HIGH 시간 = {duration:.0f} us")
        print(f"거리 = {distance:.2f} cm")

        time.sleep(0.3)

finally:
    GPIO.cleanup()