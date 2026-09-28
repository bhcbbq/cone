import Jetson.GPIO as GPIO
import time

TRIG = 12
ECHO = 16

GPIO.setmode(GPIO.BOARD)

GPIO.setup(TRIG, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(ECHO, GPIO.IN)

try:
    print("AJ-SR04M 초음파 테스트 시작")
    print("5초 후 측정을 시작합니다.")
    time.sleep(5)

    for i in range(10):

        print(f"\n--- {i + 1}번째 측정 ---")

        # TRIG 10us HIGH
        GPIO.output(TRIG, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(TRIG, GPIO.LOW)

        # ECHO가 HIGH가 될 때까지 기다림
        start_wait = time.perf_counter()

        while GPIO.input(ECHO) == GPIO.LOW:
            if time.perf_counter() - start_wait > 0.1:
                print("ECHO 없음")
                break
        else:

            # ECHO HIGH 시작
            start = time.perf_counter()

            while GPIO.input(ECHO) == GPIO.HIGH:
                if time.perf_counter() - start > 0.1:
                    print("ECHO 너무 김")
                    break

            else:

                end = time.perf_counter()

                # ECHO HIGH 시간
                duration = (end - start) * 1_000_000

                # 거리 계산
                distance = duration * 0.0343 / 2

                print(f"ECHO 시간 : {duration:.0f} us")
                print(f"거리      : {distance:.2f} cm")

        time.sleep(1)

finally:
    GPIO.cleanup()