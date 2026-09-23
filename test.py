import Jetson.GPIO as GPIO
import time

TRIG = 12
ECHO = 16

GPIO.setmode(GPIO.BOARD)

GPIO.setup(TRIG, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(ECHO, GPIO.IN)

print("==============================")
print(" JSN-SR04T 거리 측정 테스트")
print("==============================")
print("TRIG : Jetson Pin 12")
print("ECHO : Jetson Pin 16")
print("==============================")

try:
    while True:

        # -------------------------
        # 1. TRIG 신호 발생
        # -------------------------
        GPIO.output(TRIG, GPIO.LOW)
        time.sleep(0.000002)

        GPIO.output(TRIG, GPIO.HIGH)
        time.sleep(0.000012)

        GPIO.output(TRIG, GPIO.LOW)

        # -------------------------
        # 2. ECHO HIGH 시작 대기
        # -------------------------
        rising = GPIO.wait_for_edge(
            ECHO,
            GPIO.RISING,
            timeout=100
        )

        if rising is None:
            print("ECHO 감지 안됨")
            time.sleep(0.1)
            continue

        # ECHO HIGH 시작
        start = time.monotonic()

        # -------------------------
        # 3. ECHO LOW 대기
        # -------------------------
        falling = GPIO.wait_for_edge(
            ECHO,
            GPIO.FALLING,
            timeout=100
        )

        if falling is None:
            print("ECHO 종료 안됨")
            time.sleep(0.1)
            continue

        # ECHO HIGH 종료
        end = time.monotonic()

        # -------------------------
        # 4. ECHO 시간 계산
        # -------------------------
        echo_time = end - start

        # -------------------------
        # 5. 거리 계산
        # -------------------------
        distance = (echo_time * 34300.0) / 2.0

        # -------------------------
        # 6. 거리 범위 확인
        # -------------------------
        if 20.0 <= distance <= 600.0:

            print(
                "ECHO 감지됨 → 거리: {:.2f} cm".format(
                    distance
                )
            )

        else:

            print(
                "ECHO 감지됨 → 잘못된 거리: {:.2f} cm".format(
                    distance
                )
            )

        # 측정 간격
        time.sleep(0.08)


except KeyboardInterrupt:

    print("\n측정 종료")


finally:

    GPIO.output(TRIG, GPIO.LOW)
    GPIO.cleanup()