import Jetson.GPIO as GPIO
import time

# =========================
# GPIO 설정
# =========================
TRIG = 12
ECHO = 16

GPIO.setmode(GPIO.BOARD)

GPIO.setup(TRIG, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(ECHO, GPIO.IN)

# 실제 테스트에서 20cm 이하에서 불안정했으므로
# 20cm를 실질적인 최소 유효 거리로 설정
MIN_DISTANCE = 20.0

# JSN-SR04T 계열의 일반적인 최대 범위를 고려
MAX_DISTANCE = 600.0

print("================================")
print(" JSN-SR04T 거리 측정 시작")
print(" TRIG : Jetson Pin 12")
print(" ECHO : Jetson Pin 16")
print("================================")

try:

    while True:

        # ---------------------------------
        # 이전 ECHO 신호가 남아있다면 기다림
        # ---------------------------------
        wait_start = time.monotonic()

        while GPIO.input(ECHO) == GPIO.HIGH:

            if time.monotonic() - wait_start > 0.1:
                break

        # ---------------------------------
        # TRIG 신호
        # ---------------------------------
        GPIO.output(TRIG, GPIO.LOW)
        time.sleep(0.000002)

        GPIO.output(TRIG, GPIO.HIGH)
        time.sleep(0.000012)   # 12us
        GPIO.output(TRIG, GPIO.LOW)

        # ---------------------------------
        # ECHO HIGH 시작 대기
        # ---------------------------------
        start_wait = time.monotonic()

        while GPIO.input(ECHO) == GPIO.LOW:

            if time.monotonic() - start_wait > 0.06:
                break

        else:

            # ---------------------------------
            # ECHO HIGH 시작
            # ---------------------------------
            start = time.monotonic()

            # ---------------------------------
            # ECHO LOW 대기
            # ---------------------------------
            while GPIO.input(ECHO) == GPIO.HIGH:

                if time.monotonic() - start > 0.06:
                    break

            else:

                end = time.monotonic()

                # ---------------------------------
                # 거리 계산
                # ---------------------------------
                echo_time = end - start

                distance = echo_time * 34300.0 / 2.0

                # ---------------------------------
                # 비정상 값 제거
                # ---------------------------------
                if MIN_DISTANCE <= distance <= MAX_DISTANCE:

                    print(
                        "거리: {:.2f} cm".format(distance)
                    )

                else:

                    print(
                        "측정 범위 밖: {:.2f} cm".format(distance)
                    )

        # JSN-SR04T 권장 측정 간격 확보
        time.sleep(0.06)

except KeyboardInterrupt:

    print("\n측정 종료")

finally:

    GPIO.output(TRIG, GPIO.LOW)
    GPIO.cleanup()