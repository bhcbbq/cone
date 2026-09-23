import Jetson.GPIO as GPIO
import time

TRIG = 12
ECHO = 16

GPIO.setmode(GPIO.BOARD)

GPIO.setup(TRIG, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(ECHO, GPIO.IN)

print("==============================")
print(" JSN-SR04T ECHO 진단 테스트")
print("==============================")
print("TRIG : Jetson Pin 12")
print("ECHO : Jetson Pin 16")
print("==============================")
print()

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
        # 2. ECHO HIGH 기다리기
        # -------------------------
        start_wait = time.monotonic()

        echo_detected = False

        while time.monotonic() - start_wait < 0.1:

            if GPIO.input(ECHO) == GPIO.HIGH:
                echo_detected = True
                break

        # -------------------------
        # 3. 결과 출력
        # -------------------------
        if echo_detected:

            print(">>> ECHO HIGH 감지!")

            # ECHO가 LOW가 될 때까지 기다림
            start = time.monotonic()

            while GPIO.input(ECHO) == GPIO.HIGH:

                if time.monotonic() - start > 0.1:
                    break

            end = time.monotonic()

            echo_time = end - start

            distance = echo_time * 34300 / 2

            print("ECHO 시간 : {:.6f} sec".format(echo_time))
            print("계산 거리 : {:.2f} cm".format(distance))
            print()

        else:

            print("ECHO 없음")

        time.sleep(0.2)


except KeyboardInterrupt:

    print()
    print("테스트 종료")


finally:

    GPIO.output(TRIG, GPIO.LOW)
    GPIO.cleanup()