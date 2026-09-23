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
print("TRIG : Pin 12")
print("ECHO : Pin 16")
print("==============================")

try:
    while True:

        # 이전 ECHO 상태 확인
        if GPIO.input(ECHO) == GPIO.HIGH:
            print("ECHO가 이미 HIGH 상태입니다.")
            
            wait_start = time.monotonic()

            while GPIO.input(ECHO) == GPIO.HIGH:
                if time.monotonic() - wait_start > 0.1:
                    break

        # -------------------------
        # TRIG 신호 발생
        # -------------------------
        GPIO.output(TRIG, GPIO.LOW)
        time.sleep(0.000002)

        GPIO.output(TRIG, GPIO.HIGH)
        time.sleep(0.000012)

        GPIO.output(TRIG, GPIO.LOW)

        print("TRIG 전송 완료")

        # -------------------------
        # ECHO HIGH 기다리기
        # -------------------------
        timeout_start = time.monotonic()

        while GPIO.input(ECHO) == GPIO.LOW:

            if time.monotonic() - timeout_start > 0.1:
                print("ECHO 감지 안됨")
                break

        else:

            # ECHO HIGH 감지
            start = time.monotonic()

            print("ECHO HIGH 감지!")

            # ECHO LOW 기다리기
            while GPIO.input(ECHO) == GPIO.HIGH:

                if time.monotonic() - start > 0.1:
                    print("ECHO 종료 안됨")
                    break

            else:

                end = time.monotonic()

                echo_time = end - start

                distance = echo_time * 34300.0 / 2.0

                print(
                    "ECHO 시간: {:.6f} sec".format(echo_time)
                )

                if 20 <= distance <= 600:

                    print(
                        "거리: {:.2f} cm".format(distance)
                    )

                else:

                    print(
                        "잘못된 측정값: {:.2f} cm".format(distance)
                    )

        print("------------------------------")

        # 다음 측정까지 대기
        time.sleep(0.1)

except KeyboardInterrupt:

    print("\n측정 종료")

finally:

    GPIO.output(TRIG, GPIO.LOW)
    GPIO.cleanup()