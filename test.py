import Jetson.GPIO as GPIO
import time

TRIG_PIN = 12
ECHO_PIN = 16

GPIO.setmode(GPIO.BOARD)

GPIO.setup(TRIG_PIN, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(ECHO_PIN, GPIO.IN)

print("==============================")
print(" JSN-SR04T 거리 측정 테스트")
print("==============================")
print("TRIG : Pin 12")
print("ECHO : Pin 16")
print("종료 : Ctrl + C")
print()

try:
    while True:

        # 이전 ECHO 신호가 끝날 때까지 잠시 대기
        time.sleep(0.05)

        # TRIG 10us 펄스
        GPIO.output(TRIG_PIN, GPIO.LOW)
        time.sleep(0.00001)

        GPIO.output(TRIG_PIN, GPIO.HIGH)
        time.sleep(0.00001)

        GPIO.output(TRIG_PIN, GPIO.LOW)

        # ECHO가 HIGH가 되기를 기다림
        wait_start = time.perf_counter()

        while GPIO.input(ECHO_PIN) == GPIO.LOW:
            if time.perf_counter() - wait_start > 0.1:
                break

        # ECHO가 HIGH가 된 순간
        echo_start = time.perf_counter()

        # ECHO가 LOW가 되기를 기다림
        while GPIO.input(ECHO_PIN) == GPIO.HIGH:
            if time.perf_counter() - echo_start > 0.1:
                break

        echo_end = time.perf_counter()

        pulse_width = echo_end - echo_start

        # 거리 계산
        distance = pulse_width * 34300 / 2

        if pulse_width >= 0.1:
            print("측정 실패")
        else:
            print("거리: {:.1f} cm".format(distance))

except KeyboardInterrupt:
    print("\n프로그램 종료")

finally:
    GPIO.cleanup()