import Jetson.GPIO as GPIO
import time

TRIG = 12
ECHO = 16

GPIO.setmode(GPIO.BOARD)

GPIO.setup(TRIG, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(ECHO, GPIO.IN)

print("JSN-SR04T 테스트")
print("TRIG = Pin 12")
print("ECHO = Pin 16")
print("종료: Ctrl+C")
print()

try:
    while True:

        # 센서 안정화
        time.sleep(0.06)

        # TRIG 10us
        GPIO.output(TRIG, GPIO.LOW)
        time.sleep(0.000002)

        GPIO.output(TRIG, GPIO.HIGH)
        time.sleep(0.000010)

        GPIO.output(TRIG, GPIO.LOW)

        # ECHO가 HIGH가 되기를 기다림
        start_wait = time.perf_counter()

        while GPIO.input(ECHO) == GPIO.LOW:
            if time.perf_counter() - start_wait > 0.07:
                break

        # 정말 HIGH가 되었는지 확인
        if GPIO.input(ECHO) == GPIO.LOW:
            print("ECHO 없음")
            continue

        # HIGH 시작
        echo_start = time.perf_counter()

        # ECHO가 LOW가 되기를 기다림
        while GPIO.input(ECHO) == GPIO.HIGH:
            if time.perf_counter() - echo_start > 0.07:
                break

        echo_end = time.perf_counter()

        # 실제 ECHO HIGH 시간
        pulse_time = echo_end - echo_start

        distance = pulse_time * 34300 / 2

        if distance < 20 or distance > 600:
            print("비정상 측정: {:.1f} cm".format(distance))
        else:
            print("거리: {:.1f} cm".format(distance))

except KeyboardInterrupt:
    print("\n프로그램 종료")

finally:
    GPIO.cleanup()