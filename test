import Jetson.GPIO as GPIO
import time

TRIG_PIN = 12
ECHO_PIN = 16

GPIO.setmode(GPIO.BOARD)

GPIO.setup(TRIG_PIN, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(ECHO_PIN, GPIO.IN)

print("================================")
print("JSN-SR04T 초음파 센서 테스트")
print("TRIG : 물리 핀 12번")
print("ECHO : 물리 핀 16번")
print("================================")
print("센서 앞에 물체를 놓아주세요.")
print("종료: Ctrl + C")

try:
    while True:

        # Trigger 신호
        GPIO.output(TRIG_PIN, GPIO.LOW)
        time.sleep(0.000002)

        GPIO.output(TRIG_PIN, GPIO.HIGH)
        time.sleep(0.000010)
        GPIO.output(TRIG_PIN, GPIO.LOW)

        # ECHO HIGH 기다리기
        timeout = time.perf_counter()

        while GPIO.input(ECHO_PIN) == GPIO.LOW:
            if time.perf_counter() - timeout > 0.1:
                break

        pulse_start = time.perf_counter()

        # ECHO LOW 기다리기
        while GPIO.input(ECHO_PIN) == GPIO.HIGH:
            if time.perf_counter() - pulse_start > 0.1:
                break

        pulse_end = time.perf_counter()

        pulse_time = pulse_end - pulse_start

        # 거리 계산
        distance = pulse_time * 34300 / 2

        if pulse_time >= 0.1:
            print("측정 실패")
        else:
            print("거리: {:.1f} cm".format(distance))

        time.sleep(0.1)

except KeyboardInterrupt:
    print("\n프로그램 종료")

finally:
    GPIO.cleanup()