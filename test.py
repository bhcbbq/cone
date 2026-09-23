import Jetson.GPIO as GPIO
import time

TRIG = 12
ECHO = 16

GPIO.setmode(GPIO.BOARD)

GPIO.setup(TRIG, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(ECHO, GPIO.IN)

print("ECHO 상태 확인 시작")
print("TRIG = Pin 12")
print("ECHO = Pin 16")
print()

try:
    while True:
        # TRIG 신호 발생
        GPIO.output(TRIG, GPIO.LOW)
        time.sleep(0.000002)

        GPIO.output(TRIG, GPIO.HIGH)
        time.sleep(0.000012)

        GPIO.output(TRIG, GPIO.LOW)

        # ECHO 상태를 일정 시간 확인
        start = time.monotonic()

        echo_detected = False

        while time.monotonic() - start < 0.1:
            if GPIO.input(ECHO) == GPIO.HIGH:
                echo_detected = True
                print(">>> ECHO HIGH 감지!")
                break

        if not echo_detected:
            print("ECHO 없음")

        time.sleep(0.2)

except KeyboardInterrupt:
    print("\n종료")

finally:
    GPIO.output(TRIG, GPIO.LOW)
    GPIO.cleanup()