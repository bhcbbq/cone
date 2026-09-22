import Jetson.GPIO as GPIO
import time

TRIG = 12
ECHO = 16

GPIO.setmode(GPIO.BOARD)

GPIO.setup(TRIG, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(ECHO, GPIO.IN)

print("ECHO 신호 테스트")
print("Ctrl+C 로 종료")
print()

try:
    while True:

        # TRIG 신호 발생
        GPIO.output(TRIG, GPIO.LOW)
        time.sleep(0.000002)

        GPIO.output(TRIG, GPIO.HIGH)
        time.sleep(0.000010)

        GPIO.output(TRIG, GPIO.LOW)

        # ECHO가 HIGH가 되는지 확인
        start = time.perf_counter()

        echo_detected = False

        while time.perf_counter() - start < 0.05:

            if GPIO.input(ECHO) == GPIO.HIGH:
                echo_detected = True
                break

        if echo_detected:
            print("★ ECHO HIGH 감지!")

            # HIGH가 유지되는 시간 측정
            pulse_start = time.perf_counter()

            while GPIO.input(ECHO) == GPIO.HIGH:
                if time.perf_counter() - pulse_start > 0.05:
                    break

            pulse_end = time.perf_counter()

            pulse_width = pulse_end - pulse_start

            print("ECHO 시간: {:.6f} sec".format(pulse_width))
            print()

        else:
            print("ECHO 없음")

        time.sleep(0.1)

except KeyboardInterrupt:
    print("\n종료")

finally:
    GPIO.cleanup()