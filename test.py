import Jetson.GPIO as GPIO
import time

TRIG_PIN = 12
ECHO_PIN = 16

GPIO.setmode(GPIO.BOARD)

GPIO.setup(TRIG_PIN, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(ECHO_PIN, GPIO.IN)

print("초음파 센서 테스트 시작")

try:
    while True:
        # TRIG 신호
        GPIO.output(TRIG_PIN, GPIO.HIGH)
        time.sleep(0.00003)   # 30us
        GPIO.output(TRIG_PIN, GPIO.LOW)

        # ECHO가 HIGH가 될 때까지 대기
        start_wait = time.time()

        while GPIO.input(ECHO_PIN) == 0:
            if time.time() - start_wait > 0.1:
                print("ECHO 없음")
                break

        else:
            pulse_start = time.time()

            # ECHO가 LOW가 될 때까지 대기
            while GPIO.input(ECHO_PIN) == 1:
                if time.time() - pulse_start > 0.1:
                    print("ECHO 너무 김")
                    break
            else:
                pulse_end = time.time()

                pulse_time = pulse_end - pulse_start
                distance = pulse_time * 34300 / 2

                print(f"거리: {distance:.2f} cm")

        time.sleep(0.1)

except KeyboardInterrupt:
    print("\n종료")

finally:
    GPIO.cleanup()