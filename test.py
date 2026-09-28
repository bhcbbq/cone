import Jetson.GPIO as GPIO
import time

TRIG = 12
ECHO = 16

GPIO.setmode(GPIO.BOARD)

GPIO.setup(TRIG, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(ECHO, GPIO.IN)

try:
    for i in range(10):
        print(f"\n--- {i+1}번째 테스트 ---")

        # 센서에 TRIG 신호
        GPIO.output(TRIG, GPIO.HIGH)
        time.sleep(0.0001)   # 100 us
        GPIO.output(TRIG, GPIO.LOW)

        # ECHO가 HIGH가 되는지 약 10ms 동안 확인
        start = time.perf_counter()
        detected = False

        while time.perf_counter() - start < 0.01:
            if GPIO.input(ECHO) == GPIO.HIGH:
                detected = True
                high_start = time.perf_counter()

                # ECHO가 LOW로 내려갈 때까지 측정
                while GPIO.input(ECHO) == GPIO.HIGH:
                    pass

                high_end = time.perf_counter()

                duration = (high_end - high_start) * 1_000_000

                print(f"ECHO 감지! HIGH 시간 = {duration:.0f} us")
                break

        if not detected:
            print("ECHO 없음")

        time.sleep(0.5)

finally:
    GPIO.cleanup()