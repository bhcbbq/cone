import Jetson.GPIO as GPIO
import time

ECHO = 16

GPIO.setmode(GPIO.BOARD)
GPIO.setup(ECHO, GPIO.IN)

try:
    print("Pin 16 감시 시작")

    end_time = time.time() + 5

    while time.time() < end_time:
        if GPIO.input(ECHO) == GPIO.HIGH:
            start = time.perf_counter()

            while GPIO.input(ECHO) == GPIO.HIGH:
                pass

            end = time.perf_counter()

            duration = (end - start) * 1_000_000

            print(f"ECHO 감지: {duration:.0f} us")

except KeyboardInterrupt:
    pass

finally:
    GPIO.cleanup()