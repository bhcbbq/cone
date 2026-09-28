import Jetson.GPIO as GPIO
import time

TRIG = 12
ECHO = 16

GPIO.setmode(GPIO.BOARD)
GPIO.setup(TRIG, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(ECHO, GPIO.IN)

try:
    for i in range(10):
        print(f"\n--- {i+1}번째 측정 ---")

        # AJ-SR04M Mode 1 Trigger
        GPIO.output(TRIG, GPIO.HIGH)
        time.sleep(0.0001)   # 100 us
        GPIO.output(TRIG, GPIO.LOW)

        # ECHO HIGH 기다리기
        start = time.time()

        while GPIO.input(ECHO) == GPIO.LOW:
            if time.time() - start > 0.1:
                print("ECHO 없음")
                break

        else:
            echo_start = time.time()

            while GPIO.input(ECHO) == GPIO.HIGH:
                if time.time() - echo_start > 0.1:
                    print("ECHO 너무 김")
                    break

            else:
                echo_end = time.time()
                duration = echo_end - echo_start
                distance = duration * 34300 / 2

                print(f"거리 = {distance:.2f} cm")

        time.sleep(0.2)

finally:
    GPIO.cleanup()