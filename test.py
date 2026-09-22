import Jetson.GPIO as GPIO
import time

ECHO = 16

GPIO.setmode(GPIO.BOARD)
GPIO.setup(ECHO, GPIO.IN)

print("ECHO 평상시 상태 테스트")
print("10초 동안 ECHO 상태를 확인합니다.")
print()

try:
    for i in range(100):
        state = GPIO.input(ECHO)

        if state == GPIO.HIGH:
            print("ECHO = HIGH")
        else:
            print("ECHO = LOW")

        time.sleep(0.1)

except KeyboardInterrupt:
    pass

finally:
    GPIO.cleanup()