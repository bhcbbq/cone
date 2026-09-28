import Jetson.GPIO as GPIO
import time

TRIG = 12
ECHO = 16

GPIO.setmode(GPIO.BOARD)

GPIO.setup(TRIG, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(ECHO, GPIO.IN)

print("====================================")
print(" JSN-SR04T ECHO TEST")
print("====================================")
print("TRIG : Physical Pin 12")
print("ECHO : Physical Pin 16")
print()

try:
    for i in range(10):

        print("측정 {} / 10".format(i + 1))

        # 센서 안정화
        GPIO.output(TRIG, GPIO.LOW)
        time.sleep(0.05)

        # 10 us TRIG 펄스
        GPIO.output(TRIG, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(TRIG, GPIO.LOW)

        # ECHO가 HIGH가 될 때까지 최대 0.2초 기다림
        wait_start = time.time()

        while GPIO.input(ECHO) == GPIO.LOW:
            if time.time() - wait_start > 0.2:
                print("  -> ECHO 없음")
                break

        else:
            # ECHO HIGH 감지
            echo_start = time.time()

            # ECHO가 LOW가 될 때까지 기다림
            while GPIO.input(ECHO) == GPIO.HIGH:
                if time.time() - echo_start > 0.2:
                    print("  -> ECHO HIGH 유지시간 초과")
                    break

            echo_end = time.time()

            pulse_time = echo_end - echo_start

            distance = (pulse_time * 34300) / 2

            print("  -> ECHO 감지!")
            print("  -> 거리: {:.2f} cm".format(distance))

        time.sleep(0.5)

except KeyboardInterrupt:
    print("\n사용자가 종료했습니다.")

finally:
    GPIO.cleanup()
    print("\nGPIO 정리 완료")