import Jetson.GPIO as GPIO
import time

ECHO = 16

GPIO.setmode(GPIO.BOARD)
GPIO.setup(ECHO, GPIO.IN)

print("Jetson ECHO 수신 테스트")
print("Arduino가 TRIG를 보내고 있습니다.")

try:
    while True:

        # ECHO HIGH 대기
        start_wait = time.time()

        while GPIO.input(ECHO) == GPIO.LOW:
            if time.time() - start_wait > 0.1:
                print("ECHO 없음")
                break

        else:
            # ECHO HIGH 시작
            start = time.time()

            # ECHO LOW 대기
            while GPIO.input(ECHO) == GPIO.HIGH:
                if time.time() - start > 0.1:
                    print("ECHO LOW 없음")
                    break

            else:
                end = time.time()

                echo_time = end - start
                distance = echo_time * 34300 / 2

                print(
                    "ECHO 시간: {:.6f} sec   거리: {:.2f} cm"
                    .format(echo_time, distance)
                )

        time.sleep(0.05)

except KeyboardInterrupt:
    print("\n테스트 종료")

finally:
    GPIO.cleanup()