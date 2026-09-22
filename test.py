import Jetson.GPIO as GPIO
import time

TRIG = 12
ECHO = 16

GPIO.setmode(GPIO.BOARD)

GPIO.setup(TRIG, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(ECHO, GPIO.IN)

print("ECHO 테스트 시작")
print("센서 앞에 약 40~50cm 거리의 물체를 놓으세요.")

try:
    while True:

        # TRIG 신호
        GPIO.output(TRIG, GPIO.LOW)
        time.sleep(0.000002)

        GPIO.output(TRIG, GPIO.HIGH)
        time.sleep(0.000010)

        GPIO.output(TRIG, GPIO.LOW)

        # ECHO가 HIGH가 되기를 기다림
        start_wait = time.time()

        while GPIO.input(ECHO) == GPIO.LOW:
            if time.time() - start_wait > 0.1:
                print("ECHO HIGH 없음")
                break

        else:
            # ECHO HIGH 시작
            start = time.time()

            # ECHO가 LOW가 되기를 기다림
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

        time.sleep(0.2)

except KeyboardInterrupt:
    print("\n테스트 종료")

finally:
    GPIO.output(TRIG, GPIO.LOW)
    GPIO.cleanup()