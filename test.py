import Jetson.GPIO as GPIO
import time


# ============================================================
# JSN-SR04T + Jetson Orin Nano
#
# TRIG : Physical Pin 12
# ECHO : Physical Pin 16
#
# ECHO는 반드시 레벨시프터를 통과해서 Jetson으로 연결
# ============================================================

TRIG_PIN = 12
ECHO_PIN = 16

GPIO.setmode(GPIO.BOARD)

GPIO.setup(TRIG_PIN, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(ECHO_PIN, GPIO.IN)


def measure_distance():
    """
    JSN-SR04T에서 초음파를 발사하고
    ECHO 시간을 이용하여 거리를 계산한다.

    반환값:
        거리(cm)
        측정 실패 시 None
    """

    # 센서 안정화를 위한 짧은 대기
    time.sleep(0.05)

    # --------------------------------------------------------
    # 1. TRIG 신호 발생
    # --------------------------------------------------------

    GPIO.output(TRIG_PIN, GPIO.LOW)
    time.sleep(0.000002)

    GPIO.output(TRIG_PIN, GPIO.HIGH)
    time.sleep(0.00001)   # 10 us
    GPIO.output(TRIG_PIN, GPIO.LOW)

    # --------------------------------------------------------
    # 2. ECHO가 HIGH가 될 때까지 기다림
    # --------------------------------------------------------

    timeout = time.time() + 0.1

    while GPIO.input(ECHO_PIN) == GPIO.LOW:
        if time.time() > timeout:
            return None

    echo_start = time.time()

    # --------------------------------------------------------
    # 3. ECHO가 LOW가 될 때까지 기다림
    # --------------------------------------------------------

    timeout = time.time() + 0.1

    while GPIO.input(ECHO_PIN) == GPIO.HIGH:
        if time.time() > timeout:
            return None

    echo_end = time.time()

    # --------------------------------------------------------
    # 4. 시간 → 거리 계산
    # --------------------------------------------------------

    pulse_time = echo_end - echo_start

    # 음속 약 343 m/s
    # 왕복 거리이므로 2로 나눔
    distance_cm = (pulse_time * 34300) / 2

    return distance_cm


def main():

    print("====================================")
    print(" JSN-SR04T Ultrasonic Sensor Test")
    print(" Jetson Orin Nano")
    print("====================================")
    print()
    print("TRIG : Physical Pin 12")
    print("ECHO : Physical Pin 16")
    print()
    print("측정을 시작합니다.")
    print("종료하려면 Ctrl + C")
    print()

    try:

        while True:

            distance = measure_distance()

            if distance is None:

                print("ECHO 없음")

            else:

                print("거리 : {:.2f} cm".format(distance))

            time.sleep(0.2)

    except KeyboardInterrupt:

        print()
        print("측정을 종료합니다.")

    finally:

        GPIO.cleanup()
        print("GPIO 정리 완료")


if __name__ == "__main__":
    main()