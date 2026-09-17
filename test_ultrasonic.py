import time

import Jetson.GPIO as GPIO


# Jetson Orin Nano 40핀 헤더의 물리적 핀 번호
TRIG_PIN = 29
ECHO_PIN = 31

SPEED_OF_SOUND = 34300  # cm/s
TIMEOUT = 0.05


def measure_distance():
    # 초음파 센서에 10us TRIG 신호 전송
    GPIO.output(TRIG_PIN, GPIO.LOW)
    time.sleep(0.000002)

    GPIO.output(TRIG_PIN, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(TRIG_PIN, GPIO.LOW)

    # ECHO가 HIGH가 될 때까지 대기
    wait_started = time.perf_counter()

    while GPIO.input(ECHO_PIN) == GPIO.LOW:
        if time.perf_counter() - wait_started > TIMEOUT:
            return None

    pulse_started = time.perf_counter()

    # ECHO가 다시 LOW가 될 때까지 대기
    while GPIO.input(ECHO_PIN) == GPIO.HIGH:
        if time.perf_counter() - pulse_started > TIMEOUT:
            return None

    pulse_ended = time.perf_counter()
    pulse_time = pulse_ended - pulse_started

    # 초음파가 왕복했으므로 2로 나눔
    return pulse_time * SPEED_OF_SOUND / 2


GPIO.setmode(GPIO.BOARD)
GPIO.setup(TRIG_PIN, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(ECHO_PIN, GPIO.IN)

print("초음파 거리 측정을 시작합니다. 종료하려면 Ctrl+C를 누르세요.")

try:
    while True:
        distance = measure_distance()

        if distance is None:
            print("측정 실패: 센서 응답이 없습니다.")
        else:
            print(f"거리: {distance:.1f} cm")

        time.sleep(0.1)

except KeyboardInterrupt:
    print("\n측정을 종료합니다.")

finally:
    GPIO.output(TRIG_PIN, GPIO.LOW)
    GPIO.cleanup()