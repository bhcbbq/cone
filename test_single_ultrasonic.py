import Jetson.GPIO as GPIO
import time

TRIG_PIN = 29
ECHO_PIN = 18

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)

GPIO.output(TRIG_PIN, GPIO.LOW)
time.sleep(0.5)

print("========================================")
print("  AJ-SR04M 신호 디버깅 테스트 시작")
print("========================================")

try:
    while True:
        # 1. Trig 신호 10us
        GPIO.output(TRIG_PIN, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(TRIG_PIN, GPIO.LOW)

        # 2. Echo 신호 수신 시간 측정 (타임아웃 적용)
        start_time = time.time()
        timeout = start_time + 0.05  # 50ms 대기 (최대 거리용)

        pulse_start = time.time()
        echo_received = False

        # LOW -> HIGH (Echo 신호 시작 대기)
        while time.time() < timeout:
            if GPIO.input(ECHO_PIN) == 1:
                pulse_start = time.time()
                echo_received = True
                break

        if not echo_received:
            print("[신호 에러] Echo 신호(HIGH)를 받지 못함 -> 레벨변환기/GND/배선 점검 필요")
            time.sleep(0.5)
            continue

        # HIGH -> LOW (Echo 신호 종료 대기)
        pulse_end = time.time()
        timeout_end = pulse_start + 0.05
        while time.time() < timeout_end:
            if GPIO.input(ECHO_PIN) == 0:
                pulse_end = time.time()
                break

        # 3. 거리 계산
        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 17150

        if distance >= 20:
            print(f"-> 측정 성공: {distance:.2f} cm")
        else:
            print(f"-> 신호는 들어오나 거리가 너무 가까움: {distance:.2f} cm")

        time.sleep(0.4)

except KeyboardInterrupt:
    print("\n테스트를 종료합니다.")
finally:
    GPIO.cleanup()