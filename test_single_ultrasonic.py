import Jetson.GPIO as GPIO
import time

# 핀 번호 설정 (BOARD 핀 번호 기준)
TRIG_PIN = 29
ECHO_PIN = 22

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)

# 초기 신호 LOW 설정
GPIO.output(TRIG_PIN, GPIO.LOW)
time.sleep(0.5)

print("========================================")
print("  AJ-SR04M 정밀 초음파 테스트 시작 ")
print("========================================")

try:
    while True:
        # 1. Trig 신호 전송 (10us)
        GPIO.output(TRIG_PIN, GPIO.HIGH)
        time.sleep(0.00001)  # 10 microseconds
        GPIO.output(TRIG_PIN, GPIO.LOW)

        # 2. Echo 신호 수신 대기 (타임아웃 처리)
        pulse_start = time.time()
        timeout_start = pulse_start

        # Echo가 HIGH가 될 때까지 대기
        while GPIO.input(ECHO_PIN) == 0:
            pulse_start = time.time()
            if pulse_start - timeout_start > 0.1:  # 0.1초 넘으면 타임아웃
                break

        # Echo가 LOW가 될 때까지 대기
        pulse_end = time.time()
        timeout_end = pulse_end

        while GPIO.input(ECHO_PIN) == 1:
            pulse_end = time.time()
            if pulse_end - timeout_end > 0.1:
                break

        # 3. 거리 계산
        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 17150  # 음속 (34300cm/s) / 2

        if 20 <= distance <= 450:
            print(f"측정 거리: {distance:.2f} cm")
        else:
            print(f"[알림] 측정값: {distance:.2f} cm (센서 최소 20cm 이상 유지 필요)")

        # AJ-SR04M 센서는 주기 간격이 최소 0.2~0.3초 필요함
        time.sleep(0.3)

except KeyboardInterrupt:
    print("\n테스트를 종료합니다.")
finally:
    GPIO.cleanup()