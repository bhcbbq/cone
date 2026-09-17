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
print("  AJ-SR04M 정밀 초음파 테스트 시작 (Edge 감지) ")
print("========================================")

try:
    while True:
        # 1. Trig 신호 전송
        GPIO.output(TRIG_PIN, GPIO.HIGH)
        time.sleep(0.00001)  # 10us
        GPIO.output(TRIG_PIN, GPIO.LOW)

        # 2. Echo 신호 대기 (Edge 감지 방식)
        # 0.1초 동안 Echo 핀의 RISING(0->1) 상태 변화 대기
        if not GPIO.wait_for_edge(ECHO_PIN, GPIO.RISING, timeout=100):
            print("[에러] Echo 신호 인식 실패 (Timeout)")
            time.sleep(0.5)
            continue
        
        # Echo 신호 수신 시작 시점
        pulse_start = time.time()

        # 0.1초 동안 Echo 핀의 FALLING(1->0) 상태 변화 대기
        if not GPIO.wait_for_edge(ECHO_PIN, GPIO.FALLING, timeout=100):
            print("[에러] Echo 신호 수신 실패 (Timeout)")
            time.sleep(0.5)
            continue

        # Echo 신호 수신 종료 시점
        pulse_end = time.time()

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