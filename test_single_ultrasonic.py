import Jetson.GPIO as GPIO
import time

# BOARD 핀 번호 설정 (물리 핀 번호 기준)
TRIG_PIN = 29  # Jetson Pin 29
ECHO_PIN = 22  # Jetson Pin 22

def setup():
    """GPIO 핀 초기화"""
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(TRIG_PIN, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(ECHO_PIN, GPIO.IN)
    time.sleep(0.5)  # 센서 안정화 대기

def measure_distance():
    """초음파 센서로부터 거리를 측정 (단위: cm)"""
    # 10us 동안 High 신호를 주어 초음파 발사
    GPIO.output(TRIG_PIN, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(TRIG_PIN, GPIO.LOW)

    pulse_start = time.time()
    pulse_end = time.time()
    
    # 무한 루프 방지를 위한 타임아웃 설정 (0.03초 = 약 5m 측정 범위 limit)
    timeout = time.time() + 0.03

    # Echo 핀이 High로 바뀔 때까지 대기 (신호 발사 시점)
    while GPIO.input(ECHO_PIN) == 0:
        pulse_start = time.time()
        if pulse_start > timeout:
            return None

    # Echo 핀이 Low로 바뀔 때까지 대기 (신호 반사 수신 시점)
    while GPIO.input(ECHO_PIN) == 1:
        pulse_end = time.time()
        if pulse_end > timeout:
            return None

    # 왕복 시간 계산 후 음속(343m/s) 기준 거리 계산
    pulse_duration = pulse_end - pulse_start
    distance = (pulse_duration * 34300) / 2
    return round(distance, 2)

def main():
    setup()
    print("========================================")
    print("  AJ-SR04M 초음파 센서 단독 테스트 시작 ")
    print("  (종료하려면 Ctrl+C 를 누르세요)       ")
    print("========================================")

    try:
        while True:
            dist = measure_distance()
            
            if dist is None:
                print("[경고] 측정 범위 초과 또는 신호 수신 실패")
            else:
                # 30cm 이내 장애물 감지 시 경고 문구 출력
                if dist <= 30.0:
                    print(f"거리: {dist} cm  ---> [!! 장애물 감지 !!]")
                else:
                    print(f"거리: {dist} cm")
            
            time.sleep(0.2)

    except KeyboardInterrupt:
        print("\n테스트를 종료합니다.")
    finally:
        GPIO.cleanup()  # GPIO 핀 초기화

if __name__ == "__main__":
    main()
    