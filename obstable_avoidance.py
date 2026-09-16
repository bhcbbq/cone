import time
import Jetson.GPIO as GPIO

# [핀 설정] 3번 항목 확정 후 젯슨 40핀 헤더의 물리 번호로 변경해 주세요.
# 예: Pin 33 (GPIO13/PWM), Pin 37 (GPIO12) 등
TRIG_PIN = 33  # Jetson BOARD Pin 번호 (Trig -> LV1 -> HV1 -> Sensor Trig)
ECHO_PIN = 37  # Jetson BOARD Pin 번호 (Sensor Echo -> HV2 -> LV2 -> Echo)

def init_ultrasonic():
    """AJ-SR04M 센서 및 Jetson GPIO 초기화"""
    GPIO.setmode(GPIO.BOARD)  # 물리적 핀 번호 기준 사용
    GPIO.setup(TRIG_PIN, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(ECHO_PIN, GPIO.IN)
    time.sleep(0.2)

def get_distance():
    """AJ-SR04M 방수 초음파 센서 거리 측정 (cm)"""
    # Trig 신호 방출 (10us 이상)
    GPIO.output(TRIG_PIN, GPIO.HIGH)
    time.sleep(0.000015)  # AJ-SR04M 안정 구동을 위해 15us 유지
    GPIO.output(TRIG_PIN, GPIO.LOW)

    pulse_start = time.time()
    pulse_end = time.time()
    timeout = time.time() + 0.04  # 최대 대기시간 (약 6m 범위)

    # Echo High 진입 대기
    while GPIO.input(ECHO_PIN) == GPIO.LOW:
        pulse_start = time.time()
        if pulse_start > timeout:
            return 999.0  # 타임아웃 발생 시 안전거리 반환

    # Echo Low 전환 대기
    while GPIO.input(ECHO_PIN) == GPIO.HIGH:
        pulse_end = time.time()
        if pulse_end > timeout:
            return 999.0

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150  # 음속(343m/s) 기반 계산
    return round(distance, 2)

def cleanup():
    """프로그램 종료 시 GPIO 리소스 해제"""
    GPIO.cleanup()