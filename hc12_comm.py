import serial

def init_hc12(port='/dev/ttyUSB1', baudrate=9600):
    """
    [무선 초기화 함수] 젯슨 나노와 HC-12 모듈을 연결합니다.
    """
    try:
        ser = serial.Serial(port, baudrate, timeout=0.1)
        print("[알림] HC-12 무선 통신 모듈이 준비되었습니다!")
        return ser
    except:
        # 모듈이 안 꽂혀 있어도 프로그램이 뻗지 않도록 예외 처리
        print("[경고] HC-12 포트를 찾을 수 없습니다. 무선 전송이 생략됩니다.")
        return None

def send_distance(ser, distance):
    """
    [데이터 송신 함수] 거리를 문자열로 예쁘게 포장해서 전파로 쏩니다.
    """
    if ser:
        # 소수점 둘째 자리(%.2f)까지 맞춰서 "DIST: 12.34 m" 형태로 제작
        send_msg = f"DIST: {distance:.2f} m\n"
        
        # 문자열을 전기 신호(바이트 배열)로 인코딩해서 전송
        ser.write(send_msg.encode('utf-8'))

def close_hc12(ser):
    """
    [포트 닫기 함수] 사용이 끝난 통신 포트를 안전하게 닫아줍니다.
    """
    if ser:
        ser.close()