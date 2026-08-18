import serial

def init_hc12(port='/dev/ttyUSB1', baudrate=9600):
    try:
        ser = serial.Serial(port, baudrate, timeout=0.1)
        print("[알림] HC-12 무선 통신 모듈이 준비되었습니다!")
        return ser
    except:
        print("[경고] HC-12 포트를 찾을 수 없습니다. 무선 전송이 생략됩니다.")
        return None

def send_distance(ser, distance):
    if ser:
        send_msg = f"DIST: {distance:.2f} m\n"
        ser.write(send_msg.encode('utf-8'))

def close_hc12(ser):
    if ser:
        ser.close()