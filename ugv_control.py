import serial
import json

# 수정: ESP32 기본 연결 포트를 '/dev/ttyACM0'로 변경
def init_ugv(port='/dev/ttyACM0', baudrate=115200):
    """
    [초기화 함수] 젯슨 나노와 UGV02 하체를 USB 시리얼 통신으로 연결합니다.
    """
    try:
        ser = serial.Serial(port, baudrate, timeout=0.1)
        print("[알림] UGV02와 정상적으로 연결되었습니다!")
        return ser
    except Exception as e:
        print(f"[에러] UGV02 연결 실패: {e}")
        return None

def read_odometry(ser, dt):
    """
    [거리 측정 함수] 바퀴 엔코더 값을 읽어와서 이동한 거리를 계산(시간 적분)합니다.
    """
    distance_added = 0.0
    while ser and ser.in_waiting > 0:
        try:
            raw_data = ser.readline().decode('utf-8').strip()
            if not raw_data: continue
            
            feedback = json.loads(raw_data)
            if feedback.get("T") == 1001: 
                left_speed = feedback.get("L", 0.0)
                right_speed = feedback.get("R", 0.0)
                real_speed = (left_speed + right_speed) / 2.0
                distance_added += real_speed * dt
        except json.JSONDecodeError:
            pass
    return distance_added

def calculate_pid(current_error, previous_error, dt, Kp=0.005, Kd=0.002):
    """
    [조향 두뇌 함수] 차선 오차를 바탕으로 모터를 얼마나 꺾을지(각속도) 계산합니다.
    """
    derivative = (current_error - previous_error) / dt
    angular_z = -(Kp * current_error + Kd * derivative)
    
    # 수정: 급회전 및 기체 전복을 막기 위한 조향 각속도 상한선(Clamp) 추가
    MAX_ANGULAR_Z = 1.5
    angular_z = max(min(angular_z, MAX_ANGULAR_Z), -MAX_ANGULAR_Z)
    
    return angular_z

def send_driving_command(ser, base_speed, angular_z):
    """
    [명령 하달 함수] 계산된 속도를 하체 보드에 JSON 양식으로 쏴줍니다.
    """
    if ser:
        command_dict = {"T": 13, "X": base_speed, "Z": round(angular_z, 3)}
        ser.write((json.dumps(command_dict) + '\n').encode('utf-8'))

def stop_ugv(ser):
    """
    [안전 정지 함수] 로봇을 강제로 멈춥니다.
    """
    if ser:
        stop_command = '{"T":13, "X":0.0, "Z":0.0}\n'
        ser.write(stop_command.encode('utf-8'))
        ser.close()