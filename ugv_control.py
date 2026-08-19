import serial
import json

def init_ugv(port='/dev/ttyUSB0', baudrate=115200):
    """
    [초기화 함수] 젯슨 나노와 UGV02 하체를 USB 시리얼 통신으로 연결합니다.
    - baudrate 115200: 기기간 통신 속도 약속
    - 반환값(ser): 앞으로 통신에 사용할 '통신 파이프(객체)'
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
    - dt: 이전 계산 이후 흘러간 시간 (약 0.02초)
    """
    distance_added = 0.0
    # 수신 대기 중인 데이터가 있으면 모두 읽어들임
    while ser and ser.in_waiting > 0:
        try:
            # 하체 보드가 보내는 JSON 문자열 데이터를 한 줄 읽어옴
            raw_data = ser.readline().decode('utf-8').strip()
            if not raw_data: continue
            
            # 문자열을 파이썬 딕셔너리(사전) 형태로 변환
            feedback = json.loads(raw_data)
            
            # T: 1001은 UGV02 제조사(Waveshare)에서 정한 '엔코더 피드백' 코드번호
            if feedback.get("T") == 1001: 
                left_speed = feedback.get("L", 0.0)   # 왼쪽 바퀴 속도
                right_speed = feedback.get("R", 0.0)  # 오른쪽 바퀴 속도
                
                # 로봇의 진짜 전진 속도(v) = 양쪽 바퀴 속도의 평균
                real_speed = (left_speed + right_speed) / 2.0
                
                # 거속시 공식: 이동 거리 = 속도(v) * 시간(dt)
                distance_added += real_speed * dt
        except json.JSONDecodeError:
            pass # 통신 노이즈로 데이터가 깨지면 무시하고 넘어감
    return distance_added

def calculate_pid(current_error, previous_error, dt, Kp=0.005, Kd=0.002):
    """
    [조향 두뇌 함수] 차선 오차를 바탕으로 모터를 얼마나 꺾을지(각속도) 계산합니다.
    - Kp (비례 제어): 오차가 클수록 강하게 핸들을 꺾음
    - Kd (미분 제어): 오차가 변하는 '속도'를 계산해 핸들이 떨리는(진동) 현상을 막아줌
    """
    # 에러의 미분값 = (현재 에러 - 아까 에러) / 걸린 시간
    derivative = (current_error - previous_error) / dt
    
    # 조향 각속도(Z) 도출 수식 (왼쪽/오른쪽 방향을 맞추기 위해 앞에 -를 붙임)
    angular_z = -(Kp * current_error + Kd * derivative)
    return angular_z

def send_driving_command(ser, base_speed, angular_z):
    """
    [명령 하달 함수] 계산된 속도를 하체 보드에 JSON 양식으로 쏴줍니다.
    """
    if ser:
        # T: 13 은 UGV02의 '속도 제어 명령' 코드번호
        # X: 직진 선속도, Z: 회전 각속도
        command_dict = {"T": 13, "X": base_speed, "Z": round(angular_z, 3)}
        
        # 딕셔너리를 다시 문자열로 묶어서 하체로 전송
        ser.write((json.dumps(command_dict) + '\n').encode('utf-8'))

def stop_ugv(ser):
    """
    [안전 정지 함수] 로봇을 강제로 멈춥니다. (프로그램 종료 시 벽 충돌 방지)
    """
    if ser:
        # 직진 속도(X)와 회전 속도(Z)를 모두 0.0으로 명령
        stop_command = '{"T":13, "X":0.0, "Z":0.0}\n'
        ser.write(stop_command.encode('utf-8'))
        ser.close() # 포트 닫기