import serial
import json

def init_ugv(port='/dev/ttyUSB0', baudrate=115200):
    try:
        ser = serial.Serial(port, baudrate, timeout=0.1)
        print("[알림] UGV02와 정상적으로 연결되었습니다!")
        return ser
    except Exception as e:
        print(f"[에러] UGV02 연결 실패: {e}")
        return None

def read_odometry(ser, dt):
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
    현재 오차와 이전 오차를 비교하여 PID 제어를 통해 모터의 각속도(Z)를 계산합니다.
    """
    derivative = (current_error - previous_error) / dt
    angular_z = -(Kp * current_error + Kd * derivative)
    return angular_z

def send_driving_command(ser, base_speed, angular_z):
    if ser:
        command_dict = {"T": 13, "X": base_speed, "Z": round(angular_z, 3)}
        ser.write((json.dumps(command_dict) + '\n').encode('utf-8'))

def stop_ugv(ser):
    if ser:
        stop_command = '{"T":13, "X":0.0, "Z":0.0}\n'
        ser.write(stop_command.encode('utf-8'))
        ser.close()