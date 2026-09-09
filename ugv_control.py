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
    latest_speed = 0.0
    has_data = False

    while ser and ser.in_waiting > 0:
        try:
            # 노이즈(Garbage byte)로 인한 시스템 다운 방지 (errors='ignore')
            raw_data = ser.readline().decode('utf-8', errors='ignore').strip()
            if not raw_data:
                continue

            feedback = json.loads(raw_data)

            # 수정: JSON 파싱은 성공했지만 dict가 아닌 경우(숫자, null, 리스트 등)
            # feedback.get(...)에서 AttributeError가 나서 스레드가 죽는 것을 방지
            if not isinstance(feedback, dict):
                continue

            if feedback.get("T") == 1001:
                left_speed = feedback.get("L", 0.0)
                right_speed = feedback.get("R", 0.0)
                latest_speed = (left_speed + right_speed) / 2.0
                has_data = True

        except json.JSONDecodeError:
            # 이 줄만 버리고 다음 줄 계속 읽기
            pass
        except (serial.SerialException, OSError) as e:
            # 수정: 케이블 분리 등으로 포트 자체가 끊긴 경우 -> 더 읽기를 시도하지 않고 즉시 탈출
            print(f"[에러] 시리얼 읽기 실패 (오도메트리): {e}")
            break
        except Exception as e:
            # 수정: 예상 못한 예외로 백그라운드 제어 스레드 전체가 조용히 죽는 것을 방지
            print(f"[경고] 오도메트리 데이터 처리 중 알 수 없는 오류: {e}")
            continue

    # 버퍼를 다 비우고 최신 속도값으로 한 번만 거리 계산 (중복 누적 버그 방지)
    if has_data:
        distance_added = latest_speed * dt

    return distance_added

def calculate_pid(current_error, previous_error, dt, Kp=0.005, Kd=0.002):
    """
    [조향 두뇌 함수] 차선 오차를 바탕으로 모터를 얼마나 꺾을지(각속도) 계산합니다.
    """
    derivative = (current_error - previous_error) / dt

    # 수정: UGV02의 "Z" 값은 ROS Twist(angular.z) 컨벤션을 그대로 따름
    # (양수 = 반시계 = 전진 기준 좌회전, Waveshare 공식 문서 기준)
    # error > 0 은 "좌회전이 필요한 상태"이므로 그대로 양수로 내보내야 함.
    # 기존 코드는 앞에 '-' 부호가 붙어 있어 차선 이탈 시 반대 방향으로 꺾이는 문제가 있었음.
    # ⚠️ 실제 로봇에서 반드시 바퀴를 들고 벤치 테스트로 좌/우 방향을 재확인할 것!
    angular_z = Kp * current_error + Kd * derivative

    # 급회전 및 기체 전복을 막기 위한 조향 각속도 상한선(Clamp)
    MAX_ANGULAR_Z = 1.5
    angular_z = max(min(angular_z, MAX_ANGULAR_Z), -MAX_ANGULAR_Z)

    return angular_z

def send_driving_command(ser, base_speed, angular_z):
    """
    [명령 하달 함수] 계산된 속도를 하체 보드에 JSON 양식으로 쏴줍니다.
    """
    if ser:
        command_dict = {"T": 13, "X": base_speed, "Z": round(angular_z, 3)}
        try:
            ser.write((json.dumps(command_dict) + '\n').encode('utf-8'))
        except (serial.SerialException, OSError) as e:
            # 수정: 케이블 분리 등으로 쓰기가 실패해도 제어 스레드가 죽지 않도록 예외 처리
            print(f"[에러] 주행 명령 전송 실패: {e}")

def stop_ugv(ser):
    """
    [안전 정지 함수] 로봇을 강제로 멈춥니다.
    """
    if ser:
        stop_command = '{"T":13, "X":0.0, "Z":0.0}\n'
        try:
            ser.write(stop_command.encode('utf-8'))
        except (serial.SerialException, OSError) as e:
            # 수정: 종료 시점에 포트가 이미 끊겨 있어도 프로그램이 죽지 않도록 처리
            print(f"[에러] 정지 명령 전송 실패: {e}")
        finally:
            try:
                ser.close()
            except Exception:
                pass