import cv2
import time
import threading
import numpy as np

# 모듈 임포트
import lane_detection
import ugv_control
from ultrasonic_reader import UltrasonicReader

# 기본 설정 및 스레드 간 데이터 공유 변수
BASE_SPEED = 0.25           # 기본 주행 속도 (m/s)
AVOID_SPEED = 0.20          # 회피 동작 시 속도
TURN_ANGULAR_SPEED = 0.8    # 회피 회전 각속도 (rad/s)

shared_error = 0            
shared_speed = 0.0          
shared_angular_override = None # None이 아니면 PID 조향 대신 이 각속도를 사용

error_lock = threading.Lock()
is_running = True

# 장애물 회피 상태 정의 (State Machine)
STATE_LANE_FOLLOWING = "LANE_FOLLOWING" # 정상 차선 주행
STATE_CHECK_OBSTACLE = "CHECK_OBSTACLE" # 장애물 감지 후 정지 & 판단
STATE_AVOID_TURN    = "AVOID_TURN"    # 넓은 쪽으로 회전
STATE_AVOID_FORWARD = "AVOID_FORWARD" # 장애물 옆 지나치기
STATE_RECOVERY_TURN = "RECOVERY_TURN" # 차선 방향으로 복귀 회전
STATE_BLOCKED_STOP  = "BLOCKED_STOP"  # 양쪽 모두 막힘 (정지)

current_state = STATE_LANE_FOLLOWING

# 거리 임계값 설정 (단위: cm)
FRONT_OBSTACLE_THRESH = 35.0  # 정면 장애물 감지 거리
SIDE_SAFE_CLEARANCE    = 30.0  # 회피 가능한 양 옆 최소 공간 거리

# 하드웨어 객체
ugv_serial = None
ultrasonic = None

# ==========================================
# [백그라운드 스레드] 로봇 모터 제어
# ==========================================
def control_thread_task():
    global shared_error, shared_speed, shared_angular_override, is_running
    
    previous_error = 0
    last_time = time.time()

    print("[알림] 제어 백그라운드 스레드가 시작되었습니다.")

    while is_running:
        current_time = time.time()
        dt = current_time - last_time
        if dt <= 0: dt = 0.001
        last_time = current_time

        # 1. 시리얼 버퍼 읽기
        if ugv_serial:
            ugv_control.read_odometry(ugv_serial, dt)

        # 2. 제어 변수 복사
        with error_lock:
            c_error = shared_error
            c_speed = shared_speed
            c_override = shared_angular_override

        # 3. 조향 계산 (수동 강제 조향 또는 PID 차선 조향)
        if c_override is not None:
            angular_z = c_override
        else:
            angular_z = ugv_control.calculate_pid(c_error, previous_error, dt)
            previous_error = c_error

        # 4. 주행 명령 하달
        if ugv_serial:
            send_angular = angular_z if (c_speed != 0.0 or c_override is not None) else 0.0
            ugv_control.send_driving_command(ugv_serial, c_speed, send_angular)

        time.sleep(0.02)

def gstreamer_pipeline(
    sensor_id=0, capture_width=1280, capture_height=720,
    display_width=640, display_height=360, framerate=30, flip_method=0,
):
    return (
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (sensor_id, capture_width, capture_height, framerate, flip_method, display_width, display_height)
    )

# ==========================================
# [메인 자율주행 실행 파트]
# ==========================================
if __name__ == "__main__":
    
    # 1. UGV 하체 연결 (포트는 환경에 맞춰 확인)
    ugv_serial = ugv_control.init_ugv(port='/dev/ttyACM0', baudrate=115200)
    if not ugv_serial:
        print("[종료] UGV02 연결 실패. 포트를 확인하세요.")
        exit()

    # 2. 아두이노 초음파 센서 모듈 연결 (아두이노 포트 지정)
    ultrasonic = UltrasonicReader(port='/dev/ttyACM1', baudrate=115200)
    if not ultrasonic.start():
        print("[경고] 초음파 센서 포트 연결 실패! 아두이노 포트(/dev/ttyACM1 등)를 점검하세요.")

    # 3. CSI 카메라 초기화
    pipeline = gstreamer_pipeline(flip_method=0)
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        print("[에러] 카메라 초기화 실패!")
        ugv_control.stop_ugv(ugv_serial)
        exit()

    # 4. 백그라운드 제어 스레드 시작
    control_thread = threading.Thread(target=control_thread_task, daemon=True)
    control_thread.start()

    print("[알림] 자율주행 및 장애물 회피 모듈이 구동되었습니다.")

    avoid_direction = None  # 'LEFT' 또는 'RIGHT'
    state_timer = 0

    try:
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                with error_lock: shared_speed = 0.0
                break

            # --------------------------------------------------------
            # A. 초음파 데이터 읽기 [Left, Front, Right]
            # --------------------------------------------------------
            left_d, front_d, right_d = ultrasonic.get_distances()

            # --------------------------------------------------------
            # B. 차선 인식 처리
            # --------------------------------------------------------
            height, width = frame.shape[:2]
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            blur = cv2.GaussianBlur(gray, (5, 5), 0)
            edges = cv2.Canny(blur, 50, 150)

            roi_vertices = [
                (0, height), 
                (int(width * 0.2), int(height * 0.45)),
                (int(width * 0.8), int(height * 0.45)), 
                (width, height)
            ]
            cropped_edges = lane_detection.region_of_interest(edges, np.array([roi_vertices], np.int32))
            lines = cv2.HoughLinesP(cropped_edges, rho=1, theta=np.pi/180, threshold=40, 
                                    minLineLength=20, maxLineGap=10)
            error, result_image, is_detected = lane_detection.calculate_steering(frame.copy(), lines)

            # --------------------------------------------------------
            # C. 주행 및 장애물 회피 상태 머신 (State Machine)
            # --------------------------------------------------------
            
            # [상태 1] 차선 주행 중 정면 장애물 탐지
            if current_state == STATE_LANE_FOLLOWING:
                if 0 < front_d <= FRONT_OBSTACLE_THRESH:
                    print(f"\n[장애물 감지] 정면 거리: {front_d:.1f}cm -> 정지 및 회피 공간 판단 중...")
                    current_state = STATE_CHECK_OBSTACLE
                    with error_lock:
                        shared_speed = 0.0
                        shared_angular_override = 0.0
                    state_timer = time.time()
                else:
                    # 정상 주행
                    with error_lock:
                        shared_error = error
                        shared_speed = BASE_SPEED if is_detected else 0.0
                        shared_angular_override = None

            # [상태 2] 정지 후 45도 양옆 센서 측정 및 회피 방향 결정
            elif current_state == STATE_CHECK_OBSTACLE:
                # 관성 정지를 위해 0.5초 대기
                if time.time() - state_timer > 0.5:
                    print(f"[센서 비교] 좌: {left_d:.1f}cm | 우: {right_d:.1f}cm")
                    
                    # -1.0은 측정 불가/장애물 없음(최대거리 초과)을 의미하므로 무한대(999)로 간주
                    eff_left = 999.0 if left_d < 0 else left_d
                    eff_right = 999.0 if right_d < 0 else right_d

                    # 양쪽 모두 회피 불가한 상황
                    if eff_left < SIDE_SAFE_CLEARANCE and eff_right < SIDE_SAFE_CLEARANCE:
                        print("[경고] 양쪽에 회피할 공간이 없습니다! 제자리 정지합니다.")
                        current_state = STATE_BLOCKED_STOP
                        with error_lock:
                            shared_speed = 0.0
                            shared_angular_override = 0.0
                    else:
                        # 더 넓은 쪽 선택
                        if eff_left >= eff_right:
                            avoid_direction = 'LEFT'
                            print("-> [회피 결정] 왼쪽 공간이 더 넓음: 왼쪽으로 회전")
                        else:
                            avoid_direction = 'RIGHT'
                            print("-> [회피 결정] 오른쪽 공간이 더 넓음: 오른쪽으로 회전")
                        
                        current_state = STATE_AVOID_TURN
                        state_timer = time.time()

            # [상태 3] 결정된 방향으로 회전
            elif current_state == STATE_AVOID_TURN:
                # 약 0.8초간 제자리 회전
                if time.time() - state_timer < 0.8:
                    turn_dir = TURN_ANGULAR_SPEED if avoid_direction == 'LEFT' else -TURN_ANGULAR_SPEED
                    with error_lock:
                        shared_speed = 0.0
                        shared_angular_override = turn_dir
                else:
                    current_state = STATE_AVOID_FORWARD
                    state_timer = time.time()

            # [상태 4] 장애물을 옆으로 비껴 지나감 (전진)
            elif current_state == STATE_AVOID_FORWARD:
                # 약 1.2초간 전진
                if time.time() - state_timer < 1.2:
                    with error_lock:
                        shared_speed = AVOID_SPEED
                        shared_angular_override = 0.0
                else:
                    current_state = STATE_RECOVERY_TURN
                    state_timer = time.time()

            # [상태 5] 원상 복귀를 위한 반대 방향 회전
            elif current_state == STATE_RECOVERY_TURN:
                # 원래 차선 방향으로 되돌아오기 위해 반대로 회전 (약 0.7초)
                if time.time() - state_timer < 0.7:
                    recovery_dir = -TURN_ANGULAR_SPEED if avoid_direction == 'LEFT' else TURN_ANGULAR_SPEED
                    with error_lock:
                        shared_speed = 0.0
                        shared_angular_override = recovery_dir
                else:
                    print("[복귀 완료] 다시 차선 추종 주행을 시작합니다.\n")
                    current_state = STATE_LANE_FOLLOWING
                    with error_lock:
                        shared_angular_override = None

            # [상태 6] 양쪽 다 막힌 상태 (정지 유지 및 재확인)
            elif current_state == STATE_BLOCKED_STOP:
                # 정면이나 옆 공간이 다시 확보되었는지 체크
                if front_d > FRONT_OBSTACLE_THRESH or front_d < 0:
                    print("[장애물 제거됨] 주행을 재개합니다.")
                    current_state = STATE_LANE_FOLLOWING
                else:
                    with error_lock:
                        shared_speed = 0.0
                        shared_angular_override = 0.0

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n[알림] 강제 종료 신호를 감지했습니다.")
    finally:
        is_running = False
        if 'control_thread' in locals() and control_thread.is_alive():
            control_thread.join(timeout=1.0)
        if ultrasonic:
            ultrasonic.stop()
        ugv_control.stop_ugv(ugv_serial)
        cap.release()
        cv2.destroyAllWindows()