import cv2
import time
import threading
import numpy as np

# 모듈 임포트
import lane_detection
import ugv_control
from ultrasonic_reader import UltrasonicReader

# ==========================================
# 기본 설정 및 차선/차체 데이터 (단위: cm, m/s)
# ==========================================
LANE_WIDTH_CM = 70.0          # 차선 폭 (cm)
VEHICLE_WIDTH_CM = 21.0        # UGV02 차체 폭 (cm)
MIN_LANE_CLEARANCE_CM = 30.0   # 차선 내 최소 회피 필요 공간 (차체 21cm + 여유 9cm)

BASE_SPEED = 0.25              # 기본 주행 속도 (m/s)
AVOID_SPEED = 0.18             # 차선 내 회피 서행 속도 (m/s)
TURN_ANGULAR_SPEED = 0.6       # 좁은 차선 내 미세 회전 각속도 (rad/s)

shared_error = 0            
shared_speed = 0.0          
shared_angular_override = None # None이 아니면 PID 조향 대신 이 각속도를 사용

error_lock = threading.Lock()
is_running = True

# 장애물 회피 상태 정의 (State Machine)
STATE_LANE_FOLLOWING = "LANE_FOLLOWING" # 정상 차선 주행
STATE_CHECK_OBSTACLE = "CHECK_OBSTACLE" # 장애물 감지 후 정지 & 판단
STATE_AVOID_TURN    = "AVOID_TURN"    # 차선 내 비껴서기 회전
STATE_AVOID_FORWARD = "AVOID_FORWARD" # 차선 내 장애물 옆 지나치기
STATE_RECOVERY_TURN = "RECOVERY_TURN" # 차선 중앙 복귀 회전
STATE_BLOCKED_STOP  = "BLOCKED_STOP"  # 차선 내 공간 부족 (탈선 방지 정지)

current_state = STATE_LANE_FOLLOWING

# 거리 임계값 설정 (단위: cm)
FRONT_OBSTACLE_THRESH = 35.0  # 정면 장애물 감지 거리

# 하드웨어 객체
ugv_serial = None
ultrasonic = None


# ==========================================
# [보조 함수] 비전 기반 차선-장애물 간 실제 공간(cm) 계산
# ==========================================
def estimate_lane_space(frame, lines, lane_width_cm=65.0):
    """
    카메라 영상에서 좌/우 차선 X좌표를 추출한 뒤,
    전방 장애물(화면 중앙) 기준 좌/우 차선까지의 실제 거리(cm)를 계산합니다.
    """
    if lines is None:
        return 0.0, 0.0, False

    height, width = frame.shape[:2]
    left_x_list = []
    right_x_list = []

    for line in lines:
        for x1, y1, x2, y2 in line:
            if x2 == x1:
                continue
            slope = (y2 - y1) / (x2 - x1)
            # 기울기에 따라 좌/우 차선 구분 (영상 하단 X 좌표 추정)
            if slope < -0.3:  # 좌측 차선
                x_bottom = int(x1 + (height - y1) / slope)
                left_x_list.append(x_bottom)
            elif slope > 0.3: # 우측 차선
                x_bottom = int(x1 + (height - y1) / slope)
                right_x_list.append(x_bottom)

    if not left_x_list or not right_x_list:
        return 0.0, 0.0, False

    x_left = float(np.median(left_x_list))
    x_right = float(np.median(right_x_list))

    pixel_lane_width = x_right - x_left
    if pixel_lane_width <= 50:  # 비정상 데이터 검출 방지
        return 0.0, 0.0, False

    # 픽셀 당 센치미터 비율 (65cm / 화면 내 차선 픽셀 폭)
    cm_per_pixel = lane_width_cm / pixel_lane_width

    # 전방 장애물 X 위치 (카메라 중앙 화면 x_center = width / 2.0 기준)
    obstacle_x = width / 2.0

    # 장애물 기준 좌/우 차선까지의 실제 거리(cm)
    s_left = (obstacle_x - x_left) * cm_per_pixel
    s_right = (x_right - obstacle_x) * cm_per_pixel

    return max(0.0, s_left), max(0.0, s_right), True


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
    
    # 1. UGV 하체 연결
    ugv_serial = ugv_control.init_ugv(port='/dev/ttyACM0', baudrate=115200)
    if not ugv_serial:
        print("[종료] UGV02 연결 실패. 포트를 확인하세요.")
        exit()

    # 2. 아두이노 초음파 센서 모듈 연결
    ultrasonic = UltrasonicReader(port='/dev/ttyUSB0', baudrate=115200)
    if not ultrasonic.start():
        print("[경고] 초음파 센서 포트 연결 실패! 아두이노 포트를 점검하세요.")

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

    print("[알림] 차선 내 자율주행 및 이탈 방지 장애물 회피 시스템 구동 시작.")

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
                    print(f"\n[장애물 감지] 정면 거리: {front_d:.1f}cm -> 정지 및 차선 내 여유 공간 측정 중...")
                    current_state = STATE_CHECK_OBSTACLE
                    with error_lock:
                        shared_speed = 0.0
                        shared_angular_override = 0.0
                    state_timer = time.time()
                else:
                    # 정상 차선 추종 주행
                    with error_lock:
                        shared_error = error
                        shared_speed = BASE_SPEED if is_detected else 0.0
                        shared_angular_override = None

            # [상태 2] 비전(차선 거리) + 초음파 복합 판단 (차선 이탈 검사)
            elif current_state == STATE_CHECK_OBSTACLE:
                if time.time() - state_timer > 0.5: # 정지 후 센서/카메라 화면 안정화 대기
                    
                    # 1. 비전 기반: 장애물과 좌/우 차선 사이의 실제 여유 공간(cm) 계산
                    s_left_lane, s_right_lane, lane_ok = estimate_lane_space(frame, lines, LANE_WIDTH_CM)

                    # 2. 초음파 기반 물리적 거리 값 정리 (-1은 장애물 없음/최대거리)
                    eff_left_us = 999.0 if left_d < 0 else left_d
                    eff_right_us = 999.0 if right_d < 0 else right_d

                    # 3. 차선 내 거리와 초음파 거리 중 더 위험한(좁은) 값을 최종 회피 공간으로 간주
                    if lane_ok:
                        final_left_space = min(s_left_lane, eff_left_us)
                        final_right_space = min(s_right_lane, eff_right_us)
                        print(f"[공간 분석] 차선내 공간 - 좌: {s_left_lane:.1f}cm, 우: {s_right_lane:.1f}cm | 초음파 센서 - 좌: {eff_left_us:.1f}cm, 우: {eff_right_us:.1f}cm")
                        print(f"-> 최종 회피 공간: 좌 {final_left_space:.1f}cm / 우 {final_right_space:.1f}cm (필요: {MIN_LANE_CLEARANCE_CM}cm)")
                    else:
                        final_left_space = eff_left_us
                        final_right_space = eff_right_us
                        print("[경고] 차선 미인식 -> 센서 거리만으로 보수적 판단 진행")

                    # 4. 차선 내 회피 가능 여부 판정 (최소 30cm 확보 여부)
                    if final_left_space < MIN_LANE_CLEARANCE_CM and final_right_space < MIN_LANE_CLEARANCE_CM:
                        print(f"[정지 명령] 양쪽 모두 차선 내 공간 부족 (최소 {MIN_LANE_CLEARANCE_CM}cm 필요). 차선 이탈 방지를 위해 멈춥니다.")
                        current_state = STATE_BLOCKED_STOP
                        with error_lock:
                            shared_speed = 0.0
                            shared_angular_override = 0.0
                    else:
                        # 더 넓고 안전한 쪽으로 차선 내 회피 선택
                        if final_left_space >= final_right_space and final_left_space >= MIN_LANE_CLEARANCE_CM:
                            avoid_direction = 'LEFT'
                            print("-> [회피 결정] 좌측 차선 내 공간 확보됨: 좌측으로 미세 회전")
                        else:
                            avoid_direction = 'RIGHT'
                            print("-> [회피 결정] 우측 차선 내 공간 확보됨: 우측으로 미세 회전")
                        
                        current_state = STATE_AVOID_TURN
                        state_timer = time.time()

            # [상태 3] 차선 안에서 비껴서기 위한 미세 회전
            elif current_state == STATE_AVOID_TURN:
                # 65cm 차선을 넘어가지 않도록 짧은 시간(0.4초) 및 낮은 각속도 적용
                if time.time() - state_timer < 0.4:
                    turn_dir = TURN_ANGULAR_SPEED if avoid_direction == 'LEFT' else -TURN_ANGULAR_SPEED
                    with error_lock:
                        shared_speed = 0.0
                        shared_angular_override = turn_dir
                else:
                    current_state = STATE_AVOID_FORWARD
                    state_timer = time.time()

            # [상태 4] 장애물 옆을 차선 안에서 통과 (짧은 서행 전진)
            elif current_state == STATE_AVOID_FORWARD:
                # 약 0.8초간 짧게 전진하여 장애물 옆 통과
                if time.time() - state_timer < 0.8:
                    with error_lock:
                        shared_speed = AVOID_SPEED
                        shared_angular_override = 0.0
                else:
                    current_state = STATE_RECOVERY_TURN
                    state_timer = time.time()

            # [상태 5] 차선 중앙으로 복귀 회전
            elif current_state == STATE_RECOVERY_TURN:
                # 반대 방향으로 미세 회전하여 차선 정렬 (0.4초)
                if time.time() - state_timer < 0.4:
                    recovery_dir = -TURN_ANGULAR_SPEED if avoid_direction == 'LEFT' else TURN_ANGULAR_SPEED
                    with error_lock:
                        shared_speed = 0.0
                        shared_angular_override = recovery_dir
                else:
                    print("[복귀 완료] 차선 중앙 정렬 완료. PID 차선 추종을 재개합니다.\n")
                    current_state = STATE_LANE_FOLLOWING
                    with error_lock:
                        shared_angular_override = None

            # [상태 6] 공간 부족으로 정지 유지 및 해제 감시
            elif current_state == STATE_BLOCKED_STOP:
                # 정면 장애물 제거 여부 재확인
                if front_d > FRONT_OBSTACLE_THRESH or front_d < 0:
                    print("[장애물 제거됨] 차선 내 주행을 재개합니다.")
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