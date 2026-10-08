import cv2
import time
import threading
import numpy as np

# ============================================================
# 모듈 임포트
# ============================================================
import lane_detection
import ugv_control
from ultrasonic_reader import UltrasonicReader


# ============================================================
# 기본 주행 설정
# ============================================================

# 정상 주행 속도 (m/s)
BASE_SPEED = 0.25

# 장애물 회피 중 서행 속도 (m/s)
AVOID_SPEED = 0.12

# 회전 각속도 (rad/s)
TURN_ANGULAR_SPEED = 0.5


# ============================================================
# 초음파 장애물 판단 기준
# ============================================================

# 정면 장애물 감지 시작 거리
FRONT_OBSTACLE_THRESH = 35.0

# 회피 중 이 거리 이하로 정면 장애물이 가까워지면
# 즉시 정지
EMERGENCY_FRONT_THRESH = 20.0

# 좌/우 센서가 이 거리보다 가까운 물체를 발견하면
# 해당 방향 회피를 위험하다고 판단
SIDE_DANGER_THRESH = 20.0

# 좌/우 방향을 회피 가능한 것으로 판단하기 위한 최소 거리
SIDE_SAFE_THRESH = 25.0

# 정면 장애물이 이 거리보다 멀어지면
# 장애물을 통과했다고 판단
OBSTACLE_CLEAR_THRESH = 45.0

# 장애물 통과 판정을 위해
# 몇 번 연속으로 안전한 값이 나와야 하는지
CLEAR_CONFIRM_COUNT = 3

# 장애물 회피를 최대 몇 초까지 진행할지
MAX_AVOID_FORWARD_TIME = 3.0


# ============================================================
# 회피 동작 시간
# ============================================================

# 장애물을 피하기 위해 최초로 방향을 틀어주는 시간
AVOID_TURN_TIME = 0.4

# 장애물을 지나간 후 원래 방향으로 복귀하는 시간
RECOVERY_TURN_TIME = 0.4


# ============================================================
# 장애물 감지 후 안정화 시간
# ============================================================

# 정면 장애물을 발견한 직후 바로 방향을 판단하지 않고
# 잠시 정지하여 센서값을 안정화
OBSTACLE_CHECK_DELAY = 0.5


# ============================================================
# 공유 제어 변수
# ============================================================

shared_error = 0
shared_speed = 0.0

# None:
#     PID 차선 조향 사용
#
# 숫자:
#     PID 대신 해당 각속도를 강제로 사용
shared_angular_override = None

error_lock = threading.Lock()

is_running = True


# ============================================================
# State Machine
# ============================================================

STATE_LANE_FOLLOWING = "LANE_FOLLOWING"
STATE_CHECK_OBSTACLE = "CHECK_OBSTACLE"
STATE_AVOID_TURN = "AVOID_TURN"
STATE_AVOID_FORWARD = "AVOID_FORWARD"
STATE_RECOVERY_TURN = "RECOVERY_TURN"
STATE_BLOCKED_STOP = "BLOCKED_STOP"

current_state = STATE_LANE_FOLLOWING


# ============================================================
# 하드웨어 객체
# ============================================================

ugv_serial = None
ultrasonic = None


# ============================================================
# 회피 방향
# ============================================================

avoid_direction = None

# "LEFT"
# "RIGHT"


# ============================================================
# 초음파 값 안전하게 처리
# ============================================================

def normalize_distance(distance):
    """
    초음파 센서에서 받은 거리값을 안전하게 처리한다.

    - 음수: 장애물 없음 / 측정 불가로 간주
    - 0: 측정 이상으로 간주
    - 정상값: 그대로 사용
    """

    try:
        distance = float(distance)
    except (TypeError, ValueError):
        return -1.0

    if distance <= 0:
        return -1.0

    return distance


# ============================================================
# 백그라운드 모터 제어
# ============================================================

def control_thread_task():

    global shared_error
    global shared_speed
    global shared_angular_override
    global is_running

    previous_error = 0
    last_time = time.time()

    print("[알림] 제어 백그라운드 스레드 시작")

    while is_running:

        current_time = time.time()

        dt = current_time - last_time

        if dt <= 0:
            dt = 0.001

        last_time = current_time


        # ----------------------------------------------------
        # UGV Odometry
        # ----------------------------------------------------

        if ugv_serial:

            ugv_control.read_odometry(
                ugv_serial,
                dt
            )


        # ----------------------------------------------------
        # 공유 변수 읽기
        # ----------------------------------------------------

        with error_lock:

            c_error = shared_error
            c_speed = shared_speed
            c_override = shared_angular_override


        # ----------------------------------------------------
        # 조향 계산
        # ----------------------------------------------------

        if c_override is not None:

            # 회피 동작 등에서 강제 조향
            angular_z = c_override

        else:

            # 평상시 PID 차선 조향
            angular_z = ugv_control.calculate_pid(
                c_error,
                previous_error,
                dt
            )

            previous_error = c_error


        # ----------------------------------------------------
        # 주행 명령
        # ----------------------------------------------------

        if ugv_serial:

            if c_speed != 0.0 or c_override is not None:

                send_angular = angular_z

            else:

                send_angular = 0.0


            ugv_control.send_driving_command(
                ugv_serial,
                c_speed,
                send_angular
            )


        time.sleep(0.02)


# ============================================================
# CSI Camera GStreamer Pipeline
# ============================================================

def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1280,
    capture_height=720,
    display_width=640,
    display_height=360,
    framerate=30,
    flip_method=0
):

    return (

        "nvarguscamerasrc sensor-id=%d ! "

        "video/x-raw(memory:NVMM), "
        "width=(int)%d, "
        "height=(int)%d, "
        "framerate=(fraction)%d/1 ! "

        "nvvidconv flip-method=%d ! "

        "video/x-raw, "
        "width=(int)%d, "
        "height=(int)%d, "
        "format=(string)BGRx ! "

        "videoconvert ! "

        "video/x-raw, "
        "format=(string)BGR ! "

        "appsink"

        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height
        )
    )


# ============================================================
# 메인 실행
# ============================================================

if __name__ == "__main__":

    # ========================================================
    # 1. UGV02 연결
    # ========================================================

    ugv_serial = ugv_control.init_ugv(
        port="/dev/ttyACM0",
        baudrate=115200
    )

    if not ugv_serial:

        print("[종료] UGV02 연결 실패")
        print("[확인] /dev/ttyACM0 포트를 확인하세요.")

        exit()


    # ========================================================
    # 2. Arduino 초음파 센서 연결
    # ========================================================

    ultrasonic = UltrasonicReader(
        port="/dev/ttyUSB0",
        baudrate=115200
    )

    ultrasonic_started = ultrasonic.start()

    if not ultrasonic_started:

        print(
            "[경고] 초음파 센서 포트 연결 실패!"
        )

        print(
            "[경고] /dev/ttyUSB0 포트를 확인하세요."
        )


    # ========================================================
    # 3. CSI 카메라 연결
    # ========================================================

    pipeline = gstreamer_pipeline(
        flip_method=0
    )

    cap = cv2.VideoCapture(
        pipeline,
        cv2.CAP_GSTREAMER
    )

    if not cap.isOpened():

        print("[에러] CSI 카메라 초기화 실패!")

        ugv_control.stop_ugv(
            ugv_serial
        )

        exit()


    # ========================================================
    # 4. 모터 제어 스레드 시작
    # ========================================================

    control_thread = threading.Thread(
        target=control_thread_task,
        daemon=True
    )

    control_thread.start()


    # ========================================================
    # 초기 상태
    # ========================================================

    print()
    print("============================================")
    print(" UGV02 자율주행 시스템 시작")
    print("============================================")
    print(" 카메라 : 차선 인식")
    print(" 중앙 US : 정면 장애물")
    print(" 좌측 US : 좌측 회피 공간")
    print(" 우측 US : 우측 회피 공간")
    print("============================================")
    print()


    # ========================================================
    # State 관련 변수
    # ========================================================

    global current_state

    avoid_direction = None

    state_timer = time.time()

    # 회피 전진 시작 시간
    avoid_forward_start_time = None

    # 장애물 통과 확인 카운터
    clear_confirm_counter = 0


    # ========================================================
    # 메인 루프
    # ========================================================

    try:

        while cap.isOpened():


            # =================================================
            # 카메라 프레임 읽기
            # =================================================

            ret, frame = cap.read()

            if not ret:

                print("[에러] 카메라 프레임 수신 실패")

                with error_lock:

                    shared_speed = 0.0
                    shared_angular_override = 0.0

                break


            # =================================================
            # A. 초음파 거리 읽기
            # =================================================

            if ultrasonic_started:

                left_d, front_d, right_d = (
                    ultrasonic.get_distances()
                )

                left_d = normalize_distance(left_d)
                front_d = normalize_distance(front_d)
                right_d = normalize_distance(right_d)

            else:

                # 초음파가 연결되지 않았다면
                # 안전을 위해 전부 장애물 없음이 아니라
                # 시스템을 정지시키는 것이 더 안전하다.

                left_d = -1.0
                front_d = -1.0
                right_d = -1.0


            # =================================================
            # B. 차선 인식
            # =================================================

            height, width = frame.shape[:2]


            # -----------------------------------------------
            # Gray
            # -----------------------------------------------

            gray = cv2.cvtColor(
                frame,
                cv2.COLOR_BGR2GRAY
            )


            # -----------------------------------------------
            # Gaussian Blur
            # -----------------------------------------------

            blur = cv2.GaussianBlur(
                gray,
                (5, 5),
                0
            )


            # -----------------------------------------------
            # Canny
            # -----------------------------------------------

            edges = cv2.Canny(
                blur,
                50,
                150
            )


            # -----------------------------------------------
            # ROI
            # -----------------------------------------------

            roi_vertices = np.array(
                [[
                    (0, height),

                    (
                        int(width * 0.2),
                        int(height * 0.45)
                    ),

                    (
                        int(width * 0.8),
                        int(height * 0.45)
                    ),

                    (width, height)
                ]],
                np.int32
            )


            cropped_edges = (
                lane_detection.region_of_interest(
                    edges,
                    roi_vertices
                )
            )


            # -----------------------------------------------
            # Hough Line
            # -----------------------------------------------

            lines = cv2.HoughLinesP(
                cropped_edges,
                rho=1,
                theta=np.pi / 180,
                threshold=40,
                minLineLength=20,
                maxLineGap=10
            )


            # -----------------------------------------------
            # 차선 오차 계산
            # -----------------------------------------------

            (
                error,
                result_image,
                is_detected
            ) = lane_detection.calculate_steering(
                frame.copy(),
                lines
            )


            # =================================================
            # C. STATE MACHINE
            # =================================================


            # =================================================
            # STATE 1
            # 정상 차선 주행
            # =================================================

            if current_state == STATE_LANE_FOLLOWING:

                # ---------------------------------------------
                # 정면 장애물 감지
                # ---------------------------------------------

                if (
                    front_d > 0
                    and front_d <= FRONT_OBSTACLE_THRESH
                ):

                    print()
                    print(
                        "============================================"
                    )

                    print(
                        f"[장애물 감지] "
                        f"정면 = {front_d:.1f} cm"
                    )

                    print(
                        "[동작] 정지 후 좌/우 회피 가능 여부 판단"
                    )

                    print(
                        "============================================"
                    )


                    current_state = (
                        STATE_CHECK_OBSTACLE
                    )

                    state_timer = time.time()

                    clear_confirm_counter = 0


                    # 즉시 정지

                    with error_lock:

                        shared_speed = 0.0

                        shared_angular_override = 0.0


                else:

                    # -----------------------------------------
                    # 정상 차선 주행
                    # -----------------------------------------

                    with error_lock:

                        shared_error = error


                        if is_detected:

                            shared_speed = BASE_SPEED

                        else:

                            # 차선이 보이지 않으면 정지
                            shared_speed = 0.0


                        # PID 사용
                        shared_angular_override = None


            # =================================================
            # STATE 2
            # 장애물 주변 공간 확인
            # =================================================

            elif current_state == STATE_CHECK_OBSTACLE:

                elapsed = (
                    time.time()
                    - state_timer
                )


                # ---------------------------------------------
                # 센서 안정화 대기
                # ---------------------------------------------

                if elapsed >= OBSTACLE_CHECK_DELAY:

                    print()
                    print(
                        "[공간 분석]"
                    )

                    print(
                        f"LEFT  = {left_d:.1f} cm"
                    )

                    print(
                        f"FRONT = {front_d:.1f} cm"
                    )

                    print(
                        f"RIGHT = {right_d:.1f} cm"
                    )


                    # -----------------------------------------
                    # 좌측 회피 가능 여부
                    # -----------------------------------------

                    left_available = (
                        left_d < 0
                        or left_d >= SIDE_SAFE_THRESH
                    )


                    # -----------------------------------------
                    # 우측 회피 가능 여부
                    # -----------------------------------------

                    right_available = (
                        right_d < 0
                        or right_d >= SIDE_SAFE_THRESH
                    )


                    print(
                        f"[판단] "
                        f"LEFT={'가능' if left_available else '불가'} / "
                        f"RIGHT={'가능' if right_available else '불가'}"
                    )


                    # -----------------------------------------
                    # 둘 다 불가능
                    # -----------------------------------------

                    if (
                        not left_available
                        and not right_available
                    ):

                        print(
                            "[정지] 좌/우 모두 회피 공간 부족"
                        )

                        print(
                            "[상태] BLOCKED_STOP"
                        )

                        current_state = (
                            STATE_BLOCKED_STOP
                        )

                        with error_lock:

                            shared_speed = 0.0

                            shared_angular_override = 0.0


                    # -----------------------------------------
                    # 좌측만 가능
                    # -----------------------------------------

                    elif left_available and not right_available:

                        avoid_direction = "LEFT"

                        print(
                            "[회피 결정] LEFT"
                        )

                        current_state = (
                            STATE_AVOID_TURN
                        )

                        state_timer = time.time()


                    # -----------------------------------------
                    # 우측만 가능
                    # -----------------------------------------

                    elif right_available and not left_available:

                        avoid_direction = "RIGHT"

                        print(
                            "[회피 결정] RIGHT"
                        )

                        current_state = (
                            STATE_AVOID_TURN
                        )

                        state_timer = time.time()


                    # -----------------------------------------
                    # 양쪽 모두 가능
                    # -----------------------------------------

                    else:

                        # 둘 다 안전하면
                        # 더 넓은 방향을 선택

                        if left_d < 0 and right_d >= 0:

                            avoid_direction = "LEFT"

                        elif right_d < 0 and left_d >= 0:

                            avoid_direction = "RIGHT"

                        elif (
                            left_d >= right_d
                        ):

                            avoid_direction = "LEFT"

                        else:

                            avoid_direction = "RIGHT"


                        print(
                            f"[회피 결정] "
                            f"{avoid_direction}"
                        )


                        current_state = (
                            STATE_AVOID_TURN
                        )

                        state_timer = time.time()


            # =================================================
            # STATE 3
            # 회피 방향으로 회전
            # =================================================

            elif current_state == STATE_AVOID_TURN:

                elapsed = (
                    time.time()
                    - state_timer
                )


                # ---------------------------------------------
                # 회전 중 정면 위험
                # ---------------------------------------------

                if (
                    front_d > 0
                    and front_d <= EMERGENCY_FRONT_THRESH
                ):

                    print(
                        f"[긴급 정지] "
                        f"회전 중 정면 {front_d:.1f} cm"
                    )

                    current_state = (
                        STATE_BLOCKED_STOP
                    )

                    with error_lock:

                        shared_speed = 0.0
                        shared_angular_override = 0.0


                # ---------------------------------------------
                # LEFT 회피
                # ---------------------------------------------

                elif avoid_direction == "LEFT":

                    # 좌측 공간이 갑자기 좁아짐
                    if (
                        left_d > 0
                        and left_d <= SIDE_DANGER_THRESH
                    ):

                        print(
                            f"[긴급 정지] "
                            f"좌측 공간 {left_d:.1f} cm"
                        )

                        current_state = (
                            STATE_BLOCKED_STOP
                        )

                        with error_lock:

                            shared_speed = 0.0
                            shared_angular_override = 0.0


                    elif elapsed < AVOID_TURN_TIME:

                        with error_lock:

                            shared_speed = 0.0

                            shared_angular_override = (
                                TURN_ANGULAR_SPEED
                            )


                    else:

                        print(
                            "[회전 완료] LEFT 방향 통과 시작"
                        )

                        current_state = (
                            STATE_AVOID_FORWARD
                        )

                        avoid_forward_start_time = (
                            time.time()
                        )

                        clear_confirm_counter = 0


                # ---------------------------------------------
                # RIGHT 회피
                # ---------------------------------------------

                else:

                    # 우측 공간이 갑자기 좁아짐
                    if (
                        right_d > 0
                        and right_d <= SIDE_DANGER_THRESH
                    ):

                        print(
                            f"[긴급 정지] "
                            f"우측 공간 {right_d:.1f} cm"
                        )

                        current_state = (
                            STATE_BLOCKED_STOP
                        )

                        with error_lock:

                            shared_speed = 0.0
                            shared_angular_override = 0.0


                    elif elapsed < AVOID_TURN_TIME:

                        with error_lock:

                            shared_speed = 0.0

                            shared_angular_override = (
                                -TURN_ANGULAR_SPEED
                            )


                    else:

                        print(
                            "[회전 완료] RIGHT 방향 통과 시작"
                        )

                        current_state = (
                            STATE_AVOID_FORWARD
                        )

                        avoid_forward_start_time = (
                            time.time()
                        )

                        clear_confirm_counter = 0


            # =================================================
            # STATE 4
            # 장애물 옆으로 서행
            # =================================================

            elif current_state == STATE_AVOID_FORWARD:

                elapsed = (
                    time.time()
                    - avoid_forward_start_time
                )


                # ---------------------------------------------
                # 회피 중 정면 센서 확인
                # ---------------------------------------------

                front_danger = (
                    front_d > 0
                    and front_d <= EMERGENCY_FRONT_THRESH
                )


                # ---------------------------------------------
                # 회피 방향 측면 센서 확인
                # ---------------------------------------------

                left_danger = (
                    left_d > 0
                    and left_d <= SIDE_DANGER_THRESH
                )

                right_danger = (
                    right_d > 0
                    and right_d <= SIDE_DANGER_THRESH
                )


                # ---------------------------------------------
                # 1. 정면이 너무 가까움
                # ---------------------------------------------

                if front_danger:

                    print(
                        f"[긴급 정지] "
                        f"회피 중 정면 {front_d:.1f} cm"
                    )

                    current_state = (
                        STATE_BLOCKED_STOP
                    )

                    with error_lock:

                        shared_speed = 0.0
                        shared_angular_override = 0.0


                # ---------------------------------------------
                # 2. LEFT 회피 중 좌측 위험
                # ---------------------------------------------

                elif (
                    avoid_direction == "LEFT"
                    and left_danger
                ):

                    print(
                        f"[긴급 정지] "
                        f"회피 중 좌측 {left_d:.1f} cm"
                    )

                    current_state = (
                        STATE_BLOCKED_STOP
                    )

                    with error_lock:

                        shared_speed = 0.0
                        shared_angular_override = 0.0


                # ---------------------------------------------
                # 3. RIGHT 회피 중 우측 위험
                # ---------------------------------------------

                elif (
                    avoid_direction == "RIGHT"
                    and right_danger
                ):

                    print(
                        f"[긴급 정지] "
                        f"회피 중 우측 {right_d:.1f} cm"
                    )

                    current_state = (
                        STATE_BLOCKED_STOP
                    )

                    with error_lock:

                        shared_speed = 0.0
                        shared_angular_override = 0.0


                # ---------------------------------------------
                # 4. 장애물이 충분히 멀어짐
                # ---------------------------------------------

                elif (
                    front_d < 0
                    or front_d >= OBSTACLE_CLEAR_THRESH
                ):

                    clear_confirm_counter += 1

                    print(
                        f"[통과 확인] "
                        f"{clear_confirm_counter}/"
                        f"{CLEAR_CONFIRM_COUNT}"
                    )


                    if (
                        clear_confirm_counter
                        >= CLEAR_CONFIRM_COUNT
                    ):

                        print(
                            "[장애물 통과 확인]"
                        )

                        print(
                            "[동작] 차선 복귀"
                        )

                        current_state = (
                            STATE_RECOVERY_TURN
                        )

                        state_timer = time.time()

                        clear_confirm_counter = 0


                else:

                    # 다시 장애물이 가까워지면
                    # 통과 확인 카운터 초기화

                    clear_confirm_counter = 0


                    # -----------------------------------------
                    # 최대 회피 시간 확인
                    # -----------------------------------------

                    if elapsed >= MAX_AVOID_FORWARD_TIME:

                        print(
                            "[경고] 최대 회피 시간이 초과되었습니다."
                        )

                        print(
                            "[안전] 강제 정지 후 재판단"
                        )

                        current_state = (
                            STATE_BLOCKED_STOP
                        )

                        with error_lock:

                            shared_speed = 0.0
                            shared_angular_override = 0.0


                    else:

                        # -------------------------------------
                        # 정상 회피 전진
                        # -------------------------------------

                        with error_lock:

                            shared_speed = AVOID_SPEED

                            # 회피 중에는 PID 대신
                            # 직진
                            shared_angular_override = 0.0


            # =================================================
            # STATE 5
            # 차선 방향으로 복귀
            # =================================================

            elif current_state == STATE_RECOVERY_TURN:

                elapsed = (
                    time.time()
                    - state_timer
                )


                # ---------------------------------------------
                # 복귀 중 정면 장애물 재확인
                # ---------------------------------------------

                if (
                    front_d > 0
                    and front_d <= EMERGENCY_FRONT_THRESH
                ):

                    print(
                        f"[복귀 중 장애물 재감지] "
                        f"{front_d:.1f} cm"
                    )

                    current_state = (
                        STATE_BLOCKED_STOP
                    )

                    with error_lock:

                        shared_speed = 0.0
                        shared_angular_override = 0.0


                # ---------------------------------------------
                # LEFT에서 회피했다면
                # 반대 방향으로 복귀
                # ---------------------------------------------

                elif avoid_direction == "LEFT":

                    if elapsed < RECOVERY_TURN_TIME:

                        with error_lock:

                            shared_speed = 0.0

                            shared_angular_override = (
                                -TURN_ANGULAR_SPEED
                            )

                    else:

                        print(
                            "[복귀 완료] PID 차선 추종 재개"
                        )

                        current_state = (
                            STATE_LANE_FOLLOWING
                        )

                        avoid_direction = None

                        with error_lock:

                            shared_angular_override = None


                # ---------------------------------------------
                # RIGHT에서 회피했다면
                # 반대 방향으로 복귀
                # ---------------------------------------------

                else:

                    if elapsed < RECOVERY_TURN_TIME:

                        with error_lock:

                            shared_speed = 0.0

                            shared_angular_override = (
                                TURN_ANGULAR_SPEED
                            )

                    else:

                        print(
                            "[복귀 완료] PID 차선 추종 재개"
                        )

                        current_state = (
                            STATE_LANE_FOLLOWING
                        )

                        avoid_direction = None

                        with error_lock:

                            shared_angular_override = None


            # =================================================
            # STATE 6
            # 회피 불가능 → 정지
            # =================================================

            elif current_state == STATE_BLOCKED_STOP:

                # ---------------------------------------------
                # 무조건 정지
                # ---------------------------------------------

                with error_lock:

                    shared_speed = 0.0

                    shared_angular_override = 0.0


                # ---------------------------------------------
                # 장애물이 없어졌는지 확인
                # ---------------------------------------------

                obstacle_cleared = (
                    front_d < 0
                    or front_d > OBSTACLE_CLEAR_THRESH
                )


                if obstacle_cleared:

                    print(
                        "[장애물 제거 확인]"
                    )

                    print(
                        "[상태] 정상 차선 주행 재개"
                    )

                    current_state = (
                        STATE_LANE_FOLLOWING
                    )

                    avoid_direction = None

                    with error_lock:

                        shared_angular_override = None


            # =================================================
            # D. 화면 표시
            # =================================================

            display = frame.copy()


            # -----------------------------------------------
            # State
            # -----------------------------------------------

            cv2.putText(
                display,
                f"STATE: {current_state}",
                (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2
            )


            # -----------------------------------------------
            # 초음파 값
            # -----------------------------------------------

            cv2.putText(
                display,
                (
                    f"L:{left_d:.1f} "
                    f"F:{front_d:.1f} "
                    f"R:{right_d:.1f}"
                ),
                (10, 50),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 255),
                2
            )


            # -----------------------------------------------
            # 회피 방향
            # -----------------------------------------------

            if avoid_direction is not None:

                cv2.putText(
                    display,
                    f"AVOID: {avoid_direction}",
                    (10, 75),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 255),
                    2
                )


            # -----------------------------------------------
            # 차선 인식 여부
            # -----------------------------------------------

            cv2.putText(
                display,
                f"LANE: {'OK' if is_detected else 'LOST'}",
                (10, 100),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                2
            )


            # -----------------------------------------------
            # 화면 출력
            # -----------------------------------------------

            cv2.imshow(
                "UGV02 Autonomous Driving",
                display
            )


            # ESC 종료
            key = cv2.waitKey(1) & 0xFF

            if key == 27:

                print(
                    "[알림] ESC 입력 -> 종료"
                )

                break


            # 메인 루프 과도한 CPU 사용 방지
            time.sleep(0.01)


    # ========================================================
    # Ctrl + C
    # ========================================================

    except KeyboardInterrupt:

        print(
            "\n[알림] Ctrl+C 감지"
        )


    # ========================================================
    # 종료 처리
    # ========================================================

    finally:

        print(
            "[알림] 시스템 종료 중..."
        )


        # ---------------------------------------------
        # 가장 먼저 모터 정지
        # ---------------------------------------------

        with error_lock:

            shared_speed = 0.0

            shared_angular_override = 0.0


        if ugv_serial:

            ugv_control.stop_ugv(
                ugv_serial
            )


        # ---------------------------------------------
        # 제어 스레드 종료
        # ---------------------------------------------

        is_running = False

        if (
            "control_thread" in locals()
            and control_thread.is_alive()
        ):

            control_thread.join(
                timeout=1.0
            )


        # ---------------------------------------------
        # 초음파 종료
        # ---------------------------------------------

        if ultrasonic:

            ultrasonic.stop()


        # ---------------------------------------------
        # 카메라 종료
        # ---------------------------------------------

        cap.release()

        cv2.destroyAllWindows()


        print(
            "[알림] 프로그램 종료 완료"
        )