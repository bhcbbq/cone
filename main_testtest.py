import cv2
import time
import threading
import numpy as np

import lane_detection
import ugv_control
from ultrasonic_reader import UltrasonicReader


# ============================================================
# 기본 주행 설정
# ============================================================

BASE_SPEED = 0.25
AVOID_SPEED = 0.12
TURN_ANGULAR_SPEED = 0.5


# ============================================================
# 초음파 장애물 판단 기준 (cm)
# ============================================================

FRONT_OBSTACLE_THRESH = 35.0
EMERGENCY_FRONT_THRESH = 20.0

# 좌/우 센서가 이 거리 이상이면 회피 가능하다고 판단
SIDE_SAFE_THRESH = 25.0

# 회피 중 이 거리 이하가 되면 즉시 정지
SIDE_DANGER_THRESH = 20.0

# 정면 장애물이 이 거리보다 멀어지면
# 장애물을 통과했다고 판단
OBSTACLE_CLEAR_THRESH = 45.0

# 장애물 통과를 몇 번 연속 확인할지
CLEAR_CONFIRM_COUNT = 3

# 한 번의 회피 전진을 허용하는 최대 시간
MAX_AVOID_FORWARD_TIME = 3.0


# ============================================================
# 회피 동작 시간
# 실제 UGV 테스트 후 조정 필요
# ============================================================

AVOID_TURN_TIME = 0.4
RECOVERY_TURN_TIME = 0.4


# ============================================================
# 장애물 감지 직후 센서 안정화 시간
# ============================================================

OBSTACLE_CHECK_DELAY = 0.5


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
# 공유 제어 변수
# ============================================================

shared_error = 0
shared_speed = 0.0
shared_angular_override = None

error_lock = threading.Lock()
is_running = True


# ============================================================
# 하드웨어 객체
# ============================================================

ugv_serial = None
ultrasonic = None


# ============================================================
# 거리값 처리
# ============================================================

def normalize_distance(distance):
    """
    초음파 거리값을 안전하게 float으로 변환한다.

    반환값:
        정상 거리 -> 양수
        측정 불가/잘못된 값 -> -1
    """

    try:
        distance = float(distance)
    except (TypeError, ValueError):
        return -1.0

    if distance <= 0:
        return -1.0

    return distance


# ============================================================
# 모터 제어 백그라운드 스레드
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

            # 정상 주행에서는 PID 차선 조향
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
    flip_method=0,
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
            display_height,
        )
    )


# ============================================================
# 메인
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

        raise SystemExit


    # ========================================================
    # 2. Arduino 초음파 센서 연결
    # ========================================================

    ultrasonic = UltrasonicReader(
        port="/dev/ttyUSB0",
        baudrate=115200
    )

    sensors_ready = ultrasonic.start()

    if not sensors_ready:

        print("[경고] 초음파 센서 연결 실패!")
        print("[확인] /dev/ttyUSB0 포트를 확인하세요.")
        print("[안전] 초음파가 없으므로 주행하지 않습니다.")


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

        if ultrasonic:
            ultrasonic.stop()

        ugv_control.stop_ugv(
            ugv_serial
        )

        raise SystemExit


    # ========================================================
    # 4. 모터 제어 스레드 시작
    # ========================================================

    control_thread = threading.Thread(
        target=control_thread_task,
        daemon=True
    )

    control_thread.start()


    # ========================================================
    # 초기 상태 변수
    # ========================================================

    avoid_direction = None

    state_timer = time.time()

    avoid_forward_start_time = None

    clear_confirm_counter = 0


    print()
    print("============================================")
    print(" UGV02 자율주행 시스템 시작")
    print("============================================")
    print(" Camera : 차선 인식 / PID")
    print(" Left US : 좌측 장애물")
    print(" Front US: 정면 장애물")
    print(" Right US: 우측 장애물")
    print("============================================")


    # ========================================================
    # 메인 루프
    # ========================================================

    try:

        while cap.isOpened():

            # =================================================
            # A. 카메라 프레임
            # =================================================

            ret, frame = cap.read()

            if not ret:

                print(
                    "[에러] 카메라 프레임 수신 실패"
                )

                with error_lock:

                    shared_speed = 0.0

                    shared_angular_override = 0.0

                break


            # =================================================
            # B. 초음파 거리
            # =================================================

            if sensors_ready:

                left_d, front_d, right_d = (
                    ultrasonic.get_distances()
                )

                left_d = normalize_distance(
                    left_d
                )

                front_d = normalize_distance(
                    front_d
                )

                right_d = normalize_distance(
                    right_d
                )

            else:

                # 초음파가 연결되지 않았다면
                # 안전을 위해 정지

                left_d = -1.0
                front_d = -1.0
                right_d = -1.0

                with error_lock:

                    shared_speed = 0.0

                    shared_angular_override = 0.0


            # =================================================
            # C. 차선 인식
            # =================================================

            height, width = frame.shape[:2]


            # -------------------------------------------------
            # Gray
            # -------------------------------------------------

            gray = cv2.cvtColor(
                frame,
                cv2.COLOR_BGR2GRAY
            )


            # -------------------------------------------------
            # Blur
            # -------------------------------------------------

            blur = cv2.GaussianBlur(
                gray,
                (5, 5),
                0
            )


            # -------------------------------------------------
            # Canny
            # -------------------------------------------------

            edges = cv2.Canny(
                blur,
                50,
                150
            )


            # -------------------------------------------------
            # ROI
            # -------------------------------------------------

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
                dtype=np.int32
            )


            cropped_edges = (
                lane_detection.region_of_interest(
                    edges,
                    roi_vertices
                )
            )


            # -------------------------------------------------
            # Hough Line
            # -------------------------------------------------

            lines = cv2.HoughLinesP(
                cropped_edges,
                rho=1,
                theta=np.pi / 180,
                threshold=40,
                minLineLength=20,
                maxLineGap=10
            )


            # -------------------------------------------------
            # 차선 오차 계산
            # -------------------------------------------------

            (
                error,
                result_image,
                is_detected
            ) = lane_detection.calculate_steering(
                frame.copy(),
                lines
            )


            # =================================================
            # D. 초음파 센서가 없으면 안전 정지
            # =================================================

            if not sensors_ready:

                current_state = STATE_BLOCKED_STOP

                avoid_direction = None

                with error_lock:

                    shared_speed = 0.0

                    shared_angular_override = 0.0


            # =================================================
            # STATE 1
            # 정상 차선 주행
            # =================================================

            elif current_state == STATE_LANE_FOLLOWING:

                # ------------------------------------------------
                # 정면 장애물 감지
                # ------------------------------------------------

                if (
                    front_d > 0
                    and front_d <= FRONT_OBSTACLE_THRESH
                ):

                    print()
                    print(
                        f"[장애물 감지] "
                        f"Front = {front_d:.1f} cm"
                    )

                    print(
                        "[동작] 정지 후 좌/우 공간 확인"
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

                    # ------------------------------------------------
                    # 정상 차선 주행
                    # ------------------------------------------------

                    with error_lock:

                        shared_error = error


                        if is_detected:

                            shared_speed = BASE_SPEED

                        else:

                            # 차선이 안 보이면 정지
                            shared_speed = 0.0


                        # PID 사용

                        shared_angular_override = None


            # =================================================
            # STATE 2
            # 장애물 좌/우 공간 확인
            # =================================================

            elif current_state == STATE_CHECK_OBSTACLE:

                elapsed = (
                    time.time()
                    - state_timer
                )


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


                    # ------------------------------------------------
                    # 좌측 회피 가능 여부
                    # ------------------------------------------------

                    left_available = (
                        left_d < 0
                        or left_d >= SIDE_SAFE_THRESH
                    )


                    # ------------------------------------------------
                    # 우측 회피 가능 여부
                    # ------------------------------------------------

                    right_available = (
                        right_d < 0
                        or right_d >= SIDE_SAFE_THRESH
                    )


                    print(
                        f"[판단] "
                        f"LEFT="
                        f"{'가능' if left_available else '불가'}"
                        f" / "
                        f"RIGHT="
                        f"{'가능' if right_available else '불가'}"
                    )


                    # ------------------------------------------------
                    # 둘 다 불가능
                    # ------------------------------------------------

                    if (
                        not left_available
                        and not right_available
                    ):

                        print(
                            "[정지] 양쪽 모두 회피 공간 부족"
                        )


                        current_state = (
                            STATE_BLOCKED_STOP
                        )


                        with error_lock:

                            shared_speed = 0.0

                            shared_angular_override = 0.0


                    # ------------------------------------------------
                    # 좌측만 가능
                    # ------------------------------------------------

                    elif (
                        left_available
                        and not right_available
                    ):

                        avoid_direction = "LEFT"


                        print(
                            "[회피 결정] LEFT"
                        )


                        current_state = (
                            STATE_AVOID_TURN
                        )

                        state_timer = time.time()


                    # ------------------------------------------------
                    # 우측만 가능
                    # ------------------------------------------------

                    elif (
                        right_available
                        and not left_available
                    ):

                        avoid_direction = "RIGHT"


                        print(
                            "[회피 결정] RIGHT"
                        )


                        current_state = (
                            STATE_AVOID_TURN
                        )

                        state_timer = time.time()


                    # ------------------------------------------------
                    # 양쪽 모두 가능
                    # ------------------------------------------------

                    else:

                        # 둘 다 가능하면
                        # 더 넓은 방향 선택

                        if (
                            left_d < 0
                            and right_d >= 0
                        ):

                            avoid_direction = "LEFT"

                        elif (
                            right_d < 0
                            and left_d >= 0
                        ):

                            avoid_direction = "RIGHT"

                        elif left_d >= right_d:

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


                # ------------------------------------------------
                # 정면이 너무 가까워지면 즉시 정지
                # ------------------------------------------------

                if (
                    front_d > 0
                    and front_d <= EMERGENCY_FRONT_THRESH
                ):

                    print(
                        f"[긴급 정지] "
                        f"회전 중 Front = "
                        f"{front_d:.1f} cm"
                    )


                    current_state = (
                        STATE_BLOCKED_STOP
                    )


                    with error_lock:

                        shared_speed = 0.0

                        shared_angular_override = 0.0


                # =================================================
                # LEFT 회피
                # =================================================

                elif avoid_direction == "LEFT":

                    # 좌측이 너무 가까워짐

                    if (
                        left_d > 0
                        and left_d <= SIDE_DANGER_THRESH
                    ):

                        print(
                            f"[긴급 정지] "
                            f"회전 중 Left = "
                            f"{left_d:.1f} cm"
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
                            "[회전 완료] "
                            "LEFT 방향 통과 시작"
                        )


                        current_state = (
                            STATE_AVOID_FORWARD
                        )

                        avoid_forward_start_time = (
                            time.time()
                        )

                        clear_confirm_counter = 0


                # =================================================
                # RIGHT 회피
                # =================================================

                else:

                    # 우측이 너무 가까워짐

                    if (
                        right_d > 0
                        and right_d <= SIDE_DANGER_THRESH
                    ):

                        print(
                            f"[긴급 정지] "
                            f"회전 중 Right = "
                            f"{right_d:.1f} cm"
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
                            "[회전 완료] "
                            "RIGHT 방향 통과 시작"
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


                # ------------------------------------------------
                # 현재 센서 위험 여부
                # ------------------------------------------------

                front_danger = (
                    front_d > 0
                    and front_d <= EMERGENCY_FRONT_THRESH
                )


                left_danger = (
                    left_d > 0
                    and left_d <= SIDE_DANGER_THRESH
                )


                right_danger = (
                    right_d > 0
                    and right_d <= SIDE_DANGER_THRESH
                )


                # ------------------------------------------------
                # 정면 위험
                # ------------------------------------------------

                if front_danger:

                    print(
                        f"[긴급 정지] "
                        f"회피 중 Front = "
                        f"{front_d:.1f} cm"
                    )


                    current_state = (
                        STATE_BLOCKED_STOP
                    )


                    with error_lock:

                        shared_speed = 0.0

                        shared_angular_override = 0.0


                # ------------------------------------------------
                # LEFT 회피 중 왼쪽 위험
                # ------------------------------------------------

                elif (
                    avoid_direction == "LEFT"
                    and left_danger
                ):

                    print(
                        f"[긴급 정지] "
                        f"회피 중 Left = "
                        f"{left_d:.1f} cm"
                    )


                    current_state = (
                        STATE_BLOCKED_STOP
                    )


                    with error_lock:

                        shared_speed = 0.0

                        shared_angular_override = 0.0


                # ------------------------------------------------
                # RIGHT 회피 중 오른쪽 위험
                # ------------------------------------------------

                elif (
                    avoid_direction == "RIGHT"
                    and right_danger
                ):

                    print(
                        f"[긴급 정지] "
                        f"회피 중 Right = "
                        f"{right_d:.1f} cm"
                    )


                    current_state = (
                        STATE_BLOCKED_STOP
                    )


                    with error_lock:

                        shared_speed = 0.0

                        shared_angular_override = 0.0


                # ------------------------------------------------
                # 정면 장애물이 충분히 멀어짐
                # ------------------------------------------------

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


                        current_state = (
                            STATE_RECOVERY_TURN
                        )

                        state_timer = time.time()

                        clear_confirm_counter = 0


                # ------------------------------------------------
                # 아직 장애물이 존재
                # ------------------------------------------------

                else:

                    clear_confirm_counter = 0


                    # --------------------------------------------
                    # 최대 회피 시간 초과
                    # --------------------------------------------

                    if (
                        elapsed
                        >= MAX_AVOID_FORWARD_TIME
                    ):

                        print(
                            "[경고] "
                            "최대 회피 시간 초과 -> 정지"
                        )


                        current_state = (
                            STATE_BLOCKED_STOP
                        )


                        with error_lock:

                            shared_speed = 0.0

                            shared_angular_override = 0.0


                    else:

                        # ----------------------------------------
                        # 정상 회피 전진
                        # ----------------------------------------

                        with error_lock:

                            shared_speed = AVOID_SPEED

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


                # ------------------------------------------------
                # 복귀 중 정면 장애물 재감지
                # ------------------------------------------------

                if (
                    front_d > 0
                    and front_d <= EMERGENCY_FRONT_THRESH
                ):

                    print(
                        f"[복귀 중 장애물 재감지] "
                        f"Front = {front_d:.1f} cm"
                    )


                    current_state = (
                        STATE_BLOCKED_STOP
                    )


                    with error_lock:

                        shared_speed = 0.0

                        shared_angular_override = 0.0


                # =================================================
                # LEFT에서 회피했을 경우
                # =================================================

                elif avoid_direction == "LEFT":

                    if elapsed < RECOVERY_TURN_TIME:

                        with error_lock:

                            shared_speed = 0.0

                            shared_angular_override = (
                                -TURN_ANGULAR_SPEED
                            )

                    else:

                        print(
                            "[복귀 완료] "
                            "PID 차선 추종 재개"
                        )


                        current_state = (
                            STATE_LANE_FOLLOWING
                        )

                        avoid_direction = None


                        with error_lock:

                            shared_angular_override = None


                # =================================================
                # RIGHT에서 회피했을 경우
                # =================================================

                else:

                    if elapsed < RECOVERY_TURN_TIME:

                        with error_lock:

                            shared_speed = 0.0

                            shared_angular_override = (
                                TURN_ANGULAR_SPEED
                            )

                    else:

                        print(
                            "[복귀 완료] "
                            "PID 차선 추종 재개"
                        )


                        current_state = (
                            STATE_LANE_FOLLOWING
                        )

                        avoid_direction = None


                        with error_lock:

                            shared_angular_override = None


            # =================================================
            # STATE 6
            # 회피 불가능 -> 정지
            # =================================================

            elif current_state == STATE_BLOCKED_STOP:

                # 무조건 정지

                with error_lock:

                    shared_speed = 0.0

                    shared_angular_override = 0.0


                # 센서가 정상적으로 연결된 경우
                # 장애물 제거 여부 확인

                if sensors_ready:

                    obstacle_cleared = (
                        front_d < 0
                        or front_d > OBSTACLE_CLEAR_THRESH
                    )


                    if obstacle_cleared:

                        print(
                            "[장애물 제거 확인] "
                            "정상 주행 재개"
                        )


                        current_state = (
                            STATE_LANE_FOLLOWING
                        )

                        avoid_direction = None


                        with error_lock:

                            shared_angular_override = None


            # =================================================
            # E. 화면 표시
            # =================================================

            display = frame.copy()


            # ------------------------------------------------
            # 현재 상태
            # ------------------------------------------------

            cv2.putText(
                display,
                f"STATE: {current_state}",
                (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2
            )


            # ------------------------------------------------
            # 초음파 값
            # ------------------------------------------------

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


            # ------------------------------------------------
            # 차선 인식
            # ------------------------------------------------

            cv2.putText(
                display,
                f"LANE: {'OK' if is_detected else 'LOST'}",
                (10, 75),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                2
            )


            # ------------------------------------------------
            # 회피 방향
            # ------------------------------------------------

            if avoid_direction is not None:

                cv2.putText(
                    display,
                    f"AVOID: {avoid_direction}",
                    (10, 100),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 255),
                    2
                )


            # ------------------------------------------------
            # 화면 출력
            # ------------------------------------------------

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
        # 모터 정지
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