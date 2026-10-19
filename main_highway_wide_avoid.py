# main_highway_wide_avoid.py
# UGV02 Lane Following + Multi-Color Obstacle Avoidance (Highway Wide Avoidance Version)

import cv2
import time
import threading
import traceback
import math
import numpy as np
from enum import Enum, auto

import lane_detection
import ugv_control
from ultrasonic_reader import UltrasonicReader


# ============================================================
# Configuration (고속도로 넓은 차선 맞춤 설정)
# ============================================================

UGV_PORT = "/dev/ttyACM0"
ULTRASONIC_PORT = "/dev/ttyUSB0"
SERIAL_BAUD = 115200

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 30

LANE_WIDTH_CM = 70.0

# [수정] 감지 거리 확장: 35cm -> 55cm (크게 꺾으므로 멀리서 미리 회피 시작)
FRONT_STOP_CM = 55.0          
OBSTACLE_CLEAR_CM = 60.0      # 비상 정지 해제 판단 거리
OBSTACLE_CLEAR_HOLD_S = 0.5

NORMAL_SPEED = 0.15           # m/s (일반 차선 추종)
AVOID_SPEED = 0.10            # m/s (S자 회피 속도: 원활한 회전을 위해 0.10m/s로 소폭 상승)
PASS_SPEED = 0.12             # m/s (장애물 측면 직진 통과 속도)

# [수정] 대각선 이동 및 정렬 시간 확장: 0.25초 -> 0.45초 (측면 이동 거리 약 30cm 이상으로 확장)
TURN_OUT_TIME_S = 0.45        # 1단계: 대각선 틀기 시간
ALIGN_TIME_S = 0.45           # 2단계: 차선 평행 정렬 시간
TURN_ANGULAR_SPEED = 1.1      # rad/s (회전 각속도)

# [수정] 통과 거리 확장: 75cm -> 90cm (더 크게 비켜섰으므로 안전 통과 거리 확보)
PASS_DISTANCE_CM = 90.0       
PASS_TIMEOUT_S = PASS_DISTANCE_CM / (PASS_SPEED * 100.0)
RECOVERY_TIMEOUT_S = 6.0      # 복귀 타임아웃 확장 (5초 -> 6초)

CONTROL_PERIOD_S = 0.02
CAMERA_PERIOD_S = 0.03
NO_OBSTACLE = -1.0

# CSI camera GStreamer pipeline
GST_PIPELINE = (
    "nvarguscamerasrc ! "
    "video/x-raw(memory:NVMM), width=1280, height=720, "
    "format=NV12, framerate=30/1 ! "
    "nvvidconv ! "
    "video/x-raw, width=640, height=480, format=BGRx ! "
    "videoconvert ! "
    "video/x-raw, format=BGR ! "
    "appsink drop=true max-buffers=1 sync=false"
)


class State(Enum):
    LANE_FOLLOWING = auto()
    AVOID_TURN_OUT = auto()
    AVOID_ALIGN = auto()
    AVOID_FORWARD = auto()
    RECOVERY_TURN = auto()
    BLOCKED_STOP = auto()


class AvoidDirection(Enum):
    LEFT = -1
    RIGHT = 1


# Shared control state
shared_lock = threading.Lock()
shared_error = 0.0
shared_speed = 0.0
shared_angular_override = 0.0
shutdown_event = threading.Event()

state = State.LANE_FOLLOWING
state_enter_time = time.monotonic()
avoid_direction = AvoidDirection.LEFT
obstacle_clear_since = None
last_blocked_log_time = 0.0


def set_command(speed, angular_override, error=None):
    global shared_speed, shared_angular_override, shared_error
    with shared_lock:
        shared_speed = float(speed)
        shared_angular_override = (
            None if angular_override is None else float(angular_override)
        )
        if error is not None:
            shared_error = float(error)


def force_stop():
    set_command(0.0, 0.0)


def transition(new_state):
    global state, state_enter_time
    state = new_state
    state_enter_time = time.monotonic()
    print(f"[FSM] -> {state.name}")


def state_elapsed():
    return time.monotonic() - state_enter_time


def open_camera():
    cap = cv2.VideoCapture(GST_PIPELINE, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        raise RuntimeError("CSI 카메라를 열 수 없습니다.")
    return cap


# ============================================================
# Vision Logic (Multi-Color Inverse Masking)
# ============================================================

def detect_hough_lines(frame):
    height, width = frame.shape[:2]
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blur, 50, 150)

    mask = np.zeros_like(edges)
    polygon = np.array([[
        (int(width * 0.05), height - 1),
        (int(width * 0.40), int(height * 0.48)),
        (int(width * 0.60), int(height * 0.48)),
        (int(width * 0.95), height - 1),
    ]], dtype=np.int32)
    cv2.fillPoly(mask, polygon, 255)
    roi_edges = cv2.bitwise_and(edges, mask)

    lines = cv2.HoughLinesP(
        roi_edges, rho=1, theta=np.pi / 180, threshold=35,
        minLineLength=25, maxLineGap=45,
    )
    return lines


def line_x_at_y(line, y):
    values = np.asarray(line).reshape(-1)
    if len(values) < 4:
        return None
    x1, y1, x2, y2 = map(float, values[:4])
    if abs(y2 - y1) < 1e-6:
        return (x1 + x2) / 2.0
    return x1 + (y - y1) * (x2 - x1) / (y2 - y1)


def estimate_lane_boundaries(lines, image_width, image_height):
    if lines is None:
        return None, None

    y_eval = int(image_height * 0.75)
    center = image_width / 2.0
    left_xs, right_xs = [], []

    try:
        segments = np.asarray(lines).reshape(-1, 4)
    except (ValueError, TypeError):
        return None, None

    for segment in segments:
        x1, y1, x2, y2 = map(float, segment)
        dx, dy = x2 - x1, y2 - y1
        if abs(dx) < 1e-6 or abs(dy / dx) < 0.5:
            continue
        x = line_x_at_y(segment, y_eval)
        if x is None or not (0 <= x < image_width):
            continue
        if dy / dx < 0 and x < center:
            left_xs.append(x)
        elif dy / dx > 0 and x > center:
            right_xs.append(x)

    if not left_xs or not right_xs:
        return None, None

    x_left = float(np.median(left_xs))
    x_right = float(np.median(right_xs))
    return (x_left, x_right) if x_right > x_left else (None, None)


def get_any_color_obstacle_direction(frame, lines):
    """
    검은색 바닥과 흰색 차선을 제외한 모든 색상을 감지하여
    장애물 반대 방향을 구합니다.
    """
    height, width = frame.shape[:2]
    x_left, x_right = estimate_lane_boundaries(lines, width, height)

    if x_left is None or x_right is None:
        return AvoidDirection.LEFT

    lane_center = (x_left + x_right) / 2.0
    lane_px = x_right - x_left
    margin_px = int(lane_px * 0.08)

    y1, y2 = int(height * 0.52), int(height * 0.84)
    x1 = max(0, int(x_left + margin_px))
    x2 = min(width, int(x_right - margin_px))

    if x2 <= x1 or y2 <= y1:
        return AvoidDirection.LEFT

    roi = frame[y1:y2, x1:x2]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    # 1. 검은색 바닥 영역 (V <= 60)
    mask_black = cv2.inRange(
        hsv, 
        np.array([0, 0, 0], np.uint8), 
        np.array([179, 255, 60], np.uint8)
    )

    # 2. 흰색 차선 영역 (S <= 50, V >= 170)
    mask_white = cv2.inRange(
        hsv, 
        np.array([0, 0, 170], np.uint8), 
        np.array([179, 50, 255], np.uint8)
    )

    floor_and_lane = cv2.bitwise_or(mask_black, mask_white)
    mask_obstacle = cv2.bitwise_not(floor_and_lane)

    kernel = np.ones((5, 5), np.uint8)
    mask_obstacle = cv2.morphologyEx(mask_obstacle, cv2.MORPH_OPEN, kernel)
    mask_obstacle = cv2.morphologyEx(mask_obstacle, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask_obstacle, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    min_area = max(30, int((x2 - x1) * (y2 - y1) * 0.005))
    contours = [c for c in contours if cv2.contourArea(c) >= min_area]

    if contours:
        contour = max(contours, key=cv2.contourArea)
        bx, by, bw, bh = cv2.boundingRect(contour)
        obs_center = x1 + bx + (bw / 2.0)

        if obs_center >= lane_center:
            return AvoidDirection.LEFT
        else:
            return AvoidDirection.RIGHT

    return AvoidDirection.LEFT


# ============================================================
# Ultrasonic Sensor & Control
# ============================================================

def create_ultrasonic_reader():
    reader = UltrasonicReader(port=ULTRASONIC_PORT, baudrate=SERIAL_BAUD)
    if not reader.start():
        raise RuntimeError("아두이노 초음파 센서 연결 실패")
    return reader


def read_distances(reader):
    values = reader.get_distances()
    if not isinstance(values, (tuple, list)) or len(values) != 3:
        raise ValueError("get_distances() 형태 오류")

    result = []
    for value in values:
        try:
            d = float(value)
            if not math.isfinite(d):
                d = NO_OBSTACLE
        except (TypeError, ValueError):
            d = NO_OBSTACLE
        result.append(d)
    return tuple(result)


def front_obstacle_detected(front_d):
    return front_d > 0.0 and front_d <= FRONT_STOP_CM


def control_thread_task(ugv_serial):
    previous_error = 0.0
    previous_time = time.monotonic()

    while not shutdown_event.is_set():
        now = time.monotonic()
        dt = max(now - previous_time, 1e-3)
        previous_time = now

        with shared_lock:
            error = shared_error
            speed = shared_speed
            override = shared_angular_override

        try:
            ugv_control.read_odometry(ugv_serial, dt)

            if speed <= 0.0:
                angular_z = 0.0
            elif override is not None:
                angular_z = override
            else:
                angular_z = ugv_control.calculate_pid(error, previous_error, dt)

            previous_error = error
            ugv_control.send_driving_command(ugv_serial, speed, angular_z)

        except Exception as exc:
            print(f"[CONTROL ERROR] {exc!r}")
            force_stop()
            shutdown_event.set()
            break

        time.sleep(CONTROL_PERIOD_S)


# ============================================================
# FSM
# ============================================================

def run_fsm(error, is_detected, frame, lines, left_d, front_d, right_d):
    global avoid_direction, obstacle_clear_since, last_blocked_log_time
    now = time.monotonic()

    # [1] 일반 차선 추종
    if state == State.LANE_FOLLOWING:
        if front_obstacle_detected(front_d):
            avoid_direction = get_any_color_obstacle_direction(frame, lines)
            print(f"[WIDE AVOID START] Multi-Color Filter -> Avoid {avoid_direction.name}")
            transition(State.AVOID_TURN_OUT)
        elif not is_detected:
            force_stop()
        else:
            set_command(NORMAL_SPEED, None, error=error)
        return

    # [2] 넓은 S자 회피 1단계: 대각선 틀기 (0.45초 동안 크게 이동)
    if state == State.AVOID_TURN_OUT:
        set_command(AVOID_SPEED, avoid_direction.value * TURN_ANGULAR_SPEED)
        if state_elapsed() >= TURN_OUT_TIME_S:
            transition(State.AVOID_ALIGN)
        return

    # [3] 넓은 S자 회피 2단계: 차선 평행 정렬 (0.45초)
    if state == State.AVOID_ALIGN:
        set_command(AVOID_SPEED, -avoid_direction.value * TURN_ANGULAR_SPEED)
        if state_elapsed() >= ALIGN_TIME_S:
            transition(State.AVOID_FORWARD)
        return

    # [4] 회피 3단계: 여유 공간 직진 통과 (90cm)
    if state == State.AVOID_FORWARD:
        set_command(PASS_SPEED, 0.0)
        if state_elapsed() >= PASS_TIMEOUT_S:
            force_stop()
            transition(State.RECOVERY_TURN)
        return

    # [5] 원래 차선 복귀 회전
    if state == State.RECOVERY_TURN:
        if state_elapsed() >= RECOVERY_TIMEOUT_S:
            force_stop()
            print("[SAFETY] 차선 복귀 시간 초과")
            transition(State.BLOCKED_STOP)
            return

        if not hasattr(run_fsm, "_recovery_detect_count"):
            run_fsm._recovery_detect_count = 0

        if is_detected:
            run_fsm._recovery_detect_count += 1
        else:
            run_fsm._recovery_detect_count = 0

        if run_fsm._recovery_detect_count >= 5:
            run_fsm._recovery_detect_count = 0
            force_stop()
            transition(State.LANE_FOLLOWING)
            return

        set_command(AVOID_SPEED, -avoid_direction.value * TURN_ANGULAR_SPEED)
        return

    # [6] 복귀 실패/비상 정지 대기
    if state == State.BLOCKED_STOP:
        force_stop()
        clear = (front_d > OBSTACLE_CLEAR_CM) or (front_d == NO_OBSTACLE)
        if clear:
            if obstacle_clear_since is None:
                obstacle_clear_since = now
            elif now - obstacle_clear_since >= OBSTACLE_CLEAR_HOLD_S:
                obstacle_clear_since = None
                transition(State.LANE_FOLLOWING)
        else:
            obstacle_clear_since = None


# ============================================================
# Main Entry Point
# ============================================================

def main():
    camera = None
    ugv_serial = None
    ultrasonic = None
    control_thread = None

    try:
        ugv_serial = ugv_control.init_ugv(port=UGV_PORT, baudrate=SERIAL_BAUD)
        ultrasonic = create_ultrasonic_reader()
        camera = open_camera()

        control_thread = threading.Thread(
            target=control_thread_task, args=(ugv_serial,), daemon=True
        )
        control_thread.start()
        force_stop()

        while not shutdown_event.is_set():
            ret, frame = camera.read()
            if not ret or frame is None:
                force_stop()
                time.sleep(0.1)
                continue

            lines = detect_hough_lines(frame)
            error, annotated, is_detected = lane_detection.calculate_steering(
                frame.copy(), lines
            )
            left_d, front_d, right_d = read_distances(ultrasonic)

            run_fsm(error, is_detected, frame, lines, left_d, front_d, right_d)

            cv2.putText(
                annotated, f"STATE: {state.name}", (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2
            )
            cv2.putText(
                annotated, f"Avoid Dir: {avoid_direction.name}", (10, 50),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1
            )
            cv2.imshow("UGV02 Highway Wide Avoidance", annotated)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

            time.sleep(CAMERA_PERIOD_S)

    except KeyboardInterrupt:
        pass
    finally:
        shutdown_event.set()
        force_stop()
        if control_thread is not None:
            control_thread.join(timeout=1.0)
        if ugv_serial is not None:
            ugv_control.stop_ugv(ugv_serial)
        if ultrasonic is not None:
            ultrasonic.stop()
        if camera is not None:
            camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()