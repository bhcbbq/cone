# main_highway_contour_avoid.py
# UGV02 Highway Edge/Contour Avoidance + SSH Terminal Diagnostic Logging

import cv2
import time
import threading
import traceback
import math
import numpy as np
import os
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

FRONT_STOP_CM = 55.0          # 장애물 회피 시작 거리 (55cm)
OBSTACLE_CLEAR_CM = 60.0      # 비상 정지 해제 판단 거리
OBSTACLE_CLEAR_HOLD_S = 0.5

NORMAL_SPEED = 0.15           # m/s (일반 차선 추종)
AVOID_SPEED = 0.10            # m/s (S자 회피 속도)
PASS_SPEED = 0.12             # m/s (장애물 측면 직진 통과 속도)

TURN_OUT_TIME_S = 0.45        # 1단계: 대각선 틀기 시간
ALIGN_TIME_S = 0.45           # 2단계: 차선 평행 정렬 시간
TURN_ANGULAR_SPEED = 1.1      # rad/s (회전 각속도)

PASS_DISTANCE_CM = 90.0       # 장애물 통과 직진 거리 (90cm)
PASS_TIMEOUT_S = PASS_DISTANCE_CM / (PASS_SPEED * 100.0)
RECOVERY_TIMEOUT_S = 6.0      

CONTROL_PERIOD_S = 0.02
CAMERA_PERIOD_S = 0.03
NO_OBSTACLE = -1.0

# SSH 환경 터미널 전용 로그 출력 간격 (초)
TERMINAL_LOG_INTERVAL_S = 1.0

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
    print(f"\n[FSM STATE TRANSITION] -> {state.name}")


def state_elapsed():
    return time.monotonic() - state_enter_time


def open_camera():
    cap = cv2.VideoCapture(GST_PIPELINE, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        raise RuntimeError("CSI 카메라를 열 수 없습니다.")
    return cap


# ============================================================
# Vision Logic (Canny Edge + Contour Based)
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


def get_obstacle_direction_by_contour(frame, lines):
    height, width = frame.shape[:2]
    x_left, x_right = estimate_lane_boundaries(lines, width, height)

    if x_left is None or x_right is None:
        return AvoidDirection.LEFT

    lane_center = (x_left + x_right) / 2.0
    lane_px = x_right - x_left
    margin_px = int(lane_px * 0.10)

    y1, y2 = int(height * 0.52), int(height * 0.84)
    x1 = max(0, int(x_left + margin_px))
    x2 = min(width, int(x_right - margin_px))

    if x2 <= x1 or y2 <= y1:
        return AvoidDirection.LEFT

    roi = frame[y1:y2, x1:x2]
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)

    _, thresh = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

    kernel = np.ones((5, 5), np.uint8)
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    min_area = max(40, int((x2 - x1) * (y2 - y1) * 0.01))
    valid_contours = [c for c in contours if cv2.contourArea(c) >= min_area]

    if valid_contours:
        largest_contour = max(valid_contours, key=cv2.contourArea)
        bx, by, bw, bh = cv2.boundingRect(largest_contour)
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
        raise RuntimeError("아두이노 초음파 센서 시리얼 연결 실패")
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
            avoid_direction = get_obstacle_direction_by_contour(frame, lines)
            print(f"\n[OBSTACLE DETECTED] Ultrasonic Front: {front_d:.1f}cm -> Avoid Dir: {avoid_direction.name}")
            transition(State.AVOID_TURN_OUT)
        elif not is_detected:
            force_stop()
        else:
            set_command(NORMAL_SPEED, None, error=error)
        return

    # [2] S자 회피 1단계: 대각선 틀기
    if state == State.AVOID_TURN_OUT:
        set_command(AVOID_SPEED, avoid_direction.value * TURN_ANGULAR_SPEED)
        if state_elapsed() >= TURN_OUT_TIME_S:
            transition(State.AVOID_ALIGN)
        return

    # [3] S자 회피 2단계: 차선 평행 정렬
    if state == State.AVOID_ALIGN:
        set_command(AVOID_SPEED, -avoid_direction.value * TURN_ANGULAR_SPEED)
        if state_elapsed() >= ALIGN_TIME_S:
            transition(State.AVOID_FORWARD)
        return

    # [4] 회피 3단계: 장애물 측면 직진 통과
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
        print("==================================================")
        print("[SYSTEM INIT] UGV02 Autonomous System Starting...")
        print("==================================================")

        # 1. UGV02 시리얼 초기화
        ugv_serial = ugv_control.init_ugv(port=UGV_PORT, baudrate=SERIAL_BAUD)
        print("[INIT OK] UGV02 Serial Connected.")

        # 2. 초음파 센서 초기화 및 수신 테스트 (SSH 수신 진단)
        print("[INIT] Connecting Ultrasonic Arduino...")
        ultrasonic = create_ultrasonic_reader()
        time.sleep(0.5)

        # [진단 로직 1] 초기 초음파 데이터 수신 확인
        test_left, test_front, test_right = read_distances(ultrasonic)
        print(f"[ULTRASONIC DIAGNOSTIC OK] Arduino -> Jetson Data Received!")
        print(f" -> Initial Distance: Left={test_left:.1f}cm | Front={test_front:.1f}cm | Right={test_right:.1f}cm\n")

        # 3. 카메라 초기화
        print("[INIT] Opening CSI Camera...")
        camera = open_camera()
        print("[INIT OK] Camera Opened.")

        # 4. 제어 백그라운드 스레드 시작
        control_thread = threading.Thread(
            target=control_thread_task, args=(ugv_serial,), daemon=True
        )
        control_thread.start()
        force_stop()

        # GUI 디스플레이 세팅 유무 확인
        has_display = bool(os.environ.get("DISPLAY", None))
        if not has_display:
            print("[INFO] Headless Mode Detected (SSH execution without GUI display).")

        print("--------------------------------------------------")
        print("[RUNNING] Autonomous driving active. Press Ctrl+C in terminal to stop.")
        print("--------------------------------------------------\n")

        last_terminal_log_time = time.monotonic()

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

            # FSM 주행 상태 제어
            run_fsm(error, is_detected, frame, lines, left_d, front_d, right_d)

            # [진단 로직 2] SSH 터미널로 실시간 초음파 데이터 주기적 출력 (1초 간격)
            now = time.monotonic()
            if now - last_terminal_log_time >= TERMINAL_LOG_INTERVAL_S:
                print(
                    f"[SSH MONITOR] State: {state.name:<15} | "
                    f"US Distances (L/F/R): {left_d:5.1f} / {front_d:5.1f} / {right_d:5.1f} cm | "
                    f"Lane Error: {error:+.2f}"
                )
                last_terminal_log_time = now

            # SSH 환경에서 DISPLAY가 있을 때만 OpenCV 화면 표시
            if has_display:
                cv2.putText(
                    annotated, f"STATE: {state.name}", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2
                )
                cv2.putText(
                    annotated, f"US L/F/R: {left_d:.1f}/{front_d:.1f}/{right_d:.1f} cm", (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1
                )
                cv2.imshow("UGV02 Edge/Contour Avoidance", annotated)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

            time.sleep(CAMERA_PERIOD_S)

    except KeyboardInterrupt:
        print("\n[STOP] Keyboard Interrupt detected via SSH.")
    except Exception as exc:
        print(f"\n[ERROR] Exception occurred: {exc!r}")
        traceback.print_exc()
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
        print("[EXIT] Autonomous driving system safely shutdown.")


if __name__ == "__main__":
    main()