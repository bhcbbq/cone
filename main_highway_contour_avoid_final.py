# main_highway_contour_avoid_final.py
# UGV02 Highway Edge/Contour Avoidance + lane_detection_ver1.py + Ultrasonic Debouncing

import cv2
import time
import threading
import traceback
import math
import numpy as np
import os
from enum import Enum, auto

# 버전 1 모듈 임포트
import lane_detection_ver1 as lane_detection
import ugv_control
from ultrasonic_reader import UltrasonicReader


# ============================================================
# Configuration (고속도로 넓은 차선 맞춤 설정)
# ============================================================

UGV_PORT = "/dev/ttyACM0"
ULTRASONIC_PORT = "/dev/ttyUSB0"
SERIAL_BAUD = 115200

FRAME_WIDTH = 640
CAMERA_FPS = 30
FRAME_TIMEOUT = 0.5
MAX_CAMERA_FAILURES = 3

FRONT_STOP_CM = 55.0          # 장애물 회피 시작 거리 (55cm)
OBSTACLE_CLEAR_CM = 60.0      # 비상 정지 해제 판단 거리
OBSTACLE_CLEAR_HOLD_S = 0.5

NORMAL_SPEED = 0.3           # m/s (일반 차선 추종)
AVOID_SPEED = 0.10            # m/s (S자 회피 속도)
PASS_SPEED = 0.12             # m/s (장애물 측면 직진 통과 속도)

TURN_OUT_TIME_S = 0.45        # 1단계: 대각선 틀기 시간
ALIGN_TIME_S = 0.45           # 2단계: 차선 평행 정렬 시간
TURN_ANGULAR_SPEED = 1.1      # rad/s (회전 각속도)

PASS_DISTANCE_CM = 90.0       # 장애물 통과 직진 거리 (90cm)
PASS_TIMEOUT_S = PASS_DISTANCE_CM / (PASS_SPEED * 100.0)
RECOVERY_TIMEOUT_S = 6.0      

CONTROL_PERIOD_S = 0.02
NO_OBSTACLE = -1.0
TERMINAL_LOG_INTERVAL_S = 1.0

# SSH 접속 여부에 따른 GUI 제어
SHOW_WINDOWS = bool(os.environ.get("DISPLAY")) and not bool(
    os.environ.get("SSH_CONNECTION") or os.environ.get("SSH_CLIENT") or os.environ.get("SSH_TTY")
)

GST_PIPELINE = (
    "nvarguscamerasrc sensor-id=0 ! "
    "video/x-raw(memory:NVMM), width=(int)1280, height=(int)720, format=(string)NV12, framerate=(fraction)30/1 ! "
    "nvvidconv flip-method=0 ! "
    "video/x-raw, width=(int)640, height=(int)480, format=(string)BGRx ! "
    "videoconvert ! "
    "video/x-raw, format=(string)BGR ! appsink drop=true max-buffers=1 sync=false"
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


# ============================================================
# Shared Control State & Globals
# ============================================================
error_lock = threading.Lock()
shared_error = 0.0
shared_speed = 0.0
shared_angular_override = None  # FSM 회피 기동 시 강제 조향값
shared_frame_time = None
is_running = True

state = State.LANE_FOLLOWING
state_enter_time = time.monotonic()
avoid_direction = AvoidDirection.LEFT
obstacle_clear_since = None

# 센서 노이즈 필터링 및 복구 카운터 (새로 추가됨)
front_obstacle_count = 0
recovery_detect_count = 0


def set_command(speed, angular_override, error=None):
    global shared_speed, shared_angular_override, shared_error
    with error_lock:
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
    print(f"\n[FSM STATE TRANSITION] -> {state.name}", flush=True)

def state_elapsed():
    return time.monotonic() - state_enter_time

def open_camera():
    cap = cv2.VideoCapture(GST_PIPELINE, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        raise RuntimeError("CSI 카메라를 열 수 없습니다.")
    return cap


# ============================================================
# Vision Logic
# ============================================================

def process_lane_frame(frame, tracker, now):
    target_height = max(2, round(frame.shape[0] * FRAME_WIDTH / frame.shape[1]))
    frame = cv2.resize(frame, (FRAME_WIDTH, target_height))

    raw_edges, edges, top, bottom = lane_detection.get_edges(frame, return_raw=True)
    pair, status, inferred = tracker.update(edges, top, bottom, now)

    result_image, error, is_detected = lane_detection.draw_result(
        frame.copy(), pair, status, inferred, top, bottom
    )

    return error, result_image, is_detected, raw_edges, edges, status, pair


def get_obstacle_direction_by_contour(frame, pair):
    if pair is None or len(pair) < 2:
        return AvoidDirection.LEFT

    height, width = frame.shape[:2]
    
    y_eval_norm = ((0.52 + 0.84) / 2.0)
    x_left = np.polyval(pair[0], y_eval_norm)
    x_right = np.polyval(pair[1], y_eval_norm)

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
# Ultrasonic & Control Thread
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


def control_thread_task(ugv_serial):
    global is_running
    previous_error = None
    previous_frame_time = None
    angular_z = 0.0

    print("[알림] 제어 스레드 시작 (FSM 오버라이드 지원)", flush=True)

    try:
        while is_running:
            ugv_control.clear_feedback_buffer(ugv_serial)
            now = time.monotonic()

            with error_lock:
                current_error = shared_error
                current_speed = shared_speed
                override = shared_angular_override
                frame_time = shared_frame_time

            stale = (frame_time is None or now - frame_time > FRAME_TIMEOUT)

            if current_speed <= 0.0 or stale:
                angular_z = 0.0
                previous_error = None
                previous_frame_time = None
            else:
                new_frame = (previous_frame_time is None or frame_time != previous_frame_time)
                
                if override is not None:
                    angular_z = override
                    previous_error = None
                    previous_frame_time = None
                elif new_frame:
                    frame_dt = None if previous_frame_time is None else (frame_time - previous_frame_time)
                    angular_z = ugv_control.calculate_pid(
                        current_error,
                        previous_error,
                        (frame_dt if frame_dt is not None else 1.0)
                    )
                    previous_error = current_error
                    previous_frame_time = frame_time

            ugv_control.send_driving_command(ugv_serial, current_speed, angular_z)
            time.sleep(CONTROL_PERIOD_S)

    except Exception as exc:
        print(f"[에러] 제어 스레드 중단: {exc}", flush=True)
        is_running = False
    finally:
        try:
            ugv_control.send_driving_command(ugv_serial, 0.0, 0.0)
        except:
            pass


# ============================================================
# FSM
# ============================================================

def run_fsm(error, is_detected, frame, pair, left_d, front_d, right_d):
    global avoid_direction, obstacle_clear_since, state
    global front_obstacle_count, recovery_detect_count
    now = time.monotonic()

    # [1] 일반 차선 추종
    if state == State.LANE_FOLLOWING:
        # 노이즈 필터링: 연속 3회 (약 0.06초) 감지해야 장애물로 판단
        if front_d > 0.0 and front_d <= FRONT_STOP_CM:
            front_obstacle_count += 1
        else:
            front_obstacle_count = 0

        if front_obstacle_count >= 3:
            front_obstacle_count = 0
            avoid_direction = get_obstacle_direction_by_contour(frame, pair)
            print(f"\n[OBSTACLE DETECTED] Front: {front_d:.1f}cm -> Avoid Dir: {avoid_direction.name}", flush=True)
            transition(State.AVOID_TURN_OUT)
        elif not is_detected:
            force_stop()
        else:
            set_command(NORMAL_SPEED, None, error=error)
        return

    # [2] S자 회피 1단계
    if state == State.AVOID_TURN_OUT:
        set_command(AVOID_SPEED, avoid_direction.value * TURN_ANGULAR_SPEED)
        if state_elapsed() >= TURN_OUT_TIME_S:
            transition(State.AVOID_ALIGN)
        return

    # [3] S자 회피 2단계
    if state == State.AVOID_ALIGN:
        set_command(AVOID_SPEED, -avoid_direction.value * TURN_ANGULAR_SPEED)
        if state_elapsed() >= ALIGN_TIME_S:
            transition(State.AVOID_FORWARD)
        return

    # [4] 회피 3단계 (직진 통과)
    if state == State.AVOID_FORWARD:
        set_command(PASS_SPEED, 0.0)
        if state_elapsed() >= PASS_TIMEOUT_S:
            force_stop()
            transition(State.RECOVERY_TURN)
        return

    # [5] 차선 복귀 회전
    if state == State.RECOVERY_TURN:
        if state_elapsed() >= RECOVERY_TIMEOUT_S:
            force_stop()
            print("[SAFETY] 차선 복귀 시간 초과", flush=True)
            transition(State.BLOCKED_STOP)
            return

        if is_detected:
            recovery_detect_count += 1
        else:
            recovery_detect_count = 0

        if recovery_detect_count >= 5:
            recovery_detect_count = 0
            force_stop()
            transition(State.LANE_FOLLOWING)
            return

        set_command(AVOID_SPEED, -avoid_direction.value * TURN_ANGULAR_SPEED)
        return

    # [6] 비상 정지 대기
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
    global is_running, shared_error, shared_speed, shared_frame_time, ugv_serial
    
    if lane_detection.ROAD_MODE not in ("highway", "black_track"):
        print("[에러] ROAD_MODE 설정을 확인하세요.", flush=True)
        return

    camera = None
    ugv_serial = None
    ultrasonic = None
    control_thread = None

    window_name = "UGV - ver1 Contour Avoidance"

    try:
        print("==================================================")
        print("[SYSTEM INIT] UGV02 Final Autonomous System Starting...")
        print("==================================================")

        ugv_serial = ugv_control.init_ugv(port=UGV_PORT, baudrate=SERIAL_BAUD)
        print("[INIT OK] UGV02 Serial Connected.")

        print("[INIT] Connecting Ultrasonic Arduino...")
        ultrasonic = create_ultrasonic_reader()
        time.sleep(0.5)

        test_left, test_front, test_right = read_distances(ultrasonic)
        print(f"[ULTRASONIC OK] Arduino -> Jetson Data Received! Front={test_front:.1f}cm\n")

        print("[INIT] Opening CSI Camera...")
        camera = open_camera()
        print("[INIT OK] Camera Opened.")

        tracker = lane_detection.LaneTracker()

        if SHOW_WINDOWS:
            cv2.namedWindow(window_name)

        control_thread = threading.Thread(target=control_thread_task, args=(ugv_serial,), daemon=True)
        control_thread.start()
        force_stop()

        print("--------------------------------------------------")
        print(f"[RUNNING] Mode={lane_detection.ROAD_MODE} | Display={SHOW_WINDOWS}")
        print("[RUNNING] Press Ctrl+C in terminal or 'q' on video window to stop.")
        print("--------------------------------------------------\n")

        last_terminal_log_time = time.monotonic()
        failure_count = 0

        while is_running:
            if not camera.isOpened():
                break

            frame_started = time.monotonic()
            ret, frame = camera.read()

            if not ret or frame is None:
                force_stop()
                tracker.reset()
                failure_count += 1
                if failure_count >= MAX_CAMERA_FAILURES:
                    print("[에러] 연속 프레임 수신 실패", flush=True)
                    break
                time.sleep(0.05)
                continue

            failure_count = 0

            error, result_image, is_detected, raw_edges, edges, status, pair = process_lane_frame(
                frame, tracker, frame_started
            )
            
            left_d, front_d, right_d = read_distances(ultrasonic)

            with error_lock:
                shared_frame_time = frame_started

            run_fsm(error, is_detected, frame, pair, left_d, front_d, right_d)

            now = time.monotonic()
            if now - last_terminal_log_time >= TERMINAL_LOG_INTERVAL_S:
                print(
                    f"[MONITOR] State: {state.name:<15} | "
                    f"US Front: {front_d:5.1f}cm | "
                    f"Lane Error: {error:+.1f} | Status: {status}", flush=True
                )
                last_terminal_log_time = now

            if SHOW_WINDOWS:
                cv2.putText(result_image, f"STATE: {state.name}", (10, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
                cv2.putText(result_image, f"Avoid: {avoid_direction.name}", (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1)
                
                cv2.imshow(window_name, result_image)
                if cv2.waitKey(1) & 0xFF in (ord("q"), ord("Q"), 27):
                    break

    except KeyboardInterrupt:
        print("\n[STOP] Keyboard Interrupt detected.")
    except Exception as exc:
        print(f"\n[ERROR] Exception occurred: {exc!r}")
        traceback.print_exc()
    finally:
        is_running = False
        force_stop()
        if control_thread is not None:
            control_thread.join(timeout=1.0)
        if ugv_serial is not None:
            ugv_control.stop_ugv(ugv_serial)
        if ultrasonic is not None:
            ultrasonic.stop()
        if camera is not None:
            camera.release()
        if SHOW_WINDOWS:
            cv2.destroyAllWindows()
        print("[EXIT] Autonomous driving system safely shutdown.", flush=True)


if __name__ == "__main__":
    main()