# main_test.py
# UGV02 lane following + in-lane obstacle avoidance
#
# Interfaces verified against the supplied files:
#   lane_detection.py:
#       calculate_steering(frame, lines) -> (error, frame, is_detected)
#   ugv_control.py:
#       init_ugv(port, baudrate) -> serial.Serial
#       read_odometry(ser, dt)
#       calculate_pid(current_error, previous_error, dt)
#       send_driving_command(ser, base_speed, angular_z)
#       stop_ugv(ser)
#   ultrasonic_reader.py:
#       UltrasonicReader(port, baudrate), start(), get_distances(), stop()

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
# Configuration
# ============================================================

UGV_PORT = "/dev/ttyACM0"
ULTRASONIC_PORT = "/dev/ttyUSB0"
SERIAL_BAUD = 115200

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 30

LANE_WIDTH_CM = 70.0
VEHICLE_WIDTH_CM = 21.0
MIN_LANE_CLEARANCE_CM = 25.0  # 차체 21cm + 여유 4cm

FRONT_STOP_CM = 45.0
OBSTACLE_CLEAR_CM = 50.0
OBSTACLE_CLEAR_HOLD_S = 0.5

NORMAL_SPEED = 0.15       # m/s
AVOID_SPEED = 0.08        # m/s (S자 회전 및 복귀 시 속도)
PASS_SPEED = 0.12         # m/s (장애물 옆 직진 통과 속도)

TURN_OUT_TIME_S = 0.25    # S자 회피 1단계 시간 (대각선 틀기)
ALIGN_TIME_S = 0.25       # S자 회피 2단계 시간 (차선과 평행 정렬)
TURN_ANGULAR_SPEED = 1.0  # rad/s (마찰 극복 각속도)
PASS_DISTANCE_CM = 45.0   # 장애물 측면 오도메트리/타이머 통과 거리
PASS_TIMEOUT_S = PASS_DISTANCE_CM / (PASS_SPEED * 100.0)
RECOVERY_TIMEOUT_S = 5.0

CONTROL_PERIOD_S = 0.02
CAMERA_PERIOD_S = 0.03
NO_OBSTACLE = -1.0

# CSI camera GStreamer pipeline (640x480 output)
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
    CHECK_OBSTACLE = auto()
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
# None = PID steering; 0.0 = forced straight; nonzero = forced turn
shared_angular_override = 0.0
shutdown_event = threading.Event()

state = State.LANE_FOLLOWING
state_enter_time = time.monotonic()
avoid_direction = None
last_lane_space = (None, None)
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
        raise RuntimeError(
            "CSI 카메라를 열 수 없습니다. GStreamer 파이프라인/카메라 연결을 확인하세요."
        )
    return cap


# ============================================================
# Vision: create Hough line segments & estimate space
# ============================================================

def detect_hough_lines(frame):
    """Return Hough segments in the shape expected by lane_detection.py."""
    height, width = frame.shape[:2]
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blur, 50, 150)

    # Restrict detection to the lower road region.
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
        roi_edges,
        rho=1,
        theta=np.pi / 180,
        threshold=35,
        minLineLength=25,
        maxLineGap=45,
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
    """Estimate lane boundary x positions at y=75% of the image."""
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
        dx = x2 - x1
        dy = y2 - y1
        if abs(dx) < 1e-6:
            continue
        slope = dy / dx
        if abs(slope) < 0.5:
            continue
        x = line_x_at_y(segment, y_eval)
        if x is None or not (0 <= x < image_width):
            continue
        if slope < 0 and x < center:
            left_xs.append(x)
        elif slope > 0 and x > center:
            right_xs.append(x)

    if not left_xs or not right_xs:
        return None, None

    x_left = float(np.median(left_xs))
    x_right = float(np.median(right_xs))
    if x_right <= x_left:
        return None, None
    return x_left, x_right


def estimate_lane_space(frame, lines):
    """
    Estimate free space left/right of the largest WHITE or BLACK obstacle contour.
    """
    height, width = frame.shape[:2]
    x_left, x_right = estimate_lane_boundaries(lines, width, height)
    if x_left is None or x_right is None:
        return None, None

    lane_px = x_right - x_left
    if lane_px <= 1:
        return None, None
    cm_per_px = LANE_WIDTH_CM / lane_px

    # Crop road area inside the estimated lane.
    y1, y2 = int(height * 0.52), int(height * 0.84)
    x1, x2 = max(0, int(x_left)), min(width, int(x_right))
    if x2 <= x1 or y2 <= y1:
        return None, None

    roi = frame[y1:y2, x1:x2]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    # =========================================================
    # [수정] 흰색 및 검은색 장애물 전용 HSV 마스크
    # =========================================================
    # 1. 흰색 마스크 (채도 S: 0~50, 명도 V: 180~255)
    mask_white = cv2.inRange(
        hsv, 
        np.array([0, 0, 180], np.uint8),
        np.array([179, 50, 255], np.uint8)
    )

    # 2. 검은색 마스크 (명도 V: 0~60)
    mask_black = cv2.inRange(
        hsv, 
        np.array([0, 0, 0], np.uint8),
        np.array([179, 255, 60], np.uint8)
    )

    # 흰색 + 검은색 영역 결합
    mask = cv2.bitwise_or(mask_white, mask_black)

    # 노이즈 제거 (Opening/Closing)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    min_area = max(30, int((x2 - x1) * (y2 - y1) * 0.001))
    contours = [c for c in contours if cv2.contourArea(c) >= min_area]

    if contours:
        contour = max(contours, key=cv2.contourArea)
        bx, by, bw, bh = cv2.boundingRect(contour)
        obs_left = x1 + bx
        obs_right = x1 + bx + bw
    else:
        # 윤곽선 미검출 시 예외 fallback (15cm 폭 가정)
        assumed_width_px = 15.0 / cm_per_px
        obs_center = (x_left + x_right) / 2.0
        obs_left = obs_center - assumed_width_px / 2.0
        obs_right = obs_center + assumed_width_px / 2.0

    left_free = max(0.0, (obs_left - x_left) * cm_per_px)
    right_free = max(0.0, (x_right - obs_right) * cm_per_px)
    return float(left_free), float(right_free)


# ============================================================
# Ultrasonic sensor
# ============================================================

def create_ultrasonic_reader():
    reader = UltrasonicReader(
        port=ULTRASONIC_PORT,
        baudrate=SERIAL_BAUD,
    )
    if not reader.start():
        raise RuntimeError("아두이노 초음파 센서 연결에 실패했습니다.")
    return reader


def read_distances(reader):
    """Return [left, front, right]. -1 means no valid echo."""
    values = reader.get_distances()
    if not isinstance(values, (tuple, list)) or len(values) != 3:
        raise ValueError("get_distances()가 [left, front, right]를 반환하지 않습니다.")

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


# ============================================================
# Control thread
# ============================================================

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
                angular_z = ugv_control.calculate_pid(
                    error, previous_error, dt
                )

            previous_error = error
            ugv_control.send_driving_command(
                ugv_serial, speed, angular_z
            )

        except Exception as exc:
            print(f"[CONTROL ERROR] {exc!r}")
            force_stop()
            shutdown_event.set()
            break

        time.sleep(CONTROL_PERIOD_S)


# ============================================================
# FSM
# ============================================================

def choose_avoid_direction(left_space, right_space):
    left_ok = left_space is not None and left_space >= MIN_LANE_CLEARANCE_CM
    right_ok = right_space is not None and right_space >= MIN_LANE_CLEARANCE_CM

    if not left_ok and not right_ok:
        return None
    if left_ok and right_ok:
        return AvoidDirection.LEFT if left_space >= right_space else AvoidDirection.RIGHT
    return AvoidDirection.LEFT if left_ok else AvoidDirection.RIGHT


def run_fsm(error, is_detected, frame, lines, left_d, front_d, right_d):
    global last_lane_space, obstacle_clear_since, last_blocked_log_time

    if state in (State.LANE_FOLLOWING, State.CHECK_OBSTACLE):
        last_lane_space = estimate_lane_space(frame, lines)

    left_space, right_space = last_lane_space
    now = time.monotonic()

    # [상태 1] 차선 추종 주행
    if state == State.LANE_FOLLOWING:
        if front_obstacle_detected(front_d):
            force_stop()
            transition(State.CHECK_OBSTACLE)
        elif not is_detected:
            force_stop()
        else:
            set_command(NORMAL_SPEED, None, error=error)
        return

    # [상태 2] 여유 공간 평가
    if state == State.CHECK_OBSTACLE:
        force_stop()
        if state_elapsed() < 0.2:
            return

        if left_space is None or right_space is None:
            print("[SAFETY] 차선 경계/여유 공간 불명확: 정지 유지")
            transition(State.BLOCKED_STOP)
            return

        direction = choose_avoid_direction(left_space, right_space)
        if direction is None:
            print(
                f"[BLOCKED] 좌우 여유 부족 (최소 25cm 필요): L={left_space:.1f}, "
                f"R={right_space:.1f} cm"
            )
            transition(State.BLOCKED_STOP)
            return

        global avoid_direction
        avoid_direction = direction
        print(
            f"[AVOID] {direction.name}: "
            f"L={left_space:.1f} cm, R={right_space:.1f} cm"
        )
        transition(State.AVOID_TURN_OUT)
        return

    # [상태 3] S자 회피 1단계: 대각선 틀기 (0.25초)
    if state == State.AVOID_TURN_OUT:
        if front_obstacle_detected(front_d):
            force_stop()
            transition(State.BLOCKED_STOP)
            return
        set_command(AVOID_SPEED, avoid_direction.value * TURN_ANGULAR_SPEED)
        if state_elapsed() >= TURN_OUT_TIME_S:
            transition(State.AVOID_ALIGN)
        return

    # [상태 4] S자 회피 2단계: 차선과 평행 정렬 (0.25초)
    if state == State.AVOID_ALIGN:
        if front_obstacle_detected(front_d):
            force_stop()
            transition(State.BLOCKED_STOP)
            return
        set_command(AVOID_SPEED, -avoid_direction.value * TURN_ANGULAR_SPEED)
        if state_elapsed() >= ALIGN_TIME_S:
            transition(State.AVOID_FORWARD)
        return

    # [상태 5] 3단계: 차선 내 평행 직진 통과 (오도메트리/타이머 기준 45cm)
    if state == State.AVOID_FORWARD:
        if front_obstacle_detected(front_d):
            force_stop()
            transition(State.BLOCKED_STOP)
            return
        set_command(PASS_SPEED, 0.0)
        if state_elapsed() >= PASS_TIMEOUT_S:
            force_stop()
            transition(State.RECOVERY_TURN)
        return

    # [상태 6] 원래 차선 복귀 회전
    if state == State.RECOVERY_TURN:
        if front_obstacle_detected(front_d):
            force_stop()
            transition(State.BLOCKED_STOP)
            return

        if state_elapsed() >= RECOVERY_TIMEOUT_S:
            force_stop()
            print("[SAFETY] 차선 복귀 시간 초과: 정지")
            transition(State.BLOCKED_STOP)
            return

        if not hasattr(run_fsm, "_recovery_detect_count"):
            run_fsm._recovery_detect_count = 0

        if is_detected:
            run_fsm._recovery_detect_count += 1
        else:
            run_fsm._recovery_detect_count = 0

        # 연속 5프레임 이상 차선 감지 시 정복귀 처리
        if run_fsm._recovery_detect_count >= 5:
            run_fsm._recovery_detect_count = 0
            force_stop()
            transition(State.LANE_FOLLOWING)
            return

        set_command(AVOID_SPEED, -avoid_direction.value * TURN_ANGULAR_SPEED)
        return

    # [상태 7] 공간 부족 비상 정지
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

        if now - last_blocked_log_time >= 1.0:
            print(f"[BLOCKED_STOP] front={front_d:.1f} cm")
            last_blocked_log_time = now


# ============================================================
# Main
# ============================================================

def main():
    camera = None
    ugv_serial = None
    ultrasonic = None
    control_thread = None

    try:
        print("[INIT] Connecting to UGV02...")
        ugv_serial = ugv_control.init_ugv(
            port=UGV_PORT, baudrate=SERIAL_BAUD
        )
        if ugv_serial is None:
            raise RuntimeError("UGV02 시리얼 연결 실패")

        print("[INIT] Connecting ultrasonic Arduino...")
        ultrasonic = create_ultrasonic_reader()

        print("[INIT] Opening CSI camera...")
        camera = open_camera()

        control_thread = threading.Thread(
            target=control_thread_task,
            args=(ugv_serial,),
            name="UGV-Control",
            daemon=True,
        )
        control_thread.start()
        force_stop()

        print("[READY] q 키를 누르면 정지합니다.")

        while not shutdown_event.is_set():
            ret, frame = camera.read()
            if not ret or frame is None:
                force_stop()
                print("[CAMERA] 프레임 수신 실패: 정지")
                time.sleep(0.1)
                continue

            lines = detect_hough_lines(frame)
            error, annotated, is_detected = lane_detection.calculate_steering(
                frame.copy(), lines
            )

            left_d, front_d, right_d = read_distances(ultrasonic)

            run_fsm(
                error, is_detected, frame, lines,
                left_d, front_d, right_d
            )

            left_space, right_space = last_lane_space
            cv2.putText(
                annotated, f"STATE: {state.name}", (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2
            )
            cv2.putText(
                annotated,
                f"US L/F/R: {left_d:.1f}/{front_d:.1f}/{right_d:.1f} cm",
                (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1
            )
            if left_space is None or right_space is None:
                space_text = "Free L/R: unavailable"
            else:
                space_text = f"Free L/R: {left_space:.1f}/{right_space:.1f} cm"
            cv2.putText(
                annotated, space_text, (10, 75),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1
            )

            cv2.imshow("UGV02 main_test", annotated)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

            time.sleep(CAMERA_PERIOD_S)

    except KeyboardInterrupt:
        print("\n[STOP] Keyboard interrupt")
    except Exception:
        traceback.print_exc()
    finally:
        shutdown_event.set()
        force_stop()

        if control_thread is not None:
            control_thread.join(timeout=1.0)

        if ugv_serial is not None:
            try:
                ugv_control.stop_ugv(ugv_serial)
            except Exception as exc:
                print(f"[STOP WARNING] {exc!r}")

        if ultrasonic is not None:
            try:
                ultrasonic.stop()
            except Exception:
                pass

        if camera is not None:
            camera.release()
        cv2.destroyAllWindows()
        print("[EXIT] Shutdown complete")


if __name__ == "__main__":
    main()