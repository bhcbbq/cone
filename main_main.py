# 최신 수정: 가감속 램핑 + 오버슈트 제거 + 카메라/IMU 퓨전 정렬 + 매트 마찰 극복
import cv2
import time
import threading
import traceback
import math
import numpy as np
import os
from enum import Enum, auto

from lane_detection_ver2 import LaneTracker, lane_edges, draw_result
import ugv_control
from ultrasonic_reader import UltrasonicReader

import json

IMU_YAW_UNIT = "rad"
TURN_ANGLE_DEG = 25.0
TURN_TOLERANCE_DEG = 2.0
TURN_KP = 0.045
TURN_MIN_SPEED = 0.45
TURN_MAX_SPEED = 0.55
TURN_TIMEOUT_S = 6.0
IMU_STALE_TIMEOUT_S = 0.35
IMU_NO_MOTION_TIMEOUT_S = 3
IMU_MIN_MOTION_DEG = 1.0
IMU_MAX_STEP_DEG = 45.0

imu_lock = threading.Lock()
shared_yaw = 0.0
shared_yaw_time = None
shared_yaw_seq = 0
imu_read_error = None
turn_context = None
safety_fault = None


def normalize_delta(degrees):
    return (degrees + 180.0) % 360.0 - 180.0


def get_imu_snapshot():
    with imu_lock:
        return shared_yaw, shared_yaw_time, shared_yaw_seq


def ugv_imu_read_task(ugv_serial):
    global shared_yaw, shared_yaw_time, shared_yaw_seq, imu_read_error
    buffer = ""
    while is_running:
        try:
            waiting = ugv_serial.in_waiting
            if waiting > 0:
                buffer += ugv_serial.read(waiting).decode("utf-8", errors="ignore")
                lines = buffer.split("\n")
                buffer = lines[-1][-8192:]
                for line in reversed(lines[:-1]):
                    try:
                        packet = json.loads(line.strip())
                        if not isinstance(packet, dict) or "yaw" not in packet:
                            continue
                        yaw = float(packet["yaw"])
                        if not math.isfinite(yaw):
                            continue
                        if IMU_YAW_UNIT == "rad":
                            yaw = math.degrees(yaw)
                        elif IMU_YAW_UNIT != "deg":
                            raise ValueError("IMU_YAW_UNIT must be deg or rad")
                        with imu_lock:
                            shared_yaw = yaw
                            shared_yaw_time = time.monotonic()
                            shared_yaw_seq += 1
                        imu_read_error = None
                        break
                    except (ValueError, TypeError, OverflowError):
                        continue
        except Exception as exc:
            imu_read_error = f"{type(exc).__name__}: {exc}"
        time.sleep(0.005)


def latch_safety_fault(reason):
    global safety_fault
    if safety_fault is None:
        safety_fault = reason
        print(f"[SAFETY STOP] {reason}; fix cause and restart", flush=True)
    force_stop()


def reset_turn():
    global turn_context
    turn_context = None


def turn_watchdog_reason(now):
    ctx = turn_context
    if ctx is None:
        return None
    _, stamp, _ = get_imu_snapshot()
    if stamp is None or now - stamp > IMU_STALE_TIMEOUT_S:
        return "IMU feedback missing/stale"
    if now - ctx["started"] >= TURN_TIMEOUT_S:
        return "IMU turn timeout"
    if now - ctx["last_motion"] >= IMU_NO_MOTION_TIMEOUT_S:
        return "IMU yaw not changing during turn"
    return None


def run_imu_turn(command_direction, next_state):
    global turn_context
    now = time.monotonic()
    yaw, stamp, seq = get_imu_snapshot()
    if stamp is None or now - stamp > IMU_STALE_TIMEOUT_S:
        latch_safety_fault("Cannot start/continue turn without fresh IMU")
        return
    if turn_context is None:
        turn_context = {
            "started": now, "last_yaw": yaw, "last_seq": seq,
            "accumulated": 0.0, "motion_anchor": 0.0, "last_motion": now,
            "direction": command_direction,
        }
        print(f"[TURN START] yaw_deg={yaw:.2f}, target={TURN_ANGLE_DEG:.1f}, command_sign={command_direction}", flush=True)
    ctx = turn_context
    if ctx["direction"] != command_direction:
        latch_safety_fault("Unexpected turn direction change")
        return
    if seq != ctx["last_seq"]:
        delta = normalize_delta(yaw - ctx["last_yaw"])
        if abs(delta) > IMU_MAX_STEP_DEG:
            latch_safety_fault(f"IMU discontinuity: {delta:.1f} deg/sample")
            return
        ctx["accumulated"] += delta
        ctx["last_yaw"], ctx["last_seq"] = yaw, seq
        if abs(ctx["accumulated"] - ctx["motion_anchor"]) >= IMU_MIN_MOTION_DEG:
            ctx["last_motion"] = now
            ctx["motion_anchor"] = ctx["accumulated"]
    progress = abs(ctx["accumulated"])
    remaining = TURN_ANGLE_DEG - progress
    if remaining <= TURN_TOLERANCE_DEG:
        force_stop()
        print(f"[TURN DONE] net_rotation={ctx['accumulated']:.2f} deg", flush=True)
        reset_turn()
        transition(next_state)
        return
    reason = turn_watchdog_reason(now)
    if reason:
        latch_safety_fault(reason)
        return
    angular = min(TURN_MAX_SPEED, max(TURN_MIN_SPEED, TURN_KP * remaining))
    set_command(0.0, command_direction * angular)


# ============================================================
# Configuration
# ============================================================
UGV_PORT = "/dev/ttyACM0"
ULTRASONIC_PORT = "/dev/ttyUSB0"
SERIAL_BAUD = 115200

FRAME_TIMEOUT = 0.5
MAX_CAMERA_FAILURES = 3

BASE_SPEED = 0.30             # m/s (일반 차선 추종)
AVOID_SPEED = 0.15            # m/s (대각선 회피 속도)
PASS_SPEED = 0.40             # m/s (직진 통과 속도)

# ★ 가감속 램핑 제한 파라미터
MAX_LINEAR_ACCEL = 0.6        # 초당 직진 가속도 제한 (m/s^2)
MAX_LINEAR_DECEL = 1.2        # 초당 직진 감속도 제한 (m/s^2)
MAX_ANGULAR_ACCEL = 2.5       # 초당 회전 가속도 제한 (rad/s^2)

# 거리 기반 회피 조건 파라미터
FRONT_STOP_CM = 70.0
SIDE_DANGER_CM = 28.0
SIDE_CLEAR_CM = 80.0

CONTROL_PERIOD_S = 0.02
NO_OBSTACLE = -1.0
TERMINAL_LOG_INTERVAL_S = 1.0

SHOW_WINDOWS = bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")) and not any(
    os.environ.get(name) for name in ("SSH_CONNECTION", "SSH_CLIENT", "SSH_TTY")
)
LANE_WINDOW = "Smart Cone - Lane Tracking"
CANNY_WINDOW = "Smart Cone - Raw Canny | Lane ROI"


# ============================================================
# FSM State & Direction
# ============================================================
class State(Enum):
    LANE_FOLLOWING = auto()
    AVOID_START_PAUSE = auto()
    AVOID_TURN_OUT = auto()
    AVOID_PAUSE = auto()
    AVOID_DIAGONAL = auto()
    AVOID_ALIGN = auto()
    AVOID_FORWARD = auto()
    RECOVERY_TURN_IN = auto()
    RECOVERY_DIAGONAL = auto()

class AvoidDirection(Enum):
    LEFT = 1
    RIGHT = -1

# ============================================================
# Shared Globals
# ============================================================
error_lock = threading.Lock()
shared_error = 0.0
shared_speed = 0.0
shared_angular_override = None
shared_frame_time = None
is_running = True
control_failure = None

state = State.LANE_FOLLOWING
state_enter_time = time.monotonic()
avoid_direction = AvoidDirection.LEFT
front_obstacle_count = 0
recovery_detect_count = 0
avoid_count = 0
recovery_done_count = 0


def set_command(speed, angular_override, error=None):
    global shared_speed, shared_angular_override, shared_error
    with error_lock:
        shared_speed = float(speed)
        shared_angular_override = None if angular_override is None else float(angular_override)
        if error is not None:
            shared_error = float(error)

def force_stop():
    set_command(0.0, 0.0)

def transition(new_state):
    global state, state_enter_time
    state = new_state
    state_enter_time = time.monotonic()
    print(f"\n[FSM TRANSITION] -> {state.name}", flush=True)

def state_elapsed():
    return time.monotonic() - state_enter_time


# ============================================================
# Vision & Pipeline Logic
# ============================================================
def gstreamer_pipeline(sensor_id=0, capture_width=1280, capture_height=720, display_width=640, display_height=360, framerate=30, flip_method=0):
    return (f"nvarguscamerasrc sensor-id={sensor_id} ! video/x-raw(memory:NVMM), width=(int){capture_width}, height=(int){capture_height}, framerate=(fraction){framerate}/1 ! "
            f"nvvidconv flip-method={flip_method} ! video/x-raw, width=(int){display_width}, height=(int){display_height}, format=(string)BGRx ! "
            f"videoconvert ! video/x-raw, format=(string)BGR ! appsink")

def process_lane_frame(frame, lane_tracker):
    raw_edges, edges = lane_edges(frame)
    pair, lane_status = lane_tracker.update(edges, time.monotonic(), frame=frame)
    result_image, error, is_detected = draw_result(frame.copy(), pair, lane_status)
    return error, result_image, is_detected, pair, raw_edges, edges

def get_obstacle_direction_by_contour(frame, pair):
    if pair is None or len(pair) < 2: return AvoidDirection.LEFT
    height, width = frame.shape[:2]
    y_eval_norm = (0.52 + 0.84) / 2.0
    x_left = np.polyval(pair[0], y_eval_norm)
    x_right = np.polyval(pair[1], y_eval_norm)
    lane_center = (x_left + x_right) / 2.0
    margin_px = int((x_right - x_left) * 0.10)
    y1, y2 = int(height * 0.52), int(height * 0.84)
    x1, x2 = max(0, int(x_left + margin_px)), min(width, int(x_right - margin_px))
    if x2 <= x1 or y2 <= y1: return AvoidDirection.LEFT

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
        bx, _, bw, _ = cv2.boundingRect(largest_contour)
        if (x1 + bx + bw / 2.0) >= lane_center: return AvoidDirection.LEFT
        return AvoidDirection.RIGHT
    return AvoidDirection.LEFT


# ============================================================
# Ultrasonic Reader & Control Thread
# ============================================================
def create_ultrasonic_reader():
    reader = UltrasonicReader(port=ULTRASONIC_PORT, baudrate=SERIAL_BAUD)
    if not reader.start():
        raise RuntimeError("아두이노 초음파 센서 시리얼 연결 실패")
    return reader

def read_distances(reader):
    values = reader.get_distances()
    if not isinstance(values, (tuple, list)) or len(values) != 3:
        raise ValueError("get_distances() 오류")
    return tuple((float(v) if math.isfinite(float(v)) else NO_OBSTACLE) for v in values)

def apply_ramp(current_val, target_val, max_increase, max_decrease, dt):
    if target_val > current_val:
        return min(target_val, current_val + max_increase * dt)
    elif target_val < current_val:
        return max(target_val, current_val - max_decrease * dt)
    return target_val


def control_thread_task(ugv_serial):
    global is_running, control_failure
    previous_error = previous_frame_time = None
    target_angular = 0.0
    ramped_speed = 0.0
    ramped_angular = 0.0
    last_loop_time = time.monotonic()

    print("[알림] 제어 스레드 시작 (가감속 램핑 및 통신 에러 방어)", flush=True)
    try:
        while is_running:
            now = time.monotonic()
            dt = max(0.001, min(0.1, now - last_loop_time))
            last_loop_time = now

            with error_lock:
                c_err, c_spd, override, f_time = shared_error, shared_speed, shared_angular_override, shared_frame_time

            stale = (f_time is None or now - f_time > FRAME_TIMEOUT)

            if turn_watchdog_reason(now):
                latch_safety_fault(turn_watchdog_reason(now))

            if safety_fault is not None or stale:
                target_speed = target_angular = 0.0
                previous_error = previous_frame_time = None
                ramped_speed = ramped_angular = 0.0
            else:
                if override is not None:
                    target_speed, target_angular = c_spd, override
                    previous_error = previous_frame_time = None
                elif c_spd <= 0:
                    target_speed = target_angular = 0.0
                    previous_error = previous_frame_time = None
                elif previous_frame_time is None or f_time != previous_frame_time:
                    pid_dt = (f_time - previous_frame_time) if previous_frame_time else 1.0
                    target_speed = c_spd
                    target_angular = ugv_control.calculate_pid(c_err, previous_error, pid_dt)
                    previous_error, previous_frame_time = c_err, f_time
                else:
                    target_speed = c_spd

                # ★ 오버슈트 방지 로직 적용
                ramped_speed = apply_ramp(ramped_speed, target_speed, MAX_LINEAR_ACCEL, MAX_LINEAR_DECEL, dt)
                
                if target_angular == 0.0:
                    ramped_angular = 0.0  # 정지 명령 시 즉시 각속도 0
                else:
                    ramped_angular = apply_ramp(ramped_angular, target_angular, MAX_ANGULAR_ACCEL, MAX_ANGULAR_ACCEL, dt)

            # Write Timeout 무시 (전압 강하 방어)
            try:
                ugv_control.send_driving_command(ugv_serial, ramped_speed, ramped_angular)
            except Exception:
                pass

            time.sleep(CONTROL_PERIOD_S)

    except Exception as exc:
        control_failure = f"{type(exc).__name__}: {exc}"
        is_running = False
    finally:
        try:
            ugv_control.send_driving_command(ugv_serial, 0.0, 0.0)
        except Exception:
            pass


# ============================================================
# FSM Logic
# ============================================================
def check_recovery_lane_detection(is_detected):
    global recovery_detect_count
    if is_detected:
        recovery_detect_count += 1
        if recovery_detect_count >= 3:
            recovery_detect_count = 0
            force_stop()
            print("[RECOVERY COMPLETE] 차선 포착 완료 -> LANE_FOLLOWING 복귀", flush=True)
            transition(State.LANE_FOLLOWING)
            return True
    else:
        recovery_detect_count = 0
    return False

def run_fsm(error, is_detected, frame, pair, left_d, front_d, right_d):
    global avoid_direction, state, front_obstacle_count
    if safety_fault is not None:
        force_stop()
        return

    # 1. 차선 추종
    if state == State.LANE_FOLLOWING:
        if (0.0 < front_d <= FRONT_STOP_CM) or (front_obstacle_count > 0 and front_d < 0):
            front_obstacle_count += 1
            force_stop()
        else:
            front_obstacle_count = max(0, front_obstacle_count - 1)

        if front_obstacle_count >= 2:
            front_obstacle_count = 0
            force_stop()
            vision_dir = get_obstacle_direction_by_contour(frame, pair)
            if 25.0 < left_d and 25.0 < right_d:
                avoid_direction = AvoidDirection.LEFT if left_d > right_d + 15.0 else (AvoidDirection.RIGHT if right_d > left_d + 15.0 else vision_dir)
            else:
                avoid_direction = vision_dir

            print(f"\n[OBSTACLE DETECTED] Front: {front_d:.1f}cm -> Avoid Dir: {avoid_direction.name}", flush=True)
            transition(State.AVOID_START_PAUSE)
            return

        elif not is_detected: force_stop()
        elif front_obstacle_count == 0: set_command(BASE_SPEED, None, error=error)
        return

    # 1-1. 대기 및 기준 각도 저장
    if state == State.AVOID_START_PAUSE:
        force_stop()
        if state_elapsed() >= 0.6:
            yaw, stamp, _ = get_imu_snapshot()
            if stamp is not None and (time.monotonic() - stamp) <= IMU_STALE_TIMEOUT_S:
                run_fsm.initial_yaw = yaw
                print(f"[기준 각도 저장] initial_yaw = {yaw:.2f} deg", flush=True)
                transition(State.AVOID_TURN_OUT)
            else:
                latch_safety_fault("기준 Yaw 저장 실패")
        return

    # 2. 회전
    if state == State.AVOID_TURN_OUT:
        run_imu_turn(avoid_direction.value, State.AVOID_PAUSE)
        return

    # 2-1. 대기 및 대각선 각도 저장
    if state == State.AVOID_PAUSE:
        force_stop()
        if state_elapsed() >= 0.3:
            yaw, _, _ = get_imu_snapshot()
            run_fsm.diagonal_yaw = yaw
            print(f"[대각선 각도 저장] diagonal_yaw = {yaw:.2f} deg", flush=True)
            transition(State.AVOID_DIAGONAL)
        return

    # 3. 대각선 전진 (마진 완화 및 강력한 P제어 적용)
    if state == State.AVOID_DIAGONAL:
        extra_time = 0.50 / AVOID_SPEED
        is_crossing_lane = False
        if is_detected and pair is not None and len(pair) >= 2:
            height, width = frame.shape[:2]
            if avoid_direction == AvoidDirection.LEFT and np.polyval(pair[0], 0.9) >= width * 0.70: is_crossing_lane = True
            elif avoid_direction == AvoidDirection.RIGHT and np.polyval(pair[1], 0.9) <= width * 0.30: is_crossing_lane = True

        if is_crossing_lane or state_elapsed() >= extra_time:
            force_stop()
            print("[대각선 완료] 마진 또는 거리 도달 -> 정렬 시작", flush=True)
            transition(State.AVOID_ALIGN)
            return

        # P게인 보정치 상향 (0.6) - 매트 마찰 억제
        yaw, _, _ = get_imu_snapshot()
        diff = normalize_delta(getattr(run_fsm, 'diagonal_yaw', yaw) - yaw)
        set_command(AVOID_SPEED, max(-0.6, min(0.6, TURN_KP * diff)))
        return

    # 4. 제자리 정렬 (카메라 퓨전 조기 종료 + 속도 상향)
    if state == State.AVOID_ALIGN:
        now = time.monotonic()
        yaw, stamp, _ = get_imu_snapshot()

        if stamp is None or (now - stamp) > IMU_STALE_TIMEOUT_S:
            latch_safety_fault("정렬 회전 중 IMU 끊김")
            return

        # ★ 8.0초 타임아웃으로 연장
        if state_elapsed() >= 8.0:
            force_stop()
            print("[ALIGN TIMEOUT] 정렬 타임아웃 -> 강제 직진", flush=True)
            transition(State.AVOID_FORWARD)
            return
            
        # ★ 카메라-IMU 퓨전 (정면 차선 보이면 0.5초 대기 후 즉시 완료)
        if is_detected and abs(error) < 15.0 and state_elapsed() > 0.5:
            force_stop()
            print(f"[카메라 정렬 완료] 차선 포착됨 (Error: {error:.1f}) -> 직진 전환", flush=True)
            transition(State.AVOID_FORWARD)
            return

        target_yaw = getattr(run_fsm, 'initial_yaw', yaw)
        diff = normalize_delta(target_yaw - yaw)

        if abs(diff) <= TURN_TOLERANCE_DEG:
            force_stop()
            print(f"[ALIGN DONE] 목표:{target_yaw:.1f} / 현재:{yaw:.1f}", flush=True)
            transition(State.AVOID_FORWARD)
            return

        # ★ 매트 마찰 극복을 위한 속도 상향 (0.60 ~ 0.75)
        turn_dir = 1.0 if diff > 0 else -1.0
        angular = min(0.75, max(0.60, TURN_KP * abs(diff)))
        set_command(0.0, turn_dir * angular)
        return

    # 5. 직진 통과
    if state == State.AVOID_FORWARD:
        set_command(PASS_SPEED, 0.0)
        if state_elapsed() >= (0.70 / PASS_SPEED):
            side_d = right_d if avoid_direction == AvoidDirection.LEFT else left_d
            if side_d >= 60.0 or side_d < 0:
                force_stop()
                transition(State.RECOVERY_TURN_IN)
        return

    # 6. 복귀
    if state == State.RECOVERY_TURN_IN:
        run_imu_turn(-avoid_direction.value, State.RECOVERY_DIAGONAL)
        return

    if state == State.RECOVERY_DIAGONAL:
        if check_recovery_lane_detection(is_detected): return
        set_command(PASS_SPEED, 0.0)
        if state_elapsed() >= 6.0:
            force_stop()
            transition(State.LANE_FOLLOWING)
        return


# ============================================================
# Main Loop
# ============================================================
def main():
    global is_running, shared_frame_time, control_failure
    ugv_serial = ultrasonic = cap = control_thread = imu_thread = None

    try:
        ugv_serial = ugv_control.init_ugv(port=UGV_PORT, baudrate=SERIAL_BAUD)
        ugv_control.send_driving_command(ugv_serial, 0.0, 0.0)
        ugv_serial.write(b'{"T":131, "cmd":1}\n')

        imu_thread = threading.Thread(target=ugv_imu_read_task, args=(ugv_serial,), daemon=True)
        imu_thread.start()

        ultrasonic = create_ultrasonic_reader()
        time.sleep(0.5)

        cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)

        if SHOW_WINDOWS:
            cv2.namedWindow(LANE_WINDOW, cv2.WINDOW_NORMAL)
            cv2.namedWindow(CANNY_WINDOW, cv2.WINDOW_NORMAL)
            print("[화면] 차선 인식과 Canny 표시. Q 또는 Esc로 종료.")

        control_thread = threading.Thread(target=control_thread_task, args=(ugv_serial,), daemon=True)
        control_thread.start()

        lane_tracker = LaneTracker()
        failure_count = 0
        report_time = time.monotonic()

        while is_running and cap.isOpened():
            frame_started = time.monotonic()
            ret, frame = cap.read()

            if not ret or frame is None or frame.size == 0:
                force_stop()
                failure_count += 1
                if failure_count >= MAX_CAMERA_FAILURES: break
                time.sleep(0.05)
                continue

            failure_count = 0
            error, result_image, is_detected, pair, raw_edges, lane_edges_img = process_lane_frame(frame, lane_tracker)
            left_d, front_d, right_d = read_distances(ultrasonic)

            with error_lock:
                if not is_running: break
                shared_frame_time = frame_started

            run_fsm(error, is_detected, frame, pair, left_d, front_d, right_d)

            now = time.monotonic()
            if now - report_time >= TERMINAL_LOG_INTERVAL_S:
                print(f"[MONITOR] State: {state.name:<18} | US: {left_d:4.0f}/{front_d:4.0f}/{right_d:4.0f} | Yaw: {shared_yaw:5.1f} | Err: {error:+.1f}", flush=True)
                report_time = now

            if SHOW_WINDOWS:
                canny_view = np.hstack((raw_edges, lane_edges_img))
                cv2.imshow(LANE_WINDOW, result_image)
                cv2.imshow(CANNY_WINDOW, canny_view)
                if (cv2.waitKey(1) & 0xFF) in (ord("q"), ord("Q"), 27): break

        if control_failure: print(f"제어 오류: {control_failure}")

    except KeyboardInterrupt: print("\n[STOP] Keyboard Interrupt detected.")
    except Exception: traceback.print_exc()
    finally:
        is_running = False
        if control_thread: control_thread.join(timeout=1.0)
        if imu_thread: imu_thread.join(timeout=1.0)
        if ugv_serial:
            for _ in range(3):
                try:
                    ugv_control.send_driving_command(ugv_serial, 0.0, 0.0)
                    time.sleep(0.05)
                except: pass
            ugv_control.stop_ugv(ugv_serial)
        if ultrasonic: ultrasonic.stop()
        if cap: cap.release()
        if SHOW_WINDOWS: cv2.destroyAllWindows()


if __name__ == "__main__":
    main()