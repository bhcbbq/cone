# main_highway_contour_avoid_final_hybrid_optimized.py
import cv2
import time
import threading
import traceback
import math
import numpy as np
import os
from enum import Enum, auto

import lane_detection_ver2 as lane_detection
import ugv_control
from ultrasonic_reader import UltrasonicReader

import json

# IMU yaw unit MUST match the firmware output: "deg" or "rad".
# Original program assumed degrees; retained as default (do not infer from a still image).
IMU_YAW_UNIT = "rad"
TURN_ANGLE_DEG = 30.0
TURN_TOLERANCE_DEG = 2.0
TURN_KP = 0.035
TURN_MIN_SPEED = 0.18
TURN_MAX_SPEED = 0.65   # command units assumed rad/s; verify ugv_control/firmware
TURN_TIMEOUT_S = 6.0
IMU_STALE_TIMEOUT_S = 0.35
IMU_NO_MOTION_TIMEOUT_S = 1.5
IMU_MIN_MOTION_DEG = 1.0
IMU_MAX_STEP_DEG = 45.0  # reject discontinuous/reset samples while turning

imu_lock = threading.Lock()
shared_yaw = 0.0       # always degrees after input conversion
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
                # Keep only the most recent valid complete packet.
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
            # Missing/stale feedback is stopped by the independent control watchdog.
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
    """Accumulate signed wrapped deltas; stop on reaching OR crossing 50 deg.

    Uses absolute NET rotation, so either yaw sign convention is supported.
    This does NOT verify that a positive motor command physically turns left:
    validate AvoidDirection/firmware motor polarity separately before operation.
    """
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
        print(f"[TURN START] yaw_deg={yaw:.2f}, target={TURN_ANGLE_DEG:.1f}, "
              f"input_unit={IMU_YAW_UNIT}, command_sign={command_direction}", flush=True)
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
AVOID_SPEED = 0.15            # m/s 
PASS_SPEED = 0.40             # m/s (대각선 및 직진 통과)

# 거리 기반 회피 조건 파라미터
FRONT_STOP_CM = 70.0         # 회피 시작 기준 거리 (전방 장애물 인식)
FRONT_SAFE_CM = 70.0          # 회전 시 전방이 70cm 이상 확보되어야 회전 완료
SIDE_DANGER_CM = 30.0         # 측면 30cm 미만일 경우 너무 가까워 임시 정지
SIDE_CLEAR_CM = 50.0          # 측면 50cm 이상 멀어지면 장애물을 통과했다고 판단X

ALIGN_TIME_S = 1.6
TURN_ANGULAR_SPEED = 2.6     # 제자리 회전 각속도

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
    AVOID_START_PAUSE = auto() # 장애물 발견 직후 브레이크(급정거) 상태  
    AVOID_TURN_OUT = auto()    
    AVOID_PAUSE = auto()       # 회전 후 센서값 안정화를 위해 잠시 멈추는 상태
    AVOID_DIAGONAL = auto()    
    AVOID_ALIGN = auto()       
    AVOID_FORWARD = auto()     
    RECOVERY_TURN_IN = auto()  
    RECOVERY_DIAGONAL = auto() 

class AvoidDirection(Enum):
    LEFT = 1     
    RIGHT = -1   

# ===========================================================
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

# main_final.py와 동일한 색상 필터 및 차선 ROI.
def lane_edges(frame):
    """전체 Canny 영상과 차선 검출용 에지를 반환합니다."""
    height, width = frame.shape[:2]

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    raw_edges = cv2.Canny(blurred, 50, 150)

    hls = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    white = cv2.inRange(
        hls,
        (0, 150, 0),
        (180, 255, 150),
    )

    yellow = cv2.inRange(
        hsv,
        (15, 65, 70),
        (40, 255, 255),
    )

    colors = cv2.dilate(
        white | yellow,
        np.ones((5, 5), np.uint8),
    )

    edges = cv2.bitwise_and(raw_edges, colors)
    mask = np.zeros_like(edges)

    vertices = np.array(
        [
            [
                (int(width * 0.05), int(height * 0.96)),
                (int(width * 0.25), int(height * 0.40)),
                (int(width * 0.75), int(height * 0.40)),
                (int(width * 0.95), int(height * 0.96)),
            ]
        ],
        dtype=np.int32,
    )

    cv2.fillPoly(mask, vertices, 255)
    lane_edge_image = cv2.bitwise_and(edges, mask)

    return raw_edges, lane_edge_image


def process_lane_frame(frame, tracker, now):
    # main_final과 같은 카메라 프레임을 리사이즈 없이 처리한다.
    raw_edges, edges = lane_edges(frame)
    pair, status = tracker.update(edges, now)
    result_image, error, is_detected = lane_detection.draw_result(frame.copy(), pair, status)

    # 회피 함수가 사용하는 x(y_norm) 형식으로만 변환한다.
    # 원본 프레임에서 검출했으므로 가로 좌표를 다시 확대하지 않는다.
    avoidance_pair = None
    if pair is not None:
        y_top, y_bottom = 0.76, 0.94
        slope = (pair[:, 1] - pair[:, 0]) / (y_bottom - y_top)
        intercept = pair[:, 0] - slope * y_top
        avoidance_pair = np.column_stack((slope, intercept))
    return error, result_image, is_detected, avoidance_pair, raw_edges, edges

def get_obstacle_direction_by_contour(frame, pair):
    if pair is None or len(pair) < 2:
        return AvoidDirection.LEFT
    height, width = frame.shape[:2]
    y_eval_norm = (0.52 + 0.84) / 2.0
    x_left = np.polyval(pair[0], y_eval_norm)
    x_right = np.polyval(pair[1], y_eval_norm)
    lane_center = (x_left + x_right) / 2.0
    margin_px = int((x_right - x_left) * 0.10)
    
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
        bx, _, bw, _ = cv2.boundingRect(largest_contour)
        if (x1 + bx + bw / 2.0) >= lane_center:
            return AvoidDirection.LEFT
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
        raise ValueError("get_distances() 형태 오류")
    result = []
    for value in values:
        try:
            d = float(value)
            result.append(d if math.isfinite(d) else NO_OBSTACLE)
        except (TypeError, ValueError):
            result.append(NO_OBSTACLE)
    return tuple(result)

def control_thread_task(ugv_serial):
    global is_running, control_failure
    previous_error = previous_frame_time = None
    angular_z = 0.0

    print("[알림] 제어 스레드 시작", flush=True)
    try:
        while is_running:
            # ugv_control.clear_feedback_buffer(ugv_serial)  # <--- 버퍼 삭제 방지 주석 처리
            now = time.monotonic()

            with error_lock:
                c_err, c_spd, override, f_time = shared_error, shared_speed, shared_angular_override, shared_frame_time

            stale = (f_time is None or now - f_time > FRAME_TIMEOUT)

            reason = turn_watchdog_reason(now)
            if reason:
                latch_safety_fault(reason)
            # Camera stalls must stop overrides too (original code kept turning).
            if safety_fault is not None or stale:
                c_spd = angular_z = 0.0
                previous_error = previous_frame_time = None
            elif override is not None:
                angular_z = override
                previous_error = previous_frame_time = None
            elif c_spd <= 0:
                c_spd = angular_z = 0.0
                previous_error = previous_frame_time = None
            elif previous_frame_time is None or f_time != previous_frame_time:
                dt = (f_time - previous_frame_time) if previous_frame_time else 1.0
                angular_z = ugv_control.calculate_pid(c_err, previous_error, dt)
                previous_error, previous_frame_time = c_err, f_time

            ugv_control.send_driving_command(ugv_serial, c_spd, angular_z)
            time.sleep(CONTROL_PERIOD_S)
    except Exception as exc:
        control_failure = f"{type(exc).__name__}: {exc}"
        print(f"[에러] 제어 스레드 중단: {control_failure}", flush=True)
        is_running = False
    finally:
        try:
            ugv_control.send_driving_command(ugv_serial, 0.0, 0.0)
        except:
            pass

# ============================================================
# FSM Logic (수정된 핵심 알고리즘 적용 구역)
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
    global avoid_direction, state, front_obstacle_count, avoid_count, recovery_done_count
    if safety_fault is not None:
        force_stop()
        return

    # 1. 차선 추종 (유지)
    if state == State.LANE_FOLLOWING:
        is_front_close = (0.0 < front_d <= FRONT_STOP_CM) or (front_obstacle_count > 0 and front_d < 0)

        if is_front_close:
            front_obstacle_count += 1
            force_stop()
        else:
            if front_obstacle_count > 0:
                front_obstacle_count -= 1

        if front_obstacle_count >= 2:
            front_obstacle_count = avoid_count = recovery_done_count = 0
            force_stop()
            
            vision_dir = get_obstacle_direction_by_contour(frame, pair)
            if 25.0 < left_d and 25.0 < right_d:
                if left_d > right_d + 15.0:  
                    avoid_direction = AvoidDirection.LEFT
                elif right_d > left_d + 15.0:
                    avoid_direction = AvoidDirection.RIGHT
                else:
                    avoid_direction = vision_dir
            else:
                avoid_direction = vision_dir
                
            print(f"\n[OBSTACLE DETECTED] Front: {front_d:.1f}cm -> Avoid Dir: {avoid_direction.name}", flush=True)
            transition(State.AVOID_START_PAUSE)
            return

        elif not is_detected:
            force_stop()
        elif front_obstacle_count == 0:
            set_command(BASE_SPEED, None, error=error)
        return

    # 1-1. 장애물 발견 직후 완전 정지 (유지)
    if state == State.AVOID_START_PAUSE:
        force_stop()
        if state_elapsed() >= 0.6: 
            transition(State.AVOID_TURN_OUT)
        return

    # 2. 제자리 회전 (초음파 조건 삭제, IMU 50도 정밀 회전)
    if state == State.AVOID_TURN_OUT:
        run_imu_turn(avoid_direction.value, State.AVOID_PAUSE)
        return

    # 2-1. 회전 후 잠시 대기 (유지)
    if state == State.AVOID_PAUSE:
        force_stop()
        if state_elapsed() >= 0.3:
            transition(State.AVOID_DIAGONAL)
        return

    # 3. 대각선 전진 (통과 후 10cm 여유 마진 주행 추가)
    if state == State.AVOID_DIAGONAL:
        side_d = right_d if avoid_direction == AvoidDirection.LEFT else left_d
        
        # 10cm 여유 마진 타이머용 내부 변수 초기화
        if not hasattr(run_fsm, 'clear_time'):
            run_fsm.clear_time = None

        if run_fsm.clear_time is None:
            if 0.0 < side_d < SIDE_DANGER_CM:
                force_stop()
                print(f"[위험] 측면 장애물 초근접! ({side_d:.1f}cm) -> 충돌 방지 대기", flush=True)
            elif side_d >= SIDE_CLEAR_CM or side_d < 0:
                # 장애물을 벗어났을 때 (또는 센서 범위를 벗어나 -1이 뜰 때) 10cm 카운트 시작
                run_fsm.clear_time = time.monotonic()
                print("[통과] 측면 클리어 -> 추가 10cm 마진 전진 시작", flush=True)
            else:
                set_command(AVOID_SPEED, 0.0)
        else:
            set_command(AVOID_SPEED, 0.0)
            # AVOID_SPEED 속도로 10cm(0.1m)를 가는 데 걸리는 시간 계산
            extra_time = 0.10 / AVOID_SPEED 
            
            if time.monotonic() - run_fsm.clear_time >= extra_time:
                force_stop()
                run_fsm.clear_time = None # 변수 초기화
                transition(State.AVOID_ALIGN)
        return

    # 4. 제자리 정렬 (초기 회전의 반대 방향으로 IMU 50도 역회전)
    if state == State.AVOID_ALIGN:
        run_imu_turn(-avoid_direction.value, State.AVOID_FORWARD)
        return

    # 5. 직진 통과 (정확히 40cm 이동)
    if state == State.AVOID_FORWARD:
        set_command(PASS_SPEED, 0.0)
        # PASS_SPEED 속도로 65cm(0.65m)를 가는 데 걸리는 시간 계산
        pass_time = 0.65 / PASS_SPEED
        
        if state_elapsed() >= pass_time:
            force_stop()
            transition(State.RECOVERY_TURN_IN)
        return

    # 6. 복귀 기동: 시간 기반 회전 대신 동일한 IMU 50도 제한 사용
    if state == State.RECOVERY_TURN_IN:
        if check_recovery_lane_detection(is_detected):
            reset_turn()
            return
        run_imu_turn(-avoid_direction.value, State.RECOVERY_DIAGONAL)
        return

    if state == State.RECOVERY_DIAGONAL:
        if check_recovery_lane_detection(is_detected): return
        set_command(PASS_SPEED, 0.0)
        
        if state_elapsed() >= 6.0:
            force_stop()
            print("[에러] 타임아웃: 차선을 찾지 못했습니다.", flush=True)
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

        # --- [추가된 부분: IMU 연속 피드백 켜기 및 읽기 스레드 시작] ---
        ugv_serial.write(b'{"T":131, "cmd":1}\n')
        
        imu_thread = threading.Thread(target=ugv_imu_read_task, args=(ugv_serial,), daemon=True)
        imu_thread.start()
        # -------------------------------------------------------------
        
        ultrasonic = create_ultrasonic_reader()
        time.sleep(0.5)

        cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
        tracker = lane_detection.LaneTracker()

        if SHOW_WINDOWS:
            cv2.namedWindow(LANE_WINDOW, cv2.WINDOW_NORMAL)
            cv2.namedWindow(CANNY_WINDOW, cv2.WINDOW_NORMAL)
            print("[화면] 차선 인식과 Canny 표시. Q 또는 Esc로 종료.")
        else:
            print("[화면] SSH 또는 디스플레이 없는 환경: 화면 출력 끔.")

        control_thread = threading.Thread(target=control_thread_task, args=(ugv_serial,), daemon=True)
        control_thread.start()
        
        failure_count = 0
        report_time = time.monotonic()

        while is_running and cap.isOpened():
            frame_started = time.monotonic()
            ret, frame = cap.read()

            if not ret or frame is None or frame.size == 0:
                force_stop()
                tracker.reset()
                failure_count += 1
                if failure_count >= MAX_CAMERA_FAILURES: break
                time.sleep(0.05)
                continue

            failure_count = 0
            error, result_image, is_detected, pair, raw_edges, lane_edges = process_lane_frame(
                frame, tracker, frame_started
            )
            left_d, front_d, right_d = read_distances(ultrasonic)

            with error_lock:
                if not is_running: break
                shared_frame_time = frame_started

            run_fsm(error, is_detected, frame, pair, left_d, front_d, right_d)

            now = time.monotonic()
            if now - report_time >= TERMINAL_LOG_INTERVAL_S:
                usable = (bool(is_detected) and now - frame_started <= FRAME_TIMEOUT)
                print(f"[MONITOR] State: {state.name:<18} | US Front: {front_d:5.1f}cm | US Side: {left_d:5.1f} / {right_d:5.1f} | Yaw: {shared_yaw:5.1f} | Error: {error:+.1f}", flush=True)
                report_time = now

            if SHOW_WINDOWS:
                # 왼쪽: 전체 Canny / 오른쪽: 색상 필터와 ROI 적용 후 차선 검출용 에지
                canny_view = np.hstack((raw_edges, lane_edges))
                cv2.imshow(LANE_WINDOW, result_image)
                cv2.imshow(CANNY_WINDOW, canny_view)

                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), ord("Q"), 27):
                    break
                    
        if control_failure: print(f"제어 오류: {control_failure}")

    except KeyboardInterrupt:
        print("\n[STOP] Keyboard Interrupt detected.")
    except Exception:
        traceback.print_exc()
    
    finally:
        is_running = False
        print("[종료] 백그라운드 제어 중지 및 모터 강제 정지 중...")
        
        if control_thread is not None:
            control_thread.join(timeout=1.0)
            
        if imu_thread is not None:
            imu_thread.join(timeout=1.0)

        if ugv_serial is not None:
            for _ in range(3):
                try:
                    ugv_control.send_driving_command(ugv_serial, 0.0, 0.0)
                    time.sleep(0.05)
                except:
                    pass
            ugv_control.stop_ugv(ugv_serial)
            
        if ultrasonic is not None:
            ultrasonic.stop()
        if cap is not None:
            cap.release()
        if SHOW_WINDOWS:
            cv2.destroyAllWindows()

if __name__ == "__main__":
    main()