# 최신 수정:
# 1. 대각선 전진(AVOID_DIAGONAL) 시 차선 넘음 감지 알고리즘 제거 (고정 거리/시간 이동)
# 2. 모든 제자리 회전(AVOID_TURN_OUT, AVOID_ALIGN, RECOVERY_TURN_IN)을 light_control2 방식의
#    다이렉트 좌/우 바퀴 제어({"T":1,"L":...,"R":...}) 및 IMU Yaw 누적/목표각 제어로 변경
# 3. SerialTimeoutException(Write timeout) 발생 시 제어 스레드가 중단되지 않도록 try-except 방어 적용
# 4. 복귀 회전 각도(RECOVERY_TURN_ANGLE_DEG) 별도 분리 및 AVOID_FORWARD 들여쓰기 오류 수정
# 5. AVOID_ALIGN 초기 오버슈트 저감: 전진 후 정지 대기(AVOID_DIAGONAL_PAUSE), 선형 감속,
#    목표각 통과 즉시 제동, 회전 중 차선 인식 생략(IMU 응답 속도 향상) 적용
# 6. 프로그램 시작 시점의 직선 도로 정면 Yaw(track_front_yaw)를 확정하여 정렬 기준각(initial_yaw)으로 적용
# 7. 라인 트레이싱(GStreamer appsink, BASE_SPEED, frame_seq/frame_time 동기화, PID 루프 및 차선 미검출 정지)을
#    light_control2.py 방식으로 교체
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

# ============================================================
# IMU & Direct Wheel Turn Settings (light_control2 방식 반영)
# ============================================================
IMU_YAW_UNIT = "rad"
TURN_ANGLE_DEG = 20.0           # 회피 회전 목표 각도 (도)
RECOVERY_TURN_ANGLE_DEG = 8.0   # 복귀 회전 목표 각도 (도) - 원하는 각도로 조절
TURN_TOLERANCE_DEG = 2.0        # 허용 오차 (도)

# light_control2.py 스타일의 다이렉트 바퀴 속도 (L, R 직접 인가)
DIRECT_TURN_SPEED = 0.15        # 기본 제자리 회전 바퀴 속도
DIRECT_TURN_SLOW_SPEED = 0.11   # 목표각 근접 시 감속 바퀴 속도
DIRECT_TURN_SLOW_REMAIN_DEG = 30

# 정렬 회전(AVOID_ALIGN) 전용 파라미터 (오버슈트 방지)
ALIGN_MAX_SPEED = 0.11          # 정렬 최대 바퀴 속도
ALIGN_MIN_SPEED = 0.065         # 목표각 직전 최소 바퀴 속도 (마찰로 안 돌면 소폭 상향)
ALIGN_SLOWDOWN_DEG = 25.0       # 이 각도 이내부터 선형 감속
ALIGN_TOLERANCE_DEG = 3.5       # 정렬 허용 오차 (도)
ALIGN_SETTLE_S = 0.25           # 목표각 통과 시 관성/IMU 안정화 정지 시간(초)

TURN_TIMEOUT_S = 6.0
IMU_STALE_TIMEOUT_S = 0.35
IMU_NO_MOTION_TIMEOUT_S = 3.0
IMU_MIN_MOTION_DEG = 1.0
IMU_MAX_STEP_DEG = 45.0

imu_lock = threading.Lock()
ugv_write_lock = threading.Lock()

shared_yaw = 0.0
shared_yaw_time = None
shared_yaw_seq = 0
imu_read_error = None
turn_context = None
safety_fault = None
track_front_yaw = None          # 출발 시 저장되는 직선 도로 정면 기준각


def normalize_delta(degrees):
    return (degrees + 180.0) % 360.0 - 180.0


def get_imu_snapshot():
    with imu_lock:
        return shared_yaw, shared_yaw_time, shared_yaw_seq


def send_raw_ugv(ugv_serial, message):
    """light_control2.py와 동일하게 UGV에 다이렉트 JSON 명령 전송 (타임아웃 예외 방어 포함)"""
    if ugv_serial is None:
        return
    if not message.endswith("\n"):
        message += "\n"
    try:
        with ugv_write_lock:
            ugv_serial.write(message.encode("utf-8"))
            ugv_serial.flush()
    except Exception as exc:
        print(f"[경고] UGV 다이렉트 시리얼 전송 지연 무시: {type(exc).__name__}: {exc}", flush=True)


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
                        if not isinstance(packet, dict):
                            continue
                        raw_yaw = packet.get("yaw", packet.get("y"))
                        if raw_yaw is None:
                            continue
                        yaw = float(raw_yaw)
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


def compute_direct_wheel_turn(direction_sign, remaining_deg):
    """
    light_control2.py의 제자리 회전 부호 규칙 적용:
    LEFT (+1): L = -speed, R = +speed
    RIGHT (-1): L = +speed, R = -speed
    """
    speed = DIRECT_TURN_SLOW_SPEED if remaining_deg <= DIRECT_TURN_SLOW_REMAIN_DEG else DIRECT_TURN_SPEED
    if direction_sign > 0:
        return -speed, speed
    else:
        return speed, -speed


def compute_align_wheel_turn(direction_sign, remaining_deg):
    """목표각에 가까워질수록 선형 감속하는 정렬 전용 바퀴 속도 계산"""
    if remaining_deg <= ALIGN_SLOWDOWN_DEG:
        ratio = max(0.0, remaining_deg / ALIGN_SLOWDOWN_DEG)
        speed = ALIGN_MIN_SPEED + (ALIGN_MAX_SPEED - ALIGN_MIN_SPEED) * ratio
    else:
        speed = ALIGN_MAX_SPEED

    if direction_sign > 0:
        return -speed, speed
    else:
        return speed, -speed


def run_direct_imu_turn(command_direction, next_state, target_angle_deg=TURN_ANGLE_DEG):
    """IMU 변화량 누적 기반 다이렉트 바퀴 회전"""
    global turn_context
    now = time.monotonic()
    yaw, stamp, seq = get_imu_snapshot()
    if stamp is None or now - stamp > IMU_STALE_TIMEOUT_S:
        latch_safety_fault("Cannot start/continue turn without fresh IMU")
        return

    if turn_context is None:
        turn_context = {
            "started": now,
            "last_yaw": yaw,
            "last_seq": seq,
            "accumulated": 0.0,
            "motion_anchor": 0.0,
            "last_motion": now,
            "direction": command_direction,
        }
        print(f"[DIRECT TURN START] yaw={yaw:.2f} deg, target={target_angle_deg:.1f} deg, dir={command_direction}", flush=True)

    ctx = turn_context
    if ctx["direction"] != command_direction:
        latch_safety_fault("Unexpected turn direction change")
        return

    if seq != ctx["last_seq"]:
        delta = normalize_delta(yaw - ctx["last_yaw"])
        if abs(delta) > IMU_MAX_STEP_DEG:
            latch_safety_fault(f"IMU discontinuity: {delta:.1f} deg/sample")
            return
        ctx["accumulated"] += abs(delta)
        ctx["last_yaw"], ctx["last_seq"] = yaw, seq
        if abs(ctx["accumulated"] - ctx["motion_anchor"]) >= IMU_MIN_MOTION_DEG:
            ctx["last_motion"] = now
            ctx["motion_anchor"] = ctx["accumulated"]

    remaining = max(0.0, target_angle_deg - ctx["accumulated"])
    if remaining <= TURN_TOLERANCE_DEG:
        force_stop()
        print(f"[DIRECT TURN DONE] accumulated={ctx['accumulated']:.2f} deg", flush=True)
        reset_turn()
        transition(next_state)
        return

    reason = turn_watchdog_reason(now)
    if reason:
        latch_safety_fault(reason)
        return

    left_wheel, right_wheel = compute_direct_wheel_turn(command_direction, remaining)
    set_direct_wheel_command(left_wheel, right_wheel)


# ============================================================
# Configuration
# ============================================================
UGV_PORT = "/dev/ttyACM0"
ULTRASONIC_PORT = "/dev/ttyUSB0"
SERIAL_BAUD = 115200

FRAME_TIMEOUT = 0.5
MAX_CAMERA_FAILURES = 3

BASE_SPEED = 0.25             # m/s (light_control2.py 차선 추종 속도)
AVOID_SPEED = 0.15            # m/s (대각선 회피 속도)
PASS_SPEED = 0.40             # m/s (직진 통과 및 복귀 대각선 속도)

FRONT_STOP_CM = 70.0
SIDE_DANGER_CM = 28.0
SIDE_CLEAR_CM = 80.0

TURN_KP = 0.045               # 대각선 주행 중 Yaw 유지용 P게인
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
    AVOID_DIAGONAL_PAUSE = auto()
    AVOID_ALIGN = auto()
    AVOID_ALIGN_PAUSE = auto()
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
shared_lane_detected = False
shared_frame_seq = 0
shared_frame_time = 0.0
shared_camera_time = None

shared_speed = 0.0
shared_angular_override = None
shared_direct_wheels = None   # (L, R) 튜플이 설정되면 {"T":1,"L":...,"R":...} 다이렉트 전송
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
    global shared_speed, shared_angular_override, shared_error, shared_direct_wheels
    with error_lock:
        shared_direct_wheels = None
        shared_speed = float(speed)
        shared_angular_override = None if angular_override is None else float(angular_override)
        if error is not None:
            shared_error = float(error)


def set_direct_wheel_command(left_speed, right_speed):
    """모터 드라이버에 {"T":1,"L":...,"R":...} 다이렉트 속도 전송을 지시"""
    global shared_speed, shared_angular_override, shared_direct_wheels
    with error_lock:
        shared_speed = 0.0
        shared_angular_override = 0.0
        shared_direct_wheels = (float(left_speed), float(right_speed))


def force_stop():
    set_direct_wheel_command(0.0, 0.0)


def transition(new_state):
    global state, state_enter_time
    state = new_state
    state_enter_time = time.monotonic()
    print(f"\n[FSM TRANSITION] -> {state.name}", flush=True)


def state_elapsed():
    return time.monotonic() - state_enter_time


# ============================================================
# Vision & Pipeline Logic (light_control2.py 방식)
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
        "appsink drop=1 max-buffers=1"
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


def process_lane_frame(frame, lane_tracker, frame_time):
    raw_edges, lane_edge_image = lane_edges(frame)
    pair, lane_status = lane_tracker.update(
        lane_edge_image,
        frame_time,
        frame=frame,
    )
    result_image, error, is_detected = draw_result(
        frame.copy(),
        pair,
        lane_status,
    )
    return error, result_image, is_detected, pair, raw_edges, lane_edge_image


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


def send_driving_safe(ugv_serial, speed, angular_z):
    try:
        with ugv_write_lock:
            ugv_control.send_driving_command(ugv_serial, speed, angular_z)
    except Exception as serial_exc:
        print(
            f"[경고] UGV 주행 시리얼 전송 지연 무시: {type(serial_exc).__name__}: {serial_exc}",
            flush=True,
        )


def control_thread_task(ugv_serial):
    global is_running, control_failure
    previous_error = None
    filtered_derivative = 0.0
    last_frame_seq = -1
    last_frame_time = None
    lane_lost_printed = False
    was_lane_tracing = False

    print("[알림] 제어 스레드 시작 (light_control2 라인 트레이싱 + 시리얼 타임아웃 보호)", flush=True)
    try:
        while is_running:
            now = time.monotonic()

            with error_lock:
                current_error = shared_error
                detected = shared_lane_detected
                current_frame_seq = shared_frame_seq
                current_frame_time = shared_frame_time
                cam_time = shared_camera_time

                c_spd = shared_speed
                override = shared_angular_override
                direct_wheels = shared_direct_wheels

            stale = (cam_time is None or now - cam_time > FRAME_TIMEOUT)

            reason = turn_watchdog_reason(now)
            if reason:
                latch_safety_fault(reason)

            in_lane_tracing = (
                safety_fault is None
                and not stale
                and direct_wheels is None
                and override is None
                and c_spd > 0.0
            )

            # light_control2.py의 모드 전환 시 PID 상태 초기화와 동일한 처리
            if in_lane_tracing and not was_lane_tracing:
                previous_error = current_error if detected else None
                filtered_derivative = 0.0
                last_frame_seq = current_frame_seq
                last_frame_time = current_frame_time if detected else None
                lane_lost_printed = False
            was_lane_tracing = in_lane_tracing

            if safety_fault is not None or stale:
                previous_error = None
                filtered_derivative = 0.0
                last_frame_time = None
                send_raw_ugv(ugv_serial, '{"T":1,"L":0.0,"R":0.0}')
            elif direct_wheels is not None:
                # light_control2 방식 다이렉트 바퀴 속도 전송
                previous_error = None
                filtered_derivative = 0.0
                last_frame_time = None
                l_spd, r_spd = direct_wheels
                send_raw_ugv(ugv_serial, f'{{"T":1,"L":{l_spd:.3f},"R":{r_spd:.3f}}}')
            elif override is not None:
                previous_error = None
                filtered_derivative = 0.0
                last_frame_time = None
                send_driving_safe(ugv_serial, c_spd, override)
            elif c_spd <= 0.0:
                previous_error = None
                filtered_derivative = 0.0
                last_frame_time = None
                send_driving_safe(ugv_serial, 0.0, 0.0)
            else:
                # ====================================================
                # light_control2.py 라인 트레이싱 (DRIVE) 제어 로직
                # ====================================================
                if detected:
                    is_new_frame = current_frame_seq != last_frame_seq

                    if is_new_frame:
                        if (
                            last_frame_time is not None
                            and current_frame_time > last_frame_time
                        ):
                            frame_dt = current_frame_time - last_frame_time
                            prev_err_arg = previous_error
                        else:
                            frame_dt = 0.033
                            prev_err_arg = None

                        angular_z, filtered_derivative = ugv_control.calculate_pid(
                            current_error,
                            prev_err_arg,
                            frame_dt,
                            filtered_derivative=filtered_derivative,
                            is_new_frame=True,
                        )

                        previous_error = current_error
                        last_frame_seq = current_frame_seq
                        last_frame_time = current_frame_time
                    else:
                        angular_z, filtered_derivative = ugv_control.calculate_pid(
                            current_error,
                            previous_error,
                            0.033,
                            filtered_derivative=filtered_derivative,
                            is_new_frame=False,
                        )

                    send_driving_safe(ugv_serial, c_spd, angular_z)
                    lane_lost_printed = False
                else:
                    previous_error = None
                    filtered_derivative = 0.0
                    last_frame_seq = current_frame_seq
                    last_frame_time = None

                    send_driving_safe(ugv_serial, 0.0, 0.0)

                    if not lane_lost_printed:
                        print("[안전정지] 차선을 찾지 못했습니다.", flush=True)
                        lane_lost_printed = True

            time.sleep(CONTROL_PERIOD_S)
    except Exception as exc:
        control_failure = f"{type(exc).__name__}: {exc}"
        print(f"[에러] 제어 스레드 중단: {control_failure}", flush=True)
        is_running = False
    finally:
        send_raw_ugv(ugv_serial, '{"T":1,"L":0.0,"R":0.0}')


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
    global avoid_direction, state, front_obstacle_count, avoid_count, recovery_done_count
    if safety_fault is not None:
        force_stop()
        return

    # 1. 차선 추종
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

        elif front_obstacle_count == 0:
            # 차선 검출/미검출 정지 및 PID는 control_thread_task에서 light_control2 방식으로 처리
            set_command(BASE_SPEED, None)
        return

    # 1-1. 장애물 발견 직후 완전 정지 (출발 시 저장한 기준 Yaw 적용)
    if state == State.AVOID_START_PAUSE:
        force_stop()
        if state_elapsed() >= 0.6:
            yaw, stamp, _ = get_imu_snapshot()
            if stamp is None or (time.monotonic() - stamp) > IMU_STALE_TIMEOUT_S:
                latch_safety_fault("기준 Yaw 확인 실패: IMU 응답 없음")
                return
            run_fsm.initial_yaw = track_front_yaw if track_front_yaw is not None else yaw
            print(
                f"[기준 각도 적용] 현재 Yaw = {yaw:.2f} deg / initial_yaw = {run_fsm.initial_yaw:.2f} deg",
                flush=True,
            )
            reset_turn()
            transition(State.AVOID_TURN_OUT)
        return

    # 2. 제자리 회전 (TURN_ANGLE_DEG 다이렉트 바퀴 제어)
    if state == State.AVOID_TURN_OUT:
        run_direct_imu_turn(avoid_direction.value, State.AVOID_PAUSE)
        return

    # 2-1. 회전 후 잠시 대기 (대각선 유지용 각도 기억)
    if state == State.AVOID_PAUSE:
        force_stop()
        if state_elapsed() >= 0.3:
            yaw, _, _ = get_imu_snapshot()
            run_fsm.diagonal_yaw = yaw
            print(f"[대각선 각도 저장] diagonal_yaw = {yaw:.2f} deg", flush=True)
            transition(State.AVOID_DIAGONAL)
        return

    # 3. 대각선 전진 (차선 넘음 감지 알고리즘 제거 -> 지정 시간 동안 대각선 이동)
    if state == State.AVOID_DIAGONAL:
        extra_time = 0.60 / AVOID_SPEED

        if state_elapsed() >= extra_time:
            force_stop()
            print("[대각선 전진 종료] 정지 후 정렬 회전 준비", flush=True)
            transition(State.AVOID_DIAGONAL_PAUSE)
            return

        # Yaw 피드백 보정으로 대각선 주행 방향 유지
        yaw, _, _ = get_imu_snapshot()
        target_yaw = getattr(run_fsm, 'diagonal_yaw', yaw)
        diff = normalize_delta(target_yaw - yaw)
        correction_angular = max(-0.3, min(0.3, TURN_KP * diff))
        set_command(AVOID_SPEED, correction_angular)
        return

    # 3-1. 대각선 전진 직후 완전 정지 (전진 관성 제거)
    if state == State.AVOID_DIAGONAL_PAUSE:
        force_stop()
        if state_elapsed() >= 0.3:
            run_fsm.align_prev_sign = None
            run_fsm.align_settle_until = 0.0
            run_fsm.align_last_log = 0.0
            transition(State.AVOID_ALIGN)
        return

    # 4. 제자리 정렬 회전 (선형 감속 + 목표각 통과 시 즉시 제동)
    if state == State.AVOID_ALIGN:
        now = time.monotonic()
        yaw, stamp, _ = get_imu_snapshot()

        if stamp is None or (now - stamp) > IMU_STALE_TIMEOUT_S:
            latch_safety_fault("정렬 회전 중 IMU 신호 끊김")
            return

        if state_elapsed() >= TURN_TIMEOUT_S:
            force_stop()
            print("[정렬 타임아웃] 강제 직진 전환", flush=True)
            transition(State.AVOID_FORWARD)
            return

        # 목표각 통과 직후 관성 및 IMU 안정화 대기
        if now < getattr(run_fsm, "align_settle_until", 0.0):
            force_stop()
            return

        target_yaw = getattr(run_fsm, 'initial_yaw', yaw)
        diff = normalize_delta(target_yaw - yaw)
        remaining = abs(diff)

        if remaining <= ALIGN_TOLERANCE_DEG:
            force_stop()
            print(f"[정렬 완료] 목표: {target_yaw:.1f} / 현재: {yaw:.1f}", flush=True)
            transition(State.AVOID_ALIGN_PAUSE)
            return

        current_sign = 1.0 if diff > 0 else -1.0
        prev_sign = getattr(run_fsm, "align_prev_sign", None)

        # 목표각을 지나쳐 방향 부호가 반전되면 즉시 정지 후 안정화
        if prev_sign is not None and current_sign != prev_sign:
            force_stop()
            run_fsm.align_prev_sign = None
            run_fsm.align_settle_until = now + ALIGN_SETTLE_S
            print(f"[정렬 통과 제동] 목표: {target_yaw:.1f} / 현재: {yaw:.1f} (diff={diff:+.1f})", flush=True)
            return

        run_fsm.align_prev_sign = current_sign
        left_wheel, right_wheel = compute_align_wheel_turn(current_sign, remaining)
        set_direct_wheel_command(left_wheel, right_wheel)

        if now - getattr(run_fsm, "align_last_log", 0.0) >= 0.2:
            print(
                f"[ALIGN] target={target_yaw:.1f}, yaw={yaw:.1f}, diff={diff:+.1f}, "
                f"L={left_wheel:+.3f}, R={right_wheel:+.3f}",
                flush=True,
            )
            run_fsm.align_last_log = now
        return

    # 4-1. 정렬 완료 후 잠시 대기 (관성 제어 및 안정화)
    if state == State.AVOID_ALIGN_PAUSE:
        force_stop()  # 차체 강제 정지 유지
        if state_elapsed() >= 0.7:  # <--- 0.7초 딜레이 (원하는 시간으로 수정 가능)
            print("[정렬 딜레이 완료] 직진 통과 시작", flush=True)
            transition(State.AVOID_FORWARD)
        return

    # 5. 직진 통과
    if state == State.AVOID_FORWARD:
        set_command(PASS_SPEED, 0.0)
        min_pass_time = 1.0 / PASS_SPEED

        if state_elapsed() >= min_pass_time:  # min_pass_time 대신 2.0 등 고정 시간 사용 가능
            force_stop()
            print("[통과] 지정 시간 직진 완료 -> 복귀 기동 시작", flush=True)
            reset_turn()
            transition(State.RECOVERY_TURN_IN)
        return

    # 6. 복귀 기동 (RECOVERY_TURN_ANGLE_DEG 다이렉트 바퀴 제어)
    if state == State.RECOVERY_TURN_IN:
        run_direct_imu_turn(
            -avoid_direction.value,
            State.RECOVERY_DIAGONAL,
            target_angle_deg=RECOVERY_TURN_ANGLE_DEG,
        )
        return

    if state == State.RECOVERY_DIAGONAL:
        if check_recovery_lane_detection(is_detected):
            return
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
    global is_running, control_failure, track_front_yaw
    global shared_error, shared_lane_detected, shared_frame_seq, shared_frame_time, shared_camera_time

    ugv_serial = ultrasonic = cap = control_thread = imu_thread = None

    try:
        ugv_serial = ugv_control.init_ugv(port=UGV_PORT, baudrate=SERIAL_BAUD)
        send_raw_ugv(ugv_serial, '{"T":1,"L":0.0,"R":0.0}')

        ugv_serial.write(b'{"T":131, "cmd":1}\n')

        imu_thread = threading.Thread(target=ugv_imu_read_task, args=(ugv_serial,), daemon=True)
        imu_thread.start()

        ultrasonic = create_ultrasonic_reader()
        time.sleep(0.5)

        # 출발 시 직선 도로 정면 기준 각도 확정 (최대 2초 대기)
        wait_start = time.monotonic()
        while time.monotonic() - wait_start < 2.0:
            yaw_init, stamp_init, _ = get_imu_snapshot()
            if stamp_init is not None and (time.monotonic() - stamp_init) <= IMU_STALE_TIMEOUT_S:
                track_front_yaw = yaw_init
                break
            time.sleep(0.05)

        if track_front_yaw is None:
            raise RuntimeError("출발 기준 IMU Yaw 수신 실패")
        run_fsm.initial_yaw = track_front_yaw
        print(f"[기준각 확정] 직선 도로 정면 track_front_yaw = {track_front_yaw:.2f} deg", flush=True)

        cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)

        if SHOW_WINDOWS:
            cv2.namedWindow(LANE_WINDOW, cv2.WINDOW_NORMAL)
            cv2.namedWindow(CANNY_WINDOW, cv2.WINDOW_NORMAL)
            print("[화면] 차선 인식과 Canny 표시. Q 또는 Esc로 종료.")
        else:
            print("[화면] SSH 또는 디스플레이 없는 환경: 화면 출력 끔.")

        control_thread = threading.Thread(target=control_thread_task, args=(ugv_serial,), daemon=True)
        control_thread.start()

        lane_tracker = LaneTracker()

        failure_count = 0
        report_time = time.monotonic()

        while is_running and cap.isOpened():
            ret, frame = cap.read()

            if not ret or frame is None or frame.size == 0:
                force_stop()
                failure_count += 1
                if failure_count >= MAX_CAMERA_FAILURES:
                    break
                time.sleep(0.05)
                continue

            failure_count = 0
            frame_time = time.monotonic()

            need_lane = state in (State.LANE_FOLLOWING, State.RECOVERY_DIAGONAL)
            if need_lane:
                error, result_image, is_detected, pair, raw_edges, lane_edges_img = process_lane_frame(
                    frame, lane_tracker, frame_time
                )
            else:
                error, is_detected, pair = 0.0, False, None
                result_image = frame
                raw_edges = lane_edges_img = None

            left_d, front_d, right_d = read_distances(ultrasonic)

            with error_lock:
                if not is_running:
                    break
                shared_camera_time = frame_time
                if need_lane:
                    shared_error = error
                    shared_lane_detected = is_detected
                    shared_frame_seq += 1
                    shared_frame_time = frame_time

            run_fsm(error, is_detected, frame, pair, left_d, front_d, right_d)

            now = time.monotonic()
            if now - report_time >= TERMINAL_LOG_INTERVAL_S:
                print(
                    f"[MONITOR] State: {state.name:<18} | US Front: {front_d:5.1f}cm | "
                    f"US Side: {left_d:5.1f} / {right_d:5.1f} | Yaw: {shared_yaw:5.1f} | Error: {error:+.1f}",
                    flush=True,
                )
                report_time = now

            if SHOW_WINDOWS:
                cv2.imshow(LANE_WINDOW, result_image)
                if raw_edges is not None and lane_edges_img is not None:
                    canny_view = np.hstack((raw_edges, lane_edges_img))
                    cv2.imshow(CANNY_WINDOW, canny_view)

                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), ord("Q"), 27):
                    break

        if control_failure:
            print(f"제어 오류: {control_failure}")

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
                send_raw_ugv(ugv_serial, '{"T":1,"L":0.0,"R":0.0}')
                time.sleep(0.05)
            ugv_control.stop_ugv(ugv_serial)

        if ultrasonic is not None:
            ultrasonic.stop()
        if cap is not None:
            cap.release()
        if SHOW_WINDOWS:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()