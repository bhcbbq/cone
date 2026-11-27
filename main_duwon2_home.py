# 최신 수정:
# 1. 대각선 전진(AVOID_DIAGONAL) 시 차선 넘음 감지 알고리즘 제거 (고정 거리/시간 이동)
# 2. 모든 제자리 회전(AVOID_TURN_OUT, AVOID_ALIGN, RECOVERY_TURN_IN)을 light_control2 방식의
#    다이렉트 좌/우 바퀴 제어({"T":1,"L":...,"R":...}) 및 IMU Yaw 누적/목표각 제어로 변경
# 3. SerialTimeoutException(Write timeout) 발생 시 제어 스레드가 중단되지 않도록 try-except 방어 적용
# 4. 복귀 회전 각도(RECOVERY_TURN_ANGLE_DEG) 별도 분리 및 AVOID_FORWARD 들여쓰기 오류 수정
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
TURN_ANGLE_DEG = 16.0           # 회피 회전 목표 각도 (도)
RECOVERY_TURN_ANGLE_DEG = 8.0 # 복귀 회전 목표 각도 (도) - 원하는 각도로 조절
TURN_TOLERANCE_DEG = 2.0        # 허용 오차 (도)

# light_control2.py 스타일의 다이렉트 바퀴 속도 (L, R 직접 인가)
DIRECT_TURN_SPEED = 0.15        # 기본 제자리 회전 바퀴 속도
DIRECT_TURN_SLOW_SPEED = 0.11   # 목표각 근접 시 감속 바퀴 속도
DIRECT_TURN_SLOW_REMAIN_DEG = 30

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

BASE_SPEED = 0.30             # m/s (일반 차선 추종)
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
shared_speed = 0.0
shared_angular_override = None
shared_direct_wheels = None   # (L, R) 튜플이 설정되면 {"T":1,"L":...,"R":...} 다이렉트 전송
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