# main_highway_contour_avoid_final_hybrid.py
import cv2
import time
import threading
import traceback
import math
import numpy as np
import os
from enum import Enum, auto

# 모듈 임포트
import lane_detection_ver1 as lane_detection
import ugv_control
from ultrasonic_reader import UltrasonicReader


# ============================================================
# Configuration (설정)
# ============================================================

UGV_PORT = "/dev/ttyACM0"
ULTRASONIC_PORT = "/dev/ttyUSB0"
SERIAL_BAUD = 115200

FRAME_WIDTH = 640
FRAME_TIMEOUT = 0.5
MAX_CAMERA_FAILURES = 3

BASE_SPEED = 0.30             # m/s (일반 차선 추종)
AVOID_SPEED = 0.15            # m/s (기본 회피 보조 속도)
PASS_SPEED = 0.3             # m/s (대각선 전진 및 직진 통과 속도)

# [장애물 회피 파라미터]
FRONT_STOP_CM = 80.0         # 회피 시작 기준 거리
OBSTACLE_CLEAR_CM = 50.0    
OBSTACLE_CLEAR_HOLD_S = 0.5

# 제자리 회전 및 대각선 시간 설정
TURN_OUT_TIME_S = 0.60
DIAGONAL_TIME_S = 1.5       
ALIGN_TIME_S = 0.60

TURN_ANGULAR_SPEED = 1.8      # 무거운 차체 대비 높은 각속도
PASS_DISTANCE_CM = 80.0      # 평행 직진 통과 구간
PASS_TIMEOUT_S = PASS_DISTANCE_CM / (PASS_SPEED * 100.0)


CONTROL_PERIOD_S = 0.02
NO_OBSTACLE = -1.0
TERMINAL_LOG_INTERVAL_S = 1.0

SHOW_WINDOWS = bool(os.environ.get("DISPLAY")) and not bool(
    os.environ.get("SSH_CONNECTION") or os.environ.get("SSH_CLIENT") or os.environ.get("SSH_TTY")
)


# ============================================================
# FSM State & Direction
# ============================================================
class State(Enum):
    LANE_FOLLOWING = auto()    # 정상 차선 추종
    AVOID_TURN_OUT = auto()    # [정지 상태] 제자리 회전 (틀기)
    AVOID_DIAGONAL = auto()    # [직진 상태] 대각선 전진
    AVOID_ALIGN = auto()       # [정지 상태] 제자리 정렬 회전
    AVOID_FORWARD = auto()     # [직진 상태] 직진 통과
    
    # [복귀 단계 추가]
    RECOVERY_TURN_IN = auto()  # [정지 상태] 차선 방향으로 제자리 회전
    RECOVERY_DIAGONAL = auto() # [직진 상태] 복귀 대각선 전진
    RECOVERY_ALIGN = auto()    # [정지 상태] 차선 정렬 회전
    
    BLOCKED_STOP = auto()      # 비상 정지

class AvoidDirection(Enum):
    LEFT = 1     # +Z : 좌회전 
    RIGHT = -1   # -Z : 우회전 


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
obstacle_clear_since = None              
front_obstacle_count = 0                 
recovery_detect_count = 0                

# [카운터 변수 추가]
avoid_count = 0           # 회피 동작 수행 횟수 카운트
recovery_done_count = 0    # 복귀 동작 수행 횟수 카운트


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
# Camera Pipeline
# ============================================================
def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1640,
    capture_height=1232,
    display_width=820,
    display_height=616,
    framerate=30,
    flip_method=0,
):
    return (
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "format=(string)NV12, "
        "framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, "
        "height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! "
        "appsink drop=true max-buffers=1 sync=false"
        % (sensor_id, capture_width, capture_height, framerate, flip_method, display_width, display_height)
    )


# ============================================================
# Vision Logic
# ============================================================
def process_lane_frame(frame, tracker, now):
    target_height = max(2, round(frame.shape[0] * FRAME_WIDTH / frame.shape[1]))
    frame = cv2.resize(frame, (FRAME_WIDTH, target_height))
    raw_edges, edges, top, bottom = lane_detection.get_edges(frame, return_raw=True)
    pair, status, inferred = tracker.update(edges, top, bottom, now)
    result_image, error, is_detected = lane_detection.draw_result(frame.copy(), pair, status, inferred, top, bottom)
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
# Ultrasonic & Thread
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
    global is_running, control_failure
    previous_error = None
    previous_frame_time = None
    angular_z = 0.0
    frame_dt = None

    print("[알림] 제어 스레드 시작: 새 영상마다 PD 계산 + FSM 오버라이드 지원", flush=True)

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

            if stale or (current_speed <= 0 and override is None):
                current_speed = 0.0 if override is None else current_speed
                angular_z = override if override is not None else 0.0
                previous_error = None
                previous_frame_time = None
                frame_dt = None
            else:
                if override is not None:
                    angular_z = override
                    previous_error = None
                    previous_frame_time = None
                else:
                    new_frame = (previous_frame_time is None or frame_time != previous_frame_time)
                    if new_frame:
                        if previous_frame_time is None:
                            frame_dt = None
                        else:
                            frame_dt = (frame_time - previous_frame_time)

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
        control_failure = f"{type(exc).__name__}: {exc}"
        print(f"[에러] 제어 스레드 중단: {control_failure}", flush=True)
        is_running = False
    finally:
        try:
            ugv_control.send_driving_command(ugv_serial, 0.0, 0.0)
        except:
            pass


# ============================================================
# FSM (회피 횟수 대칭 복귀 로직 포함 & 하이브리드 센서 보조)
# ============================================================
def run_fsm(error, is_detected, frame, pair, left_d, front_d, right_d):
    global avoid_direction, obstacle_clear_since, state
    global front_obstacle_count, recovery_detect_count
    global avoid_count, recovery_done_count
    now = time.monotonic()

    # [1] 일반 차선 추종
    if state == State.LANE_FOLLOWING:
        if front_d > 0.0 and front_d <= FRONT_STOP_CM:
            front_obstacle_count += 1
        else:
            front_obstacle_count = 0

        if front_obstacle_count >= 3:
            front_obstacle_count = 0
            avoid_count = 0          
            recovery_done_count = 0  
            force_stop()
            
            # [수정] 카메라 비전 판단 우선, 센서 데이터는 신뢰 구간(25cm 초과)일 때만 참고
            vision_dir = get_obstacle_direction_by_contour(frame, pair)
            if (25.0 < left_d) and (25.0 < right_d):
                if left_d > right_d + 15.0:  
                    avoid_direction = AvoidDirection.LEFT
                elif right_d > left_d + 15.0:
                    avoid_direction = AvoidDirection.RIGHT
                else:
                    avoid_direction = vision_dir
            else:
                avoid_direction = vision_dir
                
            print(f"\n[OBSTACLE DETECTED] Front: {front_d:.1f}cm -> Avoid Dir: {avoid_direction.name}", flush=True)
            transition(State.AVOID_TURN_OUT)
        elif not is_detected:
            force_stop()
        else:
            set_command(BASE_SPEED, None, error=error)
        return

    # [2] 제자리 회전 (회피 방향으로 틀기)
    if state == State.AVOID_TURN_OUT:
        set_command(0.0, avoid_direction.value * TURN_ANGULAR_SPEED)
        if state_elapsed() >= TURN_OUT_TIME_S:
            force_stop()
            transition(State.AVOID_DIAGONAL)
        return

    # [3] 대각선 전진
    if state == State.AVOID_DIAGONAL:
        set_command(PASS_SPEED, 0.0)
        if state_elapsed() >= DIAGONAL_TIME_S:
            force_stop()
            transition(State.AVOID_ALIGN)
        return

    # [4] 제자리 정렬 후 횟수 기록 & 전/측방 재확인
    if state == State.AVOID_ALIGN:
        set_command(0.0, -avoid_direction.value * TURN_ANGULAR_SPEED)
        
        if state_elapsed() >= ALIGN_TIME_S:
            force_stop()
            avoid_count += 1 
            print(f"[AVOID COUNTER] 현재 회피 횟수: {avoid_count}회", flush=True)
            
            # 회피 방향의 반대쪽 측면 센서 (장애물이 있는 방향)
            side_d = right_d if avoid_direction == AvoidDirection.LEFT else left_d
            
            # [수정] 전방은 80cm 기준 유지, 측면은 센서 신뢰 구간(25~40cm)일 때만 장애물로 판단
            front_blocked = (0.0 < front_d <= FRONT_STOP_CM)
            side_blocked_reliable = (25.0 < side_d <= 40.0) 

            if front_blocked or side_blocked_reliable:
                print(f"[RE-AVOID] 추가 회피 진행 (Front: {front_d:.1f}cm, Side: {side_d:.1f}cm)", flush=True)
                transition(State.AVOID_TURN_OUT)
            else:
                transition(State.AVOID_FORWARD)
        return

    # [5] 넉넉한 직진 통과 (하이브리드 연장 로직)
    if state == State.AVOID_FORWARD:
        set_command(PASS_SPEED, 0.0)
        
        if state_elapsed() >= PASS_TIMEOUT_S:
            # 시간이 지났을 때, 측면 센서를 확인하여 확실히 근처에 장애물이 있는지 확인
            side_d = right_d if avoid_direction == AvoidDirection.LEFT else left_d
            
            # [수정] 25~50cm 사이의 명확한 신뢰 구간에 장애물이 포착되면 직진 연장 (복귀 보류)
            if 25.0 < side_d <= 50.0:
                # 상태 전환 없이 직진(PASS_SPEED) 유지
                pass 
            else:
                # 50cm 초과(클리어), 25cm 이하(불감지), -1(없음)일 경우 기존 시간 제어를 믿고 복귀
                force_stop()
                recovery_done_count = 0
                transition(State.RECOVERY_TURN_IN)
        return

    # ============================================================
    # [6] 대칭 복귀 기동 (avoid_count 횟수만큼 복귀 기동 반복)
    # ============================================================
    
    # 6-1. 차선 방향으로 틀기 (회피 방향의 반대)
    if state == State.RECOVERY_TURN_IN:
        if is_detected:
            recovery_detect_count += 1
            if recovery_detect_count >= 5:
                recovery_detect_count = 0
                force_stop()
                print("[RECOVERY COMPLETE] 차선 선명 감지 -> 차선 추종 복귀", flush=True)
                transition(State.LANE_FOLLOWING)
                return
        else:
            recovery_detect_count = 0

        set_command(0.0, -avoid_direction.value * TURN_ANGULAR_SPEED)
        if state_elapsed() >= TURN_OUT_TIME_S:
            force_stop()
            transition(State.RECOVERY_DIAGONAL)
        return

    # 6-2. 대각선 전진 복귀
    if state == State.RECOVERY_DIAGONAL:
        if is_detected:
            recovery_detect_count += 1
            if recovery_detect_count >= 5:
                recovery_detect_count = 0
                force_stop()
                print("[RECOVERY COMPLETE] 차선 선명 감지 -> 차선 추종 복귀", flush=True)
                transition(State.LANE_FOLLOWING)
                return
        else:
            recovery_detect_count = 0

        set_command(PASS_SPEED, 0.0)
        if state_elapsed() >= DIAGONAL_TIME_S:
            force_stop()
            transition(State.RECOVERY_ALIGN)
        return

    # 6-3. 차선 정렬 회전
    if state == State.RECOVERY_ALIGN:
        set_command(0.0, avoid_direction.value * TURN_ANGULAR_SPEED)
        if state_elapsed() >= ALIGN_TIME_S:
            force_stop()
            recovery_done_count += 1
            print(f"[RECOVERY COUNTER] 복귀 진행 {recovery_done_count}/{avoid_count}회 완료", flush=True)

            if recovery_done_count >= avoid_count or is_detected:
                avoid_count = 0
                recovery_done_count = 0
                transition(State.LANE_FOLLOWING)
            else:
                transition(State.RECOVERY_TURN_IN)
        return

    # [7] 비상 정지 대기
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
# Main Loop
# ============================================================
def main():
    global is_running, shared_error, shared_speed, shared_frame_time
    global control_failure

    if lane_detection.ROAD_MODE not in ("highway", "black_track"):
        print("[에러] ROAD_MODE 설정을 확인하세요.", flush=True)
        return

    ugv_serial = None
    ultrasonic = None
    cap = None
    control_thread = None

    window_name = "UGV - Autonomous Avoidance"
    
    try:
        ugv_serial = ugv_control.init_ugv(port=UGV_PORT, baudrate=SERIAL_BAUD)
        ugv_control.send_driving_command(ugv_serial, 0.0, 0.0)

        ultrasonic = create_ultrasonic_reader()
        time.sleep(0.5)

        cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
        
        tracker = lane_detection.LaneTracker()

        if SHOW_WINDOWS:
            cv2.namedWindow(window_name)

        control_thread = threading.Thread(target=control_thread_task, args=(ugv_serial,), daemon=True)
        control_thread.start()
        
        failure_count = 0
        report_time = time.monotonic()

        while is_running:
            if not cap.isOpened():
                break

            frame_started = time.monotonic()
            ret, frame = cap.read()

            if not ret or frame is None or frame.size == 0:
                force_stop()
                tracker.reset()
                failure_count += 1
                if failure_count >= MAX_CAMERA_FAILURES:
                    break
                time.sleep(0.05)
                continue

            failure_count = 0

            error, result_image, is_detected, raw_edges, edges, status, pair = process_lane_frame(frame, tracker, frame_started)
            
            left_d, front_d, right_d = read_distances(ultrasonic)

            with error_lock:
                if not is_running:
                    break
                shared_frame_time = frame_started

            run_fsm(error, is_detected, frame, pair, left_d, front_d, right_d)

            now = time.monotonic()
            if now - report_time >= TERMINAL_LOG_INTERVAL_S:
                usable = (bool(is_detected) and now - frame_started <= FRAME_TIMEOUT)
                print(
                    f"[MONITOR] State: {state.name:<18} | "
                    f"US Front: {front_d:5.1f}cm | Avoid/Recov: {avoid_count}/{recovery_done_count} | "
                    f"Lane Error: {error:+.1f} | Usable: {usable}", flush=True
                )
                report_time = now

            if SHOW_WINDOWS:
                cv2.putText(result_image, f"STATE: {state.name}", (10, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
                cv2.putText(result_image, f"Avoid: {avoid_direction.name} ({avoid_count}x)", (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1)
                
                cv2.imshow(window_name, result_image)
                if cv2.waitKey(1) & 0xFF in (ord("q"), ord("Q"), 27):
                    break
                    
        if control_failure is not None:
            print(f"제어 오류: {control_failure}")

    except KeyboardInterrupt:
        print("\n[STOP] Keyboard Interrupt detected.")
    except Exception as exc:
        traceback.print_exc()
    finally:
        is_running = False
        force_stop()
        if control_thread is not None:
            control_thread.join()
        if ugv_serial is not None:
            ugv_control.stop_ugv(ugv_serial)
        if ultrasonic is not None:
            ultrasonic.stop()
        if cap is not None:
            cap.release()
        if SHOW_WINDOWS:
            cv2.destroyAllWindows()

if __name__ == "__main__":
    main()