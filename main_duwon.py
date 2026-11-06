# main_highway_contour_avoid_final_hybrid_optimized.py
import cv2
import time
import threading
import traceback
import math
import numpy as np
import os
from enum import Enum, auto

import lane_detection_ver1 as lane_detection
import ugv_control
from ultrasonic_reader import UltrasonicReader

# ============================================================
# Configuration
# ============================================================
UGV_PORT = "/dev/ttyACM0"
ULTRASONIC_PORT = "/dev/ttyUSB0"
SERIAL_BAUD = 115200

FRAME_WIDTH = 640
FRAME_TIMEOUT = 0.5
MAX_CAMERA_FAILURES = 3

BASE_SPEED = 0.30             # m/s (일반 차선 추종)
AVOID_SPEED = 0.15            # m/s 
PASS_SPEED = 0.30             # m/s (대각선 및 직진 통과)

# 거리 기반 회피 조건 파라미터
FRONT_STOP_CM = 60.0         # 회피 시작 기준 거리 (전방 장애물 인식)
FRONT_SAFE_CM = 70.0          # 회전 시 전방이 70cm 이상 확보되어야 회전 완료
SIDE_DANGER_CM = 30.0         # 측면 30cm 미만일 경우 너무 가까워 임시 정지
SIDE_CLEAR_CM = 50.0          # 측면 50cm 이상 멀어지면 장애물을 통과했다고 판단

ALIGN_TIME_S = 1.6
TURN_ANGULAR_SPEED = 2.4      # 제자리 회전 각속도

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
def gstreamer_pipeline(sensor_id=0, capture_width=1640, capture_height=1232, display_width=820, display_height=616, framerate=30, flip_method=0):
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! video/x-raw(memory:NVMM), width=(int){capture_width}, height=(int){capture_height}, format=(string)NV12, framerate=(fraction){framerate}/1 ! "
        f"nvvidconv flip-method={flip_method} ! video/x-raw, width=(int){display_width}, height=(int){display_height}, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink drop=true max-buffers=1 sync=false"
    )

def process_lane_frame(frame, tracker, now):
    target_height = max(2, round(frame.shape[0] * FRAME_WIDTH / frame.shape[1]))
    frame = cv2.resize(frame, (FRAME_WIDTH, target_height))
    _, edges, top, bottom = lane_detection.get_edges(frame, return_raw=True)
    pair, status, inferred = tracker.update(edges, top, bottom, now)
    result_image, error, is_detected = lane_detection.draw_result(frame.copy(), pair, status, inferred, top, bottom)
    return error, result_image, is_detected, pair

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
            ugv_control.clear_feedback_buffer(ugv_serial)
            now = time.monotonic()

            with error_lock:
                c_err, c_spd, override, f_time = shared_error, shared_speed, shared_angular_override, shared_frame_time

            stale = (f_time is None or now - f_time > FRAME_TIMEOUT)

            if stale or (c_spd <= 0 and override is None):
                c_spd = 0.0 if override is None else c_spd
                angular_z = override if override is not None else 0.0
                previous_error = previous_frame_time = None
            else:
                if override is not None:
                    angular_z = override
                    previous_error = previous_frame_time = None
                else:
                    if previous_frame_time is None or f_time != previous_frame_time:
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

    # 1. 차선 추종 (방어적 비상 정지 적용 완료)
    if state == State.LANE_FOLLOWING:
        is_front_close = (0.0 < front_d <= FRONT_STOP_CM) or (front_obstacle_count > 0 and front_d < 0)

        if is_front_close:
            front_obstacle_count += 1
            # 1회만 감지되어도 즉시 모터를 끄고 강제 브레이크
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

    # 1-1. 장애물 발견 직후 완전 정지 (관성 제거 0.6초)
    if state == State.AVOID_START_PAUSE:
        force_stop()
        if state_elapsed() >= 0.6: 
            transition(State.AVOID_TURN_OUT)
        return

    # 2. 제자리 회전
    if state == State.AVOID_TURN_OUT:
        set_command(0.0, avoid_direction.value * TURN_ANGULAR_SPEED)
        if front_d >= FRONT_SAFE_CM or front_d < 0:
            force_stop()
            transition(State.AVOID_PAUSE)
        return

    # 2-1. 회전 후 잠시 대기
    if state == State.AVOID_PAUSE:
        force_stop()
        if state_elapsed() >= 0.3:
            transition(State.AVOID_DIAGONAL)
        return

    # 3. 대각선 전진
    if state == State.AVOID_DIAGONAL:
        side_d = right_d if avoid_direction == AvoidDirection.LEFT else left_d
        
        if 0.0 < side_d < SIDE_DANGER_CM:
            force_stop()
            print(f"[위험] 측면 장애물 초근접! ({side_d:.1f}cm) -> 충돌 방지 대기", flush=True)
        elif side_d >= SIDE_CLEAR_CM:
            force_stop()
            print(f"[통과] 측면 클리어 ({side_d:.1f}cm) -> 평행 정렬 시작", flush=True)
            transition(State.AVOID_ALIGN)
        elif side_d < 0:
            set_command(AVOID_SPEED, 0.0)
        else:
            set_command(AVOID_SPEED, 0.0)
        return

    # 4. 제자리 정렬
    if state == State.AVOID_ALIGN:
        set_command(0.0, -avoid_direction.value * TURN_ANGULAR_SPEED)
        if state_elapsed() >= ALIGN_TIME_S:
            force_stop()
            transition(State.AVOID_FORWARD)
        return

    # 5. 직진 통과
    if state == State.AVOID_FORWARD:
        set_command(PASS_SPEED, 0.0)
        if state_elapsed() >= 1.2:
            force_stop()
            transition(State.RECOVERY_TURN_IN)
        return

    # 6. 복귀 기동
    if state == State.RECOVERY_TURN_IN:
        if check_recovery_lane_detection(is_detected): return
        set_command(0.0, -avoid_direction.value * TURN_ANGULAR_SPEED)
        if state_elapsed() >= ALIGN_TIME_S:
            force_stop()
            transition(State.RECOVERY_DIAGONAL)
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

    if lane_detection.ROAD_MODE not in ("highway", "black_track"):
        print("[에러] ROAD_MODE 설정을 확인하세요.", flush=True)
        return

    ugv_serial = ultrasonic = cap = control_thread = None
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
            error, result_image, is_detected, pair = process_lane_frame(frame, tracker, frame_started)
            left_d, front_d, right_d = read_distances(ultrasonic)

            with error_lock:
                if not is_running: break
                shared_frame_time = frame_started

            run_fsm(error, is_detected, frame, pair, left_d, front_d, right_d)

            now = time.monotonic()
            if now - report_time >= TERMINAL_LOG_INTERVAL_S:
                usable = (bool(is_detected) and now - frame_started <= FRAME_TIMEOUT)
                print(f"[MONITOR] State: {state.name:<18} | US Front: {front_d:5.1f}cm | US Side: {left_d:5.1f} / {right_d:5.1f} | Error: {error:+.1f} | Usable: {usable}", flush=True)
                report_time = now

            if SHOW_WINDOWS:
                cv2.putText(result_image, f"STATE: {state.name}", (10, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
                cv2.putText(result_image, f"Avoid: {avoid_direction.name}", (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1)
                cv2.imshow(window_name, result_image)
                if cv2.waitKey(1) & 0xFF in (ord("q"), ord("Q"), 27): break
                    
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