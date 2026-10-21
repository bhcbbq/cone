import cv2
import time
import threading
import json
import serial
import numpy as np

import lane_detection_ver1 as lane_detection
import ugv_control

# ============================================================
# SETTINGS
# ============================================================

BASE_SPEED = 0.30

# 수동 주행 전용 속도
# 앱 수동 페이지의 슬라이더(SPEED:0.20 ~ 0.50)로 변경
MANUAL_SPEED = 0.30
MANUAL_SPEED_MIN = 0.20
MANUAL_SPEED_MAX = 0.50

# 대각선 주행 시 안쪽 바퀴 속도 비율
MANUAL_INNER_RATIO = 0.50

# 복귀 제자리 180도 회전 속도
TURN_SPEED = 0.25

# 영상 정렬 시 천천히 회전
ALIGN_TURN_SPEED = 0.18

# 제자리 회전 엔코더 기준 거리
# 실제 180도 회전각을 보면서 조정
COARSE_TURN_DISTANCE_M = 0.44

# 차선 중심 오차 허용 범위
# 실제 테스트하면서 조정
ALIGN_ERROR_THRESHOLD = 15.0

# 몇 프레임 연속으로 중앙에 들어와야
# 정렬 완료로 판단할지
ALIGN_STABLE_FRAMES = 8

HC12_PORT = "/dev/ttyTHS1"
HC12_BAUD = 115200


# ============================================================
# SHARED STATE
# ============================================================

# IDLE
# DRIVE
# RETURN_TURN
# RETURN_ALIGN
# RETURN_DRIVE
mode = "IDLE"

is_running = True

shared_error = 0.0
lane_detected = False

# 수동 주행 시 좌/우 바퀴에 보낼 속도
manual_left_speed = 0.0
manual_right_speed = 0.0

state_lock = threading.Lock()
ugv_write_lock = threading.Lock()
hc12_write_lock = threading.Lock()

ugv_serial = None
hc12_serial = None


# ============================================================
# ENCODER STATE
# ============================================================

current_odl = None
current_odr = None

# 일반 전진 구간 시작값
start_odl = None
start_odr = None

# STOP 후 최종 거리 계산
stop_pending = False

# STOP 후 다시 START했는지 여부
paused_mission = False

# 지금까지 실제 전진한 총 거리
mission_outbound_total = 0.0

# RETURN 시 돌아가야 하는 거리
return_drive_distance = 0.0


# ============================================================
# RETURN STATE
# ============================================================

return_turn_start_odl = None
return_turn_start_odr = None

return_drive_start_odl = None
return_drive_start_odr = None

align_stable_count = 0

# 복귀 중 STOP 후 재개할 단계 저장
paused_return_mode = None

lane_lost_printed = False


# ============================================================
# HC12 TX
# ============================================================


def send_hc12(message):

    if hc12_serial is None:
        return

    if not message.endswith("\n"):
        message += "\n"

    try:

        with hc12_write_lock:

            hc12_serial.write(message.encode("utf-8"))

            hc12_serial.flush()

    except Exception as e:

        print("[HC12 TX 오류]", e)


# ============================================================
# UGV RAW TX
# ============================================================


def send_raw_ugv(message):

    if ugv_serial is None:
        return

    if not message.endswith("\n"):
        message += "\n"

    try:

        with ugv_write_lock:

            ugv_serial.write(message.encode("utf-8"))

            ugv_serial.flush()

    except Exception as e:

        print("[UGV TX 오류]", e)


# ============================================================
# UGV STOP
# ============================================================


def stop_ugv_now():

    send_raw_ugv('{"T":1,"L":0.0,"R":0.0}')


# ============================================================
# HC-12 COMMAND THREAD
# ============================================================


def hc12_thread_task():

    global mode
    global MANUAL_SPEED
    global manual_left_speed
    global manual_right_speed

    global start_odl
    global start_odr

    global stop_pending
    global paused_mission

    global mission_outbound_total
    global return_drive_distance

    global return_turn_start_odl
    global return_turn_start_odr

    global align_stable_count
    global paused_return_mode

    buffer = ""

    print("[알림] HC-12 명령 수신 스레드 시작")

    while is_running:

        try:

            if hc12_serial is None:

                time.sleep(0.05)
                continue

            if not hc12_serial.in_waiting:

                time.sleep(0.005)
                continue

            raw = hc12_serial.read(hc12_serial.in_waiting)

            text = raw.decode("utf-8", errors="ignore")

            for ch in text:

                if ch == "\n":

                    cmd = buffer.strip()
                    buffer = ""

                    if not cmd:
                        continue

                    print("[HC12 RX]", cmd)

                    # ==================================================
                    # EMERGENCY STOP
                    # ==================================================

                    if cmd == "EMERGENCY_STOP":

                        with state_lock:

                            mode = "IDLE"
                            stop_pending = False
                            align_stable_count = 0
                            paused_return_mode = None

                        send_raw_ugv('{"T":0}')

                        send_hc12("EMERGENCY")

                        print("[비상정지] EMERGENCY STOP")

                        continue

                    # ==================================================
                    # STOP
                    # ==================================================

                    if cmd == "STOP":

                        with state_lock:

                            # 복귀 중 STOP이면 현재 복귀 단계를 기억하고 일시정지
                            if mode in (
                                "RETURN_TURN",
                                "RETURN_ALIGN",
                                "RETURN_DRIVE",
                            ):
                                paused_return_mode = mode
                                stop_pending = False
                                was_returning = True

                            # 수동조작 중 STOP은 자동주행 거리/재개 상태에 섞지 않음
                            elif mode == "MANUAL":
                                paused_return_mode = None
                                stop_pending = False
                                was_returning = False
                                manual_left_speed = 0.0
                                manual_right_speed = 0.0

                            else:
                                paused_return_mode = None
                                stop_pending = True
                                paused_mission = True
                                was_returning = False

                            mode = "IDLE"
                            align_stable_count = 0

                        stop_ugv_now()

                        if was_returning:
                            send_hc12("RETURN_PAUSED")
                            print("[RETURN] 일시정지")
                        else:
                            print("[정지] UGV STOP")
                            print("[정지] 최종 엔코더 거리 대기")

                        continue

                    # ==================================================
                    # SPEED - 수동주행 속도 설정
                    # 예: SPEED:0.30
                    # ==================================================

                    if cmd.startswith("SPEED:"):

                        try:
                            new_speed = float(cmd.split(":", 1)[1])

                        except ValueError:
                            print("[수동 속도 오류]", cmd)
                            send_hc12("ERROR:INVALID_SPEED")
                            continue

                        if not (
                            MANUAL_SPEED_MIN
                            <= new_speed
                            <= MANUAL_SPEED_MAX
                        ):
                            print("[수동 속도 오류] 범위 초과:", new_speed)
                            send_hc12("ERROR:INVALID_SPEED")
                            continue

                        with state_lock:
                            current_mode = mode

                            # 자율주행/복귀 중에는 속도 변경 차단
                            if current_mode in (
                                "DRIVE",
                                "RETURN_TURN",
                                "RETURN_ALIGN",
                                "RETURN_DRIVE",
                            ):
                                speed_blocked = True
                            else:
                                speed_blocked = False
                                MANUAL_SPEED = new_speed

                        if speed_blocked:
                            send_hc12("MANUAL_BLOCKED")
                            print("[수동 속도] 자율주행/복귀 중 변경 차단")
                        else:
                            send_hc12(f"SPEED:{MANUAL_SPEED:.2f}")
                            print("[수동 속도]", f"{MANUAL_SPEED:.2f}")

                        continue

                    # ==================================================
                    # MANUAL - 수동 8방향 주행
                    # ==================================================

                    if cmd.startswith("MANUAL:"):

                        direction = cmd.split(":", 1)[1].strip().upper()

                        with state_lock:
                            current_mode = mode

                        # 자동주행/복귀 중에는 수동조작 금지
                        if current_mode in (
                            "DRIVE",
                            "RETURN_TURN",
                            "RETURN_ALIGN",
                            "RETURN_DRIVE",
                        ):
                            send_hc12("MANUAL_BLOCKED")
                            print("[수동] 자율주행/복귀 중 명령 차단:", direction)
                            continue

                        # 방향 버튼에서 손을 뗐을 때
                        if direction == "STOP":

                            with state_lock:
                                manual_left_speed = 0.0
                                manual_right_speed = 0.0

                                if mode == "MANUAL":
                                    mode = "IDLE"

                            stop_ugv_now()
                            send_hc12("MANUAL_STOPPED")
                            print("[수동] STOP")
                            continue

                        speed = MANUAL_SPEED
                        inner = speed * MANUAL_INNER_RATIO

                        manual_map = {
                            "FORWARD": (
                                speed,
                                speed,
                            ),
                            "FORWARD_LEFT": (
                                inner,
                                speed,
                            ),
                            "FORWARD_RIGHT": (
                                speed,
                                inner,
                            ),
                            "LEFT": (
                                -speed,
                                speed,
                            ),
                            "RIGHT": (
                                speed,
                                -speed,
                            ),
                            "BACKWARD": (
                                -speed,
                                -speed,
                            ),
                            "BACKWARD_LEFT": (
                                -inner,
                                -speed,
                            ),
                            "BACKWARD_RIGHT": (
                                -speed,
                                -inner,
                            ),
                        }

                        wheel_speed = manual_map.get(direction)

                        if wheel_speed is None:
                            print("[수동 방향 오류]", direction)
                            send_hc12("ERROR:INVALID_MANUAL")
                            continue

                        left_speed, right_speed = wheel_speed

                        with state_lock:
                            manual_left_speed = left_speed
                            manual_right_speed = right_speed
                            mode = "MANUAL"

                        send_hc12(f"MANUAL:{direction}")

                        print(
                            "[수동]",
                            direction,
                            "L=",
                            f"{left_speed:.2f}",
                            "R=",
                            f"{right_speed:.2f}",
                        )

                        continue

                    # ==================================================
                    # TARGET
                    # ==================================================

                    if cmd.startswith("TARGET:"):

                        try:

                            distance = float(cmd.split(":", 1)[1])

                            if distance <= 0:

                                print("[오류] INVALID TARGET")

                                send_hc12("ERROR:INVALID_TARGET")

                                continue

                            with state_lock:

                                if not paused_mission:

                                    mission_outbound_total = 0.0
                                    return_drive_distance = 0.0

                                start_odl = None
                                start_odr = None

                                stop_pending = False

                            send_raw_ugv('{"T":2101,"d":' + str(distance) + "}")

                            send_hc12(f"TARGET:{distance:.2f}")

                            print("[목표거리]", f"{distance:.2f} m")

                        except ValueError:

                            print("[오류] INVALID TARGET:", cmd)

                            send_hc12("ERROR:INVALID_TARGET")

                        continue

                    # ==================================================
                    # START
                    # ==================================================

                    if cmd == "START":

                        with state_lock:

                            mode = "DRIVE"
                            stop_pending = False
                            align_stable_count = 0

                        send_hc12("DRIVING")

                        print("[주행] START")

                        print("[주행] 카메라 차선추종 시작")

                        continue

                    # ==================================================
                    # RETURN
                    # ==================================================

                    if cmd == "RETURN":

                        stop_ugv_now()

                        time.sleep(0.2)

                        with state_lock:

                            if current_odl is None or current_odr is None:

                                print("[RETURN 오류] 엔코더 데이터 없음")

                                send_hc12("RETURN_ERROR:NO_ODOM")

                                continue

                            if return_drive_distance <= 0:

                                print("[RETURN 오류] 저장된 복귀거리 없음")

                                send_hc12("RETURN_ERROR:NO_DISTANCE")

                                continue

                            return_turn_start_odl = current_odl

                            return_turn_start_odr = current_odr

                            align_stable_count = 0
                            paused_return_mode = None

                            mode = "RETURN_TURN"

                            stop_pending = False

                        send_hc12("RETURNING")

                        print("[RETURN] 대략적인 180도 회전 시작")

                        print("[RETURN] 복귀거리:", f"{return_drive_distance:.2f} m")

                        continue

                    # ==================================================
                    # RESUME RETURN
                    # ==================================================

                    if cmd == "RESUME_RETURN":

                        with state_lock:

                            if paused_return_mode is None:

                                print("[RETURN] 일시정지된 복귀 없음")

                                send_hc12("RETURN_ERROR:NO_PAUSED_RETURN")

                                continue

                            mode = paused_return_mode
                            paused_return_mode = None
                            stop_pending = False

                        send_hc12("RETURNING")

                        print("[RETURN] 복귀 재개:", mode)

                        continue

                    # ==================================================
                    # RESET
                    # ==================================================

                    if cmd == "RESET_DISTANCE":

                        with state_lock:

                            start_odl = None
                            start_odr = None

                            mission_outbound_total = 0.0
                            return_drive_distance = 0.0

                            paused_mission = False
                            align_stable_count = 0
                            paused_return_mode = None

                        send_hc12("DISTANCE:0.00")

                        print("[거리] RESET")

                        continue

                    print("[HC12] UNKNOWN COMMAND:", cmd)

                elif ch != "\r":

                    buffer += ch

        except Exception as e:

            print("[HC12 RX 오류]", e)

            time.sleep(0.05)


# ============================================================
# UGV TELEMETRY THREAD
# ============================================================


def ugv_receive_thread_task():

    global mode

    global current_odl
    global current_odr

    global start_odl
    global start_odr

    global stop_pending
    global paused_mission

    global mission_outbound_total
    global return_drive_distance

    global return_drive_start_odl
    global return_drive_start_odr
    global paused_return_mode

    print("[알림] UGV 수신 스레드 시작")

    while is_running:

        try:

            if ugv_serial is None:

                time.sleep(0.05)
                continue

            if not ugv_serial.in_waiting:

                time.sleep(0.005)
                continue

            line = ugv_serial.readline().decode("utf-8", errors="ignore").strip()

            if not line:
                continue

            print("[UGV RX]", line)

            # ========================================================
            # T2100 ODOMETRY
            # ========================================================

            if line.startswith('{"T":2100'):

                try:

                    data = json.loads(line)

                    odl = float(data["odl"])

                    odr = float(data["odr"])

                except Exception as e:

                    print("[T2100 오류]", e)

                    continue

                with state_lock:

                    current_odl = odl
                    current_odr = odr

                    current_mode = mode

                    # =================================================
                    # 일반 DRIVE / STOP 거리 계산
                    # =================================================

                    if current_mode in ("DRIVE", "IDLE"):

                        if start_odl is None:
                            start_odl = odl

                        if start_odr is None:
                            start_odr = odr

                        delta_l = odl - start_odl

                        delta_r = odr - start_odr

                        segment_distance = max(0.0, ((delta_l + delta_r) / 2.0) / 100.0)

                        if current_mode == "DRIVE":

                            send_hc12(f"DISTANCE:" f"{segment_distance:.2f}")

                        elif stop_pending:

                            mission_outbound_total += segment_distance

                            return_drive_distance = mission_outbound_total

                            send_hc12(f"DISTANCE:" f"{segment_distance:.2f}")

                            send_hc12("STOPPED")

                            print("[STOP 최종거리]", f"{segment_distance:.2f} m")

                            print("[누적 전진거리]", f"{mission_outbound_total:.2f} m")

                            stop_pending = False

                    # =================================================
                    # RETURN_TURN
                    # 엔코더로 대략적인 회전
                    # =================================================

                    if current_mode == "RETURN_TURN":

                        if (
                            return_turn_start_odl is not None
                            and return_turn_start_odr is not None
                        ):

                            left_m = abs(odl - return_turn_start_odl) / 100.0

                            right_m = abs(odr - return_turn_start_odr) / 100.0

                            turn_distance = (left_m + right_m) / 2.0

                            print(
                                "[RETURN 회전]",
                                f"{turn_distance:.2f}",
                                "/",
                                f"{COARSE_TURN_DISTANCE_M:.2f} m",
                            )

                            if turn_distance >= COARSE_TURN_DISTANCE_M:

                                stop_ugv_now()

                                mode = "RETURN_ALIGN"

                                print("[RETURN] 대략적인 회전 완료")

                                print("[RETURN] 영상처리 차선 정렬 시작")

                    # =================================================
                    # RETURN_DRIVE
                    # =================================================

                    elif current_mode == "RETURN_DRIVE":

                        if (
                            return_drive_start_odl is not None
                            and return_drive_start_odr is not None
                        ):

                            left_m = abs(odl - return_drive_start_odl) / 100.0

                            right_m = abs(odr - return_drive_start_odr) / 100.0

                            returned_m = (left_m + right_m) / 2.0

                            print(
                                "[RETURN 거리]",
                                f"{returned_m:.2f}",
                                "/",
                                f"{return_drive_distance:.2f} m",
                            )

                            if returned_m >= return_drive_distance:

                                mode = "IDLE"

                                stop_ugv_now()

                                send_hc12("RETURNED")

                                paused_mission = False
                                paused_return_mode = None

                                print("[RETURN] 복귀 완료")

            # ========================================================
            # T2102 목표거리 도착
            # ========================================================

            elif '"T":2102' in line:

                with state_lock:

                    if mode != "DRIVE":
                        continue

                    mode = "IDLE"

                    stop_pending = False

                stop_ugv_now()

                try:

                    data = json.loads(line)

                    arrived_distance = float(data["distance"])

                    with state_lock:

                        mission_outbound_total += arrived_distance

                        return_drive_distance = mission_outbound_total

                        paused_mission = False

                    send_hc12(f"DISTANCE:" f"{arrived_distance:.2f}")

                    print("[도착] 마지막 구간:", f"{arrived_distance:.2f} m")

                    print("[도착] RETURN 저장거리:", f"{return_drive_distance:.2f} m")

                except Exception as e:

                    print("[T2102 거리 오류]", e)

                send_hc12("ARRIVED")

                print("[도착] ARRIVED - UGV STOP")

        except Exception as e:

            print("[UGV RX 오류]", e)

            time.sleep(0.05)


# ============================================================
# MOTOR CONTROL THREAD
# ============================================================


def control_thread_task():

    global lane_lost_printed

    global align_stable_count

    global mode

    global return_drive_start_odl
    global return_drive_start_odr

    previous_error = 0.0

    last_time = time.time()

    last_mode = None

    print("[알림] 모터 제어 스레드 시작")

    while is_running:

        current_time = time.time()

        dt = current_time - last_time

        if dt <= 0:
            dt = 0.001

        last_time = current_time

        with state_lock:

            current_mode = mode

            current_error = shared_error

            detected = lane_detected

            current_manual_left = manual_left_speed
            current_manual_right = manual_right_speed

        if current_mode != last_mode:

            previous_error = current_error

            last_mode = current_mode

        # ============================================================
        # IDLE
        # ============================================================

        if current_mode == "IDLE":

            with ugv_write_lock:

                ugv_control.send_driving_command(ugv_serial, 0.0, 0.0)

        # ============================================================
        # MANUAL
        # 카메라/PID 없이 좌/우 바퀴 직접 제어
        # ============================================================

        elif current_mode == "MANUAL":

            send_raw_ugv(
                '{"T":1,'
                f'"L":{current_manual_left:.3f},'
                f'"R":{current_manual_right:.3f}'
                '}'
            )

        # ============================================================
        # RETURN_TURN
        # ============================================================

        elif current_mode == "RETURN_TURN":

            # 좌우 바퀴를 반대 방향으로 돌려 제자리에서 180도 회전
            send_raw_ugv(
                '{"T":1,'
                f'"L":{-TURN_SPEED},'
                f'"R":{TURN_SPEED}'
                '}'
            )

        # ============================================================
        # RETURN_ALIGN
        #
        # 엔코더로 대충 반대 방향까지 돈 다음
        # 카메라로 실제 차선 중심을 맞춤
        # ============================================================

        elif current_mode == "RETURN_ALIGN":

            # --------------------------------------------------------
            # 차선 검출 성공
            # --------------------------------------------------------

            if detected:

                lane_lost_printed = False

                print("[RETURN ALIGN]", "error:", f"{current_error:.2f}")

                # 아직 중심과 차이가 큼
                if abs(current_error) > ALIGN_ERROR_THRESHOLD:

                    align_stable_count = 0

                    # 같은 방향으로 천천히 계속 회전
                    send_raw_ugv(
                        '{"T":1,'
                        f'"L":{-ALIGN_TURN_SPEED},'
                        f'"R":{ALIGN_TURN_SPEED}'
                        "}"
                    )

                # 중앙에 들어옴
                else:

                    stop_ugv_now()

                    align_stable_count += 1

                    print(
                        "[RETURN ALIGN] 안정화",
                        align_stable_count,
                        "/",
                        ALIGN_STABLE_FRAMES,
                    )

                    # 일정 프레임 연속으로 중앙
                    if align_stable_count >= ALIGN_STABLE_FRAMES:

                        with state_lock:

                            # 이 순간부터 복귀거리 측정
                            return_drive_start_odl = current_odl

                            return_drive_start_odr = current_odr

                            mode = "RETURN_DRIVE"

                            align_stable_count = 0

                        send_hc12("RETURN_DRIVING")

                        print("[RETURN] 영상 정렬 완료")

                        print("[RETURN] 차선추종 복귀 시작")

            # --------------------------------------------------------
            # 차선 아직 안 보임
            # --------------------------------------------------------

            else:

                align_stable_count = 0

                # 같은 방향으로 계속 천천히 회전하면서 탐색
                send_raw_ugv(
                    '{"T":1,' f'"L":{-ALIGN_TURN_SPEED},' f'"R":{ALIGN_TURN_SPEED}' "}"
                )

                if not lane_lost_printed:

                    print("[RETURN ALIGN] 차선 탐색 중...")

                    lane_lost_printed = True

        # ============================================================
        # DRIVE / RETURN_DRIVE
        # ============================================================

        elif current_mode in ("DRIVE", "RETURN_DRIVE"):

            if detected:

                angular_z = ugv_control.calculate_pid(current_error, previous_error, dt)

                previous_error = current_error

                with ugv_write_lock:

                    ugv_control.send_driving_command(ugv_serial, BASE_SPEED, angular_z)

                lane_lost_printed = False

            else:

                with ugv_write_lock:

                    ugv_control.send_driving_command(ugv_serial, 0.0, 0.0)

                if not lane_lost_printed:

                    print("[안전정지] 차선을 찾지 못했습니다.")

                    send_hc12("LANE_LOST")

                    lane_lost_printed = True

        time.sleep(0.02)


# ============================================================
# CAMERA
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


# ============================================================
# MAIN
# ============================================================

if __name__ == "__main__":

    try:

        # ============================================================
        # UGV
        # ============================================================

        print("[알림] UGV 연결 중...")

        ugv_serial = ugv_control.init_ugv()

        if not ugv_serial:

            print("[종료] UGV02 연결 실패")

            raise SystemExit

        print("[알림] UGV 연결 완료")

        # ============================================================
        # HC12
        # ============================================================

        print("[알림] HC-12 연결 중...")

        hc12_serial = serial.Serial(HC12_PORT, HC12_BAUD, timeout=0.05)

        print("[알림] HC12 연결:", HC12_PORT, HC12_BAUD)

        # ============================================================
        # CAMERA
        # ============================================================

        print("[알림] IMX219 CSI 카메라 초기화 중...")

        pipeline = gstreamer_pipeline(flip_method=0)

        cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

        if not cap.isOpened():

            print("[에러] 카메라를 열 수 없습니다.")

            stop_ugv_now()

            raise SystemExit

        print("[알림] 카메라 연결 완료")

        # ============================================================
        # THREADS
        # ============================================================

        hc12_thread = threading.Thread(target=hc12_thread_task, daemon=True)

        ugv_rx_thread = threading.Thread(target=ugv_receive_thread_task, daemon=True)

        control_thread = threading.Thread(target=control_thread_task, daemon=True)

        hc12_thread.start()
        ugv_rx_thread.start()
        control_thread.start()

        print()
        print("======================================")
        print(" Future Makers Smart Cone READY")
        print("======================================")
        print("[상태] IDLE")
        print("[알림] 앱 START 전에는 UGV가 움직이지 않습니다.")
        print()

        # ============================================================
        # CAMERA LOOP
        # ============================================================

        # lane_detection_ver1의 곡선 추적기: 이전 프레임의 차선과 폭을 기억
        tracker = lane_detection.LaneTracker()

        while cap.isOpened():

            ret, frame = cap.read()

            if not ret:

                print("[경고] 카메라 영상 수신 실패")

                with state_lock:

                    mode = "IDLE"

                stop_ugv_now()

                send_hc12("CAMERA_ERROR")

                break

            # 새 차선 인식: 색상/Canny 전처리 + 12개 수평 구간 + 2차 곡선 추적
            # 카메라 파이프라인과 스레드/통신/모터 제어는 기존 main 그대로 유지
            _raw_edges, lane_edges, top, bottom = lane_detection.get_edges(
                frame, return_raw=True
            )
            pair, status, inferred = tracker.update(
                lane_edges, top, bottom, time.monotonic()
            )
            result_image, error, is_detected = lane_detection.draw_result(
                frame.copy(), pair, status, inferred, top, bottom
            )

            with state_lock:

                shared_error = error

                lane_detected = is_detected

            # SSH 환경에서는 GUI 끔
            #
            # 필요하면:
            #
            # cv2.imshow(
            #     "Smart Cone",
            #     result_image
            # )
            #
            # if cv2.waitKey(1) & 0xFF == ord("q"):
            #     break

    except KeyboardInterrupt:

        print()

        print("[알림] Ctrl+C 감지")

    except Exception as e:

        print("[MAIN 오류]", e)

    finally:

        print("[알림] 시스템 안전 종료")

        is_running = False

        try:

            stop_ugv_now()

        except Exception:
            pass

        try:

            if "cap" in locals():

                cap.release()

        except Exception:
            pass

        try:

            if hc12_serial:

                hc12_serial.close()

        except Exception:
            pass

        try:

            if ugv_serial:

                ugv_serial.close()

        except Exception:
            pass

        cv2.destroyAllWindows()

        print("[종료] 완료")

