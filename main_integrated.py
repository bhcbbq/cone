import cv2
import time
import threading
import json
import serial
import numpy as np

import lane_detection
import ugv_control


# ============================================================
# SETTINGS
# ============================================================

BASE_SPEED = 0.30

# 대략적인 180도 회전 속도
TURN_SPEED = 0.25

# 영상 정렬 시 천천히 회전
ALIGN_TURN_SPEED = 0.18

# 기존 약 0.50 m가 180도라고 가정했을 때
# 일부러 조금 덜 돌고 영상처리로 마무리
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

            hc12_serial.write(
                message.encode("utf-8")
            )

            hc12_serial.flush()

    except Exception as e:

        print(
            "[HC12 TX 오류]",
            e
        )


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

            ugv_serial.write(
                message.encode("utf-8")
            )

            ugv_serial.flush()

    except Exception as e:

        print(
            "[UGV TX 오류]",
            e
        )


# ============================================================
# UGV STOP
# ============================================================

def stop_ugv_now():

    send_raw_ugv(
        '{"T":1,"L":0.0,"R":0.0}'
    )


# ============================================================
# HC-12 COMMAND THREAD
# ============================================================

def hc12_thread_task():

    global mode

    global start_odl
    global start_odr

    global stop_pending
    global paused_mission

    global mission_outbound_total
    global return_drive_distance

    global return_turn_start_odl
    global return_turn_start_odr

    global align_stable_count

    buffer = ""

    print(
        "[알림] HC-12 명령 수신 스레드 시작"
    )

    while is_running:

        try:

            if hc12_serial is None:

                time.sleep(0.05)
                continue

            if not hc12_serial.in_waiting:

                time.sleep(0.005)
                continue

            raw = hc12_serial.read(
                hc12_serial.in_waiting
            )

            text = raw.decode(
                "utf-8",
                errors="ignore"
            )

            for ch in text:

                if ch == "\n":

                    cmd = buffer.strip()
                    buffer = ""

                    if not cmd:
                        continue

                    print(
                        "[HC12 RX]",
                        cmd
                    )


                    # ==================================================
                    # EMERGENCY STOP
                    # ==================================================

                    if cmd == "EMERGENCY_STOP":

                        with state_lock:

                            mode = "IDLE"
                            stop_pending = False
                            align_stable_count = 0

                        send_raw_ugv(
                            '{"T":0}'
                        )

                        send_hc12(
                            "EMERGENCY"
                        )

                        print(
                            "[비상정지] EMERGENCY STOP"
                        )

                        continue


                    # ==================================================
                    # STOP
                    # ==================================================

                    if cmd == "STOP":

                        with state_lock:

                            mode = "IDLE"

                            stop_pending = True

                            paused_mission = True

                            align_stable_count = 0

                        stop_ugv_now()

                        print(
                            "[정지] UGV STOP"
                        )

                        print(
                            "[정지] 최종 엔코더 거리 대기"
                        )

                        continue


                    # ==================================================
                    # TARGET
                    # ==================================================

                    if cmd.startswith("TARGET:"):

                        try:

                            distance = float(
                                cmd.split(
                                    ":",
                                    1
                                )[1]
                            )

                            if distance <= 0:

                                print(
                                    "[오류] INVALID TARGET"
                                )

                                send_hc12(
                                    "ERROR:INVALID_TARGET"
                                )

                                continue


                            with state_lock:

                                if not paused_mission:

                                    mission_outbound_total = 0.0
                                    return_drive_distance = 0.0

                                start_odl = None
                                start_odr = None

                                stop_pending = False


                            send_raw_ugv(
                                '{"T":2101,"d":'
                                + str(distance)
                                + '}'
                            )

                            send_hc12(
                                f"TARGET:{distance:.2f}"
                            )

                            print(
                                "[목표거리]",
                                f"{distance:.2f} m"
                            )


                        except ValueError:

                            print(
                                "[오류] INVALID TARGET:",
                                cmd
                            )

                            send_hc12(
                                "ERROR:INVALID_TARGET"
                            )

                        continue


                    # ==================================================
                    # START
                    # ==================================================

                    if cmd == "START":

                        with state_lock:

                            mode = "DRIVE"
                            stop_pending = False
                            align_stable_count = 0

                        send_hc12(
                            "DRIVING"
                        )

                        print(
                            "[주행] START"
                        )

                        print(
                            "[주행] 카메라 차선추종 시작"
                        )

                        continue


                    # ==================================================
                    # RETURN
                    # ==================================================

                    if cmd == "RETURN":

                        stop_ugv_now()

                        time.sleep(0.2)

                        with state_lock:

                            if (
                                current_odl is None
                                or
                                current_odr is None
                            ):

                                print(
                                    "[RETURN 오류] 엔코더 데이터 없음"
                                )

                                send_hc12(
                                    "RETURN_ERROR:NO_ODOM"
                                )

                                continue


                            if return_drive_distance <= 0:

                                print(
                                    "[RETURN 오류] 저장된 복귀거리 없음"
                                )

                                send_hc12(
                                    "RETURN_ERROR:NO_DISTANCE"
                                )

                                continue


                            return_turn_start_odl = (
                                current_odl
                            )

                            return_turn_start_odr = (
                                current_odr
                            )

                            align_stable_count = 0

                            mode = "RETURN_TURN"

                            stop_pending = False


                        send_hc12(
                            "RETURNING"
                        )

                        print(
                            "[RETURN] 대략적인 180도 회전 시작"
                        )

                        print(
                            "[RETURN] 복귀거리:",
                            f"{return_drive_distance:.2f} m"
                        )

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

                        send_hc12(
                            "DISTANCE:0.00"
                        )

                        print(
                            "[거리] RESET"
                        )

                        continue


                    print(
                        "[HC12] UNKNOWN COMMAND:",
                        cmd
                    )


                elif ch != "\r":

                    buffer += ch


        except Exception as e:

            print(
                "[HC12 RX 오류]",
                e
            )

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

    print(
        "[알림] UGV 수신 스레드 시작"
    )


    while is_running:

        try:

            if ugv_serial is None:

                time.sleep(0.05)
                continue


            if not ugv_serial.in_waiting:

                time.sleep(0.005)
                continue


            line = (
                ugv_serial
                .readline()
                .decode(
                    "utf-8",
                    errors="ignore"
                )
                .strip()
            )


            if not line:
                continue


            print(
                "[UGV RX]",
                line
            )


            # ========================================================
            # T2100 ODOMETRY
            # ========================================================

            if line.startswith(
                '{"T":2100'
            ):

                try:

                    data = json.loads(
                        line
                    )

                    odl = float(
                        data["odl"]
                    )

                    odr = float(
                        data["odr"]
                    )

                except Exception as e:

                    print(
                        "[T2100 오류]",
                        e
                    )

                    continue


                with state_lock:

                    current_odl = odl
                    current_odr = odr

                    current_mode = mode


                    # =================================================
                    # 일반 DRIVE / STOP 거리 계산
                    # =================================================

                    if current_mode in (
                        "DRIVE",
                        "IDLE"
                    ):

                        if start_odl is None:
                            start_odl = odl

                        if start_odr is None:
                            start_odr = odr


                        delta_l = (
                            odl
                            -
                            start_odl
                        )

                        delta_r = (
                            odr
                            -
                            start_odr
                        )


                        segment_distance = max(
                            0.0,
                            (
                                (
                                    delta_l
                                    +
                                    delta_r
                                )
                                / 2.0
                            )
                            / 100.0
                        )


                        if current_mode == "DRIVE":

                            send_hc12(
                                f"DISTANCE:"
                                f"{segment_distance:.2f}"
                            )


                        elif stop_pending:

                            mission_outbound_total += (
                                segment_distance
                            )

                            return_drive_distance = (
                                mission_outbound_total
                            )

                            send_hc12(
                                f"DISTANCE:"
                                f"{segment_distance:.2f}"
                            )

                            send_hc12(
                                "STOPPED"
                            )

                            print(
                                "[STOP 최종거리]",
                                f"{segment_distance:.2f} m"
                            )

                            print(
                                "[누적 전진거리]",
                                f"{mission_outbound_total:.2f} m"
                            )

                            stop_pending = False


                    # =================================================
                    # RETURN_TURN
                    # 엔코더로 대략적인 회전
                    # =================================================

                    if current_mode == "RETURN_TURN":

                        if (
                            return_turn_start_odl
                            is not None
                            and
                            return_turn_start_odr
                            is not None
                        ):

                            left_m = abs(
                                odl
                                -
                                return_turn_start_odl
                            ) / 100.0

                            right_m = abs(
                                odr
                                -
                                return_turn_start_odr
                            ) / 100.0


                            turn_distance = (
                                left_m
                                +
                                right_m
                            ) / 2.0


                            print(
                                "[RETURN 회전]",
                                f"{turn_distance:.2f}",
                                "/",
                                f"{COARSE_TURN_DISTANCE_M:.2f} m"
                            )


                            if (
                                turn_distance
                                >=
                                COARSE_TURN_DISTANCE_M
                            ):

                                stop_ugv_now()

                                mode = "RETURN_ALIGN"

                                print(
                                    "[RETURN] 대략적인 회전 완료"
                                )

                                print(
                                    "[RETURN] 영상처리 차선 정렬 시작"
                                )


                    # =================================================
                    # RETURN_DRIVE
                    # =================================================

                    elif current_mode == "RETURN_DRIVE":

                        if (
                            return_drive_start_odl
                            is not None
                            and
                            return_drive_start_odr
                            is not None
                        ):

                            left_m = abs(
                                odl
                                -
                                return_drive_start_odl
                            ) / 100.0

                            right_m = abs(
                                odr
                                -
                                return_drive_start_odr
                            ) / 100.0


                            returned_m = (
                                left_m
                                +
                                right_m
                            ) / 2.0


                            print(
                                "[RETURN 거리]",
                                f"{returned_m:.2f}",
                                "/",
                                f"{return_drive_distance:.2f} m"
                            )


                            if (
                                returned_m
                                >=
                                return_drive_distance
                            ):

                                mode = "IDLE"

                                stop_ugv_now()

                                send_hc12(
                                    "RETURNED"
                                )

                                paused_mission = False

                                print(
                                    "[RETURN] 복귀 완료"
                                )


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

                    data = json.loads(
                        line
                    )

                    arrived_distance = float(
                        data["distance"]
                    )


                    with state_lock:

                        mission_outbound_total += (
                            arrived_distance
                        )

                        return_drive_distance = (
                            mission_outbound_total
                        )

                        paused_mission = False


                    send_hc12(
                        f"DISTANCE:"
                        f"{arrived_distance:.2f}"
                    )


                    print(
                        "[도착] 마지막 구간:",
                        f"{arrived_distance:.2f} m"
                    )

                    print(
                        "[도착] RETURN 저장거리:",
                        f"{return_drive_distance:.2f} m"
                    )


                except Exception as e:

                    print(
                        "[T2102 거리 오류]",
                        e
                    )


                send_hc12(
                    "ARRIVED"
                )

                print(
                    "[도착] ARRIVED - UGV STOP"
                )


        except Exception as e:

            print(
                "[UGV RX 오류]",
                e
            )

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


    print(
        "[알림] 모터 제어 스레드 시작"
    )


    while is_running:

        current_time = time.time()

        dt = (
            current_time
            -
            last_time
        )

        if dt <= 0:
            dt = 0.001

        last_time = current_time


        with state_lock:

            current_mode = mode

            current_error = shared_error

            detected = lane_detected


        if current_mode != last_mode:

            previous_error = (
                current_error
            )

            last_mode = (
                current_mode
            )


        # ============================================================
        # IDLE
        # ============================================================

        if current_mode == "IDLE":

            with ugv_write_lock:

                ugv_control.send_driving_command(
                    ugv_serial,
                    0.0,
                    0.0
                )


        # ============================================================
        # RETURN_TURN
        # ============================================================

        elif current_mode == "RETURN_TURN":

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

                print(
                    "[RETURN ALIGN]",
                    "error:",
                    f"{current_error:.2f}"
                )


                # 아직 중심과 차이가 큼
                if (
                    abs(current_error)
                    >
                    ALIGN_ERROR_THRESHOLD
                ):

                    align_stable_count = 0

                    # 같은 방향으로 천천히 계속 회전
                    send_raw_ugv(
                        '{"T":1,'
                        f'"L":{-ALIGN_TURN_SPEED},'
                        f'"R":{ALIGN_TURN_SPEED}'
                        '}'
                    )


                # 중앙에 들어옴
                else:

                    stop_ugv_now()

                    align_stable_count += 1


                    print(
                        "[RETURN ALIGN] 안정화",
                        align_stable_count,
                        "/",
                        ALIGN_STABLE_FRAMES
                    )


                    # 일정 프레임 연속으로 중앙
                    if (
                        align_stable_count
                        >=
                        ALIGN_STABLE_FRAMES
                    ):

                        with state_lock:

                            # 이 순간부터 복귀거리 측정
                            return_drive_start_odl = (
                                current_odl
                            )

                            return_drive_start_odr = (
                                current_odr
                            )

                            mode = "RETURN_DRIVE"

                            align_stable_count = 0


                        send_hc12(
                            "RETURN_DRIVING"
                        )


                        print(
                            "[RETURN] 영상 정렬 완료"
                        )

                        print(
                            "[RETURN] 차선추종 복귀 시작"
                        )


            # --------------------------------------------------------
            # 차선 아직 안 보임
            # --------------------------------------------------------

            else:

                align_stable_count = 0


                # 같은 방향으로 계속 천천히 회전하면서 탐색
                send_raw_ugv(
                    '{"T":1,'
                    f'"L":{-ALIGN_TURN_SPEED},'
                    f'"R":{ALIGN_TURN_SPEED}'
                    '}'
                )


                if not lane_lost_printed:

                    print(
                        "[RETURN ALIGN] 차선 탐색 중..."
                    )

                    lane_lost_printed = True


        # ============================================================
        # DRIVE / RETURN_DRIVE
        # ============================================================

        elif current_mode in (
            "DRIVE",
            "RETURN_DRIVE"
        ):

            if detected:

                angular_z = (
                    ugv_control.calculate_pid(
                        current_error,
                        previous_error,
                        dt
                    )
                )


                previous_error = (
                    current_error
                )


                with ugv_write_lock:

                    ugv_control.send_driving_command(
                        ugv_serial,
                        BASE_SPEED,
                        angular_z
                    )


                lane_lost_printed = False


            else:

                with ugv_write_lock:

                    ugv_control.send_driving_command(
                        ugv_serial,
                        0.0,
                        0.0
                    )


                if not lane_lost_printed:

                    print(
                        "[안전정지] 차선을 찾지 못했습니다."
                    )

                    send_hc12(
                        "LANE_LOST"
                    )

                    lane_lost_printed = True


        time.sleep(
            0.02
        )


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

        print(
            "[알림] UGV 연결 중..."
        )

        ugv_serial = (
            ugv_control.init_ugv()
        )

        if not ugv_serial:

            print(
                "[종료] UGV02 연결 실패"
            )

            raise SystemExit

        print(
            "[알림] UGV 연결 완료"
        )


        # ============================================================
        # HC12
        # ============================================================

        print(
            "[알림] HC-12 연결 중..."
        )

        hc12_serial = serial.Serial(
            HC12_PORT,
            HC12_BAUD,
            timeout=0.05
        )

        print(
            "[알림] HC12 연결:",
            HC12_PORT,
            HC12_BAUD
        )


        # ============================================================
        # CAMERA
        # ============================================================

        print(
            "[알림] IMX219 CSI 카메라 초기화 중..."
        )

        pipeline = (
            gstreamer_pipeline(
                flip_method=0
            )
        )

        cap = cv2.VideoCapture(
            pipeline,
            cv2.CAP_GSTREAMER
        )

        if not cap.isOpened():

            print(
                "[에러] 카메라를 열 수 없습니다."
            )

            stop_ugv_now()

            raise SystemExit


        print(
            "[알림] 카메라 연결 완료"
        )


        # ============================================================
        # THREADS
        # ============================================================

        hc12_thread = threading.Thread(
            target=hc12_thread_task,
            daemon=True
        )

        ugv_rx_thread = threading.Thread(
            target=ugv_receive_thread_task,
            daemon=True
        )

        control_thread = threading.Thread(
            target=control_thread_task,
            daemon=True
        )


        hc12_thread.start()
        ugv_rx_thread.start()
        control_thread.start()


        print()
        print(
            "======================================"
        )
        print(
            " Future Makers Smart Cone READY"
        )
        print(
            "======================================"
        )
        print(
            "[상태] IDLE"
        )
        print(
            "[알림] 앱 START 전에는 UGV가 움직이지 않습니다."
        )
        print()


        # ============================================================
        # CAMERA LOOP
        # ============================================================

        while cap.isOpened():

            ret, frame = (
                cap.read()
            )


            if not ret:

                print(
                    "[경고] 카메라 영상 수신 실패"
                )

                with state_lock:

                    mode = "IDLE"

                stop_ugv_now()

                send_hc12(
                    "CAMERA_ERROR"
                )

                break


            height, width = (
                frame.shape[:2]
            )


            # GRAY
            gray = cv2.cvtColor(
                frame,
                cv2.COLOR_BGR2GRAY
            )


            # BLUR
            blur = cv2.GaussianBlur(
                gray,
                (5, 5),
                0
            )


            # CANNY
            edges = cv2.Canny(
                blur,
                50,
                150
            )


            # ROI
            roi_vertices = [

                (
                    0,
                    height
                ),

                (
                    int(
                        width * 0.2
                    ),
                    int(
                        height * 0.45
                    )
                ),

                (
                    int(
                        width * 0.8
                    ),
                    int(
                        height * 0.45
                    )
                ),

                (
                    width,
                    height
                ),
            ]


            cropped_edges = (
                lane_detection.region_of_interest(
                    edges,
                    np.array(
                        [roi_vertices],
                        np.int32
                    )
                )
            )


            # HOUGH
            lines = cv2.HoughLinesP(

                cropped_edges,

                rho=1,

                theta=np.pi / 180,

                threshold=40,

                minLineLength=20,

                maxLineGap=10
            )


            # STEERING
            (
                error,
                result_image,
                is_detected

            ) = lane_detection.calculate_steering(

                frame.copy(),
                lines
            )


            with state_lock:

                shared_error = error

                lane_detected = (
                    is_detected
                )


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

        print(
            "[알림] Ctrl+C 감지"
        )


    except Exception as e:

        print(
            "[MAIN 오류]",
            e
        )


    finally:

        print(
            "[알림] 시스템 안전 종료"
        )

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

        print(
            "[종료] 완료"
        )