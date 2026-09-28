import serial
import threading
import time
import json
import math


# =========================
# SERIAL SETTINGS
# =========================

HC12_PORT = '/dev/ttyTHS1'
HC12_BAUD = 115200

UGV_PORT = '/dev/ttyACM0'
UGV_BAUD = 115200


# =========================
# DRIVE SETTINGS
# =========================

DRIVE_SPEED = 0.5
TURN_SPEED = 0.35

# UGV02 좌우 바퀴 중심 간 거리
TRACK_WIDTH_M = 0.172

# 출발점 몇 cm 이내면 복귀 완료로 판단
RETURN_STOP_RADIUS_M = 0.03

# 너무 작은 회전은 생략
MIN_TURN_ANGLE_RAD = math.radians(3.0)


# =========================
# OPEN SERIAL
# =========================

hc12 = serial.Serial(
    HC12_PORT,
    HC12_BAUD,
    timeout=0.05
)

ugv = serial.Serial(
    UGV_PORT,
    UGV_BAUD,
    timeout=0.05
)

print("BRIDGE READY")
print("HC12 :", HC12_PORT, HC12_BAUD)
print("UGV  :", UGV_PORT, UGV_BAUD)


# =========================
# GLOBAL STATE
# =========================

running = False
stop_flag = False
stop_pending = False

start_odl = None
start_odr = None

target_distance = None

# STOP 후 다시 START해도
# 앞에서 이동한 거리를 잃지 않도록 누적
outbound_distance_m = 0.0

# STOP 뒤 다음 TARGET은
# 같은 임무를 이어가는 것으로 판단
paused_mission = False


# =========================
# 2D ODOMETRY
# =========================
#
# 시작점:
# x = 0
# y = 0
# theta = 0
#
# 나중에 장애물 회피를 추가하면
# 단순 "총 이동거리"가 아니라
# 실제 추정 위치를 이용할 수 있도록 함.
#

pose_x = 0.0
pose_y = 0.0
pose_theta = 0.0

pose_last_odl = None
pose_last_odr = None
pose_initialized = False

current_odl = None
current_odr = None


# =========================
# RETURN STATE
# =========================

# None / TURN / DRIVE
return_phase = None

# +1 = 좌회전(CCW)
# -1 = 우회전(CW)
return_turn_direction = 0

turn_start_odl = None
turn_start_odr = None
turn_target_wheel_m = 0.0

return_drive_start_odl = None
return_drive_start_odr = None
return_drive_target_m = 0.0


# =========================
# HELPERS
# =========================

def send_ugv(text):

    ugv.write(
        (text + '\n').encode('utf-8')
    )

    ugv.flush()


def send_hc12(text):

    hc12.write(
        (text + '\n').encode('utf-8')
    )

    hc12.flush()


def normalize_angle(angle):

    while angle > math.pi:
        angle -= 2.0 * math.pi

    while angle < -math.pi:
        angle += 2.0 * math.pi

    return angle


def reset_pose():

    global pose_x
    global pose_y
    global pose_theta

    global pose_last_odl
    global pose_last_odr
    global pose_initialized

    pose_x = 0.0
    pose_y = 0.0
    pose_theta = 0.0

    pose_last_odl = None
    pose_last_odr = None

    pose_initialized = False


def update_pose(odl_cm, odr_cm):

    global pose_x
    global pose_y
    global pose_theta

    global pose_last_odl
    global pose_last_odr
    global pose_initialized

    # 첫 T2100은 기준값으로만 저장
    if not pose_initialized:

        pose_last_odl = odl_cm
        pose_last_odr = odr_cm

        pose_initialized = True

        return


    # T2100 odl/odr은 cm
    dl = (
        odl_cm - pose_last_odl
    ) / 100.0

    dr = (
        odr_cm - pose_last_odr
    ) / 100.0


    pose_last_odl = odl_cm
    pose_last_odr = odr_cm


    # 중심 이동거리
    dc = (
        dl + dr
    ) / 2.0


    # 회전각
    dtheta = (
        dr - dl
    ) / TRACK_WIDTH_M


    # 회전 중간 각도를 이용해서
    # x, y 적분
    mid_theta = (
        pose_theta
        + dtheta / 2.0
    )


    pose_x += (
        dc
        * math.cos(mid_theta)
    )

    pose_y += (
        dc
        * math.sin(mid_theta)
    )


    pose_theta = normalize_angle(
        pose_theta + dtheta
    )


# =========================
# RETURN START
# =========================

def begin_return():

    global running
    global stop_pending

    global return_phase
    global return_turn_direction

    global turn_start_odl
    global turn_start_odr
    global turn_target_wheel_m

    global return_drive_start_odl
    global return_drive_start_odr
    global return_drive_target_m


    running = False
    stop_pending = False


    if (
        current_odl is None
        or current_odr is None
    ):

        print(
            "RETURN ERROR: NO ODOMETRY"
        )

        send_hc12(
            "RETURN_ERROR"
        )

        return


    # 현재 위치에서 출발점까지 직선거리
    home_distance = math.hypot(
        pose_x,
        pose_y
    )


    # 이미 출발점 근처면 종료
    if (
        home_distance
        <= RETURN_STOP_RADIUS_M
    ):

        send_ugv(
            '{"T":1,"L":0.0,"R":0.0}'
        )

        send_hc12(
            "RETURNED"
        )

        print(
            "ALREADY AT START POINT"
        )

        return_phase = None

        return


    # 현재 위치에서
    # 원점 방향을 바라보는 목표각
    desired_heading = math.atan2(
        -pose_y,
        -pose_x
    )


    # 현재 방향에서
    # 얼마나 돌아야 하는지
    turn_angle = normalize_angle(
        desired_heading
        - pose_theta
    )


    print(
        "=============================="
    )

    print(
        "RETURN START"
    )

    print(
        "POSE:",
        f"x={pose_x:.3f}",
        f"y={pose_y:.3f}",
        f"theta="
        f"{math.degrees(pose_theta):.1f} deg"
    )

    print(
        "HOME DISTANCE:",
        f"{home_distance:.3f} m"
    )

    print(
        "TURN ANGLE:",
        f"{math.degrees(turn_angle):.1f} deg"
    )

    print(
        "=============================="
    )


    send_hc12(
        "RETURNING"
    )


    # 이미 거의 출발점 방향이면
    # 회전 없이 바로 복귀
    if (
        abs(turn_angle)
        < MIN_TURN_ANGLE_RAD
    ):

        return_phase = "DRIVE"

        return_drive_start_odl = (
            current_odl
        )

        return_drive_start_odr = (
            current_odr
        )

        return_drive_target_m = (
            home_distance
        )

        print(
            "RETURN DRIVE TARGET:",
            f"{return_drive_target_m:.3f}",
            "m"
        )

        return


    # =========================
    # TURN 준비
    # =========================

    return_phase = "TURN"

    turn_start_odl = current_odl
    turn_start_odr = current_odr


    # 제자리 회전 시
    # 각 바퀴가 움직여야 하는 거리
    #
    # s = theta * track_width / 2
    #
    turn_target_wheel_m = (
        abs(turn_angle)
        * TRACK_WIDTH_M
        / 2.0
    )


    if turn_angle > 0:

        return_turn_direction = 1

    else:

        return_turn_direction = -1


    print(
        "TURN WHEEL TARGET:",
        f"{turn_target_wheel_m:.3f}",
        "m"
    )


# =========================
# HC12 RECEIVE THREAD
# =========================

def hc12_receive():

    global running
    global stop_flag
    global stop_pending

    global start_odl
    global start_odr

    global target_distance
    global outbound_distance_m

    global paused_mission
    global return_phase


    buffer = ""


    while not stop_flag:

        if hc12.in_waiting:

            data = hc12.read(
                hc12.in_waiting
            )

            text = data.decode(
                'utf-8',
                errors='ignore'
            )


            for c in text:

                if c == '\n':

                    cmd = buffer.strip()

                    buffer = ""


                    if not cmd:
                        continue


                    print(
                        "\nHC12 RX:",
                        cmd
                    )


                    # =====================
                    # START
                    # =====================

                    if cmd == "START":

                        if (
                            return_phase
                            is not None
                        ):

                            print(
                                "START IGNORED:"
                                " RETURN IN PROGRESS"
                            )

                            continue


                        stop_pending = False

                        running = True


                        print(
                            "UGV START"
                        )


                        send_hc12(
                            "DRIVING"
                        )


                    # =====================
                    # STOP
                    # =====================

                    elif cmd == "STOP":

                        running = False


                        # RETURN 중 STOP
                        if (
                            return_phase
                            is not None
                        ):

                            return_phase = None

                            send_ugv(
                                '{"T":1,'
                                '"L":0.0,'
                                '"R":0.0}'
                            )

                            print(
                                "RETURN STOPPED"
                            )

                            send_hc12(
                                "STOPPED"
                            )

                            continue


                        stop_pending = True
                        paused_mission = True


                        send_ugv(
                            '{"T":1,'
                            '"L":0.0,'
                            '"R":0.0}'
                        )


                        print(
                            "UGV STOP"
                        )

                        print(
                            "WAITING FOR "
                            "FINAL STOP DISTANCE..."
                        )


                    # =====================
                    # EMERGENCY STOP
                    # =====================

                    elif (
                        cmd
                        == "EMERGENCY_STOP"
                    ):

                        running = False
                        stop_pending = False
                        return_phase = None


                        send_ugv(
                            '{"T":0}'
                        )


                        print(
                            "UGV EMERGENCY STOP"
                        )


                        send_hc12(
                            "EMERGENCY"
                        )


                    # =====================
                    # RETURN
                    # =====================

                    elif cmd == "RETURN":

                        begin_return()


                    # =====================
                    # TARGET
                    # =====================

                    elif cmd.startswith(
                        "TARGET:"
                    ):

                        try:

                            distance = float(
                                cmd.split(
                                    ":",
                                    1
                                )[1]
                            )


                            if distance <= 0:

                                print(
                                    "INVALID TARGET"
                                )

                                continue


                            # =====================
                            # 새 임무
                            # =====================

                            if not paused_mission:

                                outbound_distance_m = (
                                    0.0
                                )

                                reset_pose()


                            # =====================
                            # STOP 후 이어서 주행
                            # =====================

                            paused_mission = False

                            stop_pending = False

                            return_phase = None


                            target_distance = (
                                distance
                            )


                            # 이번 구간 거리 기준점
                            start_odl = None
                            start_odr = None


                            message = (
                                '{"T":2101,"d":'
                                + f"{distance:.3f}"
                                + '}'
                            )


                            send_ugv(
                                message
                            )


                            print(
                                "TARGET SENT:",
                                distance,
                                "m"
                            )


                            send_hc12(
                                f"TARGET:"
                                f"{distance:.2f}"
                            )


                        except ValueError:

                            print(
                                "INVALID TARGET:",
                                cmd
                            )


                    # =====================
                    # RESET DISTANCE
                    # =====================

                    elif (
                        cmd
                        == "RESET_DISTANCE"
                    ):

                        running = False

                        stop_pending = False

                        paused_mission = False

                        return_phase = None


                        start_odl = None
                        start_odr = None

                        target_distance = None


                        outbound_distance_m = 0.0


                        reset_pose()


                        print(
                            "DISTANCE RESET"
                        )


                        send_hc12(
                            "DISTANCE:0.00"
                        )


                    else:

                        print(
                            "UNKNOWN COMMAND:",
                            cmd
                        )


                elif c != '\r':

                    buffer += c


        time.sleep(
            0.001
        )


# =========================
# START HC12 THREAD
# =========================

thread = threading.Thread(
    target=hc12_receive,
    daemon=True
)

thread.start()


# =========================
# MAIN LOOP
# =========================

try:

    while True:


        # =========================
        # MOTOR COMMAND
        # =========================

        if (
            return_phase
            == "TURN"
        ):

            # 왼쪽으로 회전
            if (
                return_turn_direction
                > 0
            ):

                send_ugv(
                    '{"T":1,'
                    f'"L":{-TURN_SPEED},'
                    f'"R":{TURN_SPEED}'
                    '}'
                )


            # 오른쪽으로 회전
            else:

                send_ugv(
                    '{"T":1,'
                    f'"L":{TURN_SPEED},'
                    f'"R":{-TURN_SPEED}'
                    '}'
                )


        elif (
            return_phase
            == "DRIVE"
        ):

            send_ugv(
                '{"T":1,'
                f'"L":{DRIVE_SPEED},'
                f'"R":{DRIVE_SPEED}'
                '}'
            )


        elif running:

            # heartbeat 때문에
            # 계속 보내야 함
            send_ugv(
                '{"T":1,'
                f'"L":{DRIVE_SPEED},'
                f'"R":{DRIVE_SPEED}'
                '}'
            )


        # =========================
        # UGV RECEIVE
        # =========================

        while ugv.in_waiting:

            line = ugv.readline().decode(
                'utf-8',
                errors='ignore'
            ).strip()


            if not line:
                continue


            print(
                "UGV RX:",
                line
            )


            # =====================
            # T2100
            # =====================

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


                    current_odl = odl
                    current_odr = odr


                    # =====================
                    # 2D ODOMETRY
                    # =====================

                    update_pose(
                        odl,
                        odr
                    )


                    # =====================
                    # NORMAL DRIVE
                    # =====================

                    if (
                        return_phase
                        is None
                    ):

                        if (
                            start_odl
                            is None
                        ):

                            start_odl = odl


                        if (
                            start_odr
                            is None
                        ):

                            start_odr = odr


                        delta_l = (
                            odl
                            - start_odl
                        )

                        delta_r = (
                            odr
                            - start_odr
                        )


                        distance_cm = (
                            delta_l
                            + delta_r
                        ) / 2.0


                        distance_m = (
                            distance_cm
                            / 100.0
                        )


                        if distance_m < 0:

                            distance_m = 0.0


                        # =====================
                        # 주행 중
                        # =====================

                        if running:

                            send_hc12(
                                f"DISTANCE:"
                                f"{distance_m:.2f}"
                            )


                            print(
                                "CURRENT DISTANCE:",
                                f"{distance_m:.2f}",
                                "m"
                            )


                        # =====================
                        # STOP 직후
                        # =====================

                        elif stop_pending:

                            send_hc12(
                                f"DISTANCE:"
                                f"{distance_m:.2f}"
                            )


                            print(
                                "STOP DISTANCE:",
                                f"{distance_m:.2f}",
                                "m"
                            )


                            # 이번 구간 주행거리 누적
                            outbound_distance_m += (
                                distance_m
                            )


                            print(
                                "OUTBOUND TOTAL:",
                                f"{outbound_distance_m:.2f}",
                                "m"
                            )


                            send_hc12(
                                "STOPPED"
                            )


                            print(
                                "STOPPED SENT"
                            )


                            stop_pending = False


                    # =====================
                    # RETURN TURN
                    # =====================

                    elif (
                        return_phase
                        == "TURN"
                    ):

                        turn_left_m = abs(
                            odl
                            - turn_start_odl
                        ) / 100.0


                        turn_right_m = abs(
                            odr
                            - turn_start_odr
                        ) / 100.0


                        turn_m = (
                            turn_left_m
                            + turn_right_m
                        ) / 2.0


                        print(
                            "RETURN TURN:",
                            f"{turn_m:.3f}",
                            "/",
                            f"{turn_target_wheel_m:.3f}",
                            "m"
                        )


                        if (
                            turn_m
                            >= turn_target_wheel_m
                        ):

                            send_ugv(
                                '{"T":1,'
                                '"L":0.0,'
                                '"R":0.0}'
                            )


                            # 실제 회전 후
                            # 현재 위치 기준으로
                            # 출발점까지 거리 다시 계산
                            home_distance = (
                                math.hypot(
                                    pose_x,
                                    pose_y
                                )
                            )


                            return_drive_start_odl = (
                                odl
                            )

                            return_drive_start_odr = (
                                odr
                            )

                            return_drive_target_m = (
                                home_distance
                            )


                            return_phase = (
                                "DRIVE"
                            )


                            print(
                                "TURN COMPLETE"
                            )


                            print(
                                "RETURN DRIVE TARGET:",
                                f"{return_drive_target_m:.3f}",
                                "m"
                            )


                    # =====================
                    # RETURN DRIVE
                    # =====================

                    elif (
                        return_phase
                        == "DRIVE"
                    ):

                        # 현재 위치에서
                        # 시작점까지 남은 거리
                        home_distance = (
                            math.hypot(
                                pose_x,
                                pose_y
                            )
                        )


                        drive_left_m = abs(
                            odl
                            - return_drive_start_odl
                        ) / 100.0


                        drive_right_m = abs(
                            odr
                            - return_drive_start_odr
                        ) / 100.0


                        drive_m = (
                            drive_left_m
                            + drive_right_m
                        ) / 2.0


                        send_hc12(
                            f"RETURN_DISTANCE:"
                            f"{home_distance:.2f}"
                        )


                        print(
                            "RETURN:",
                            f"driven={drive_m:.2f}",
                            f"home={home_distance:.2f}"
                        )


                        # 출발점 근처에 왔거나
                        # 계산된 복귀거리 이상
                        # 이동하면 정지
                        if (
                            home_distance
                            <= RETURN_STOP_RADIUS_M
                            or
                            drive_m
                            >= return_drive_target_m
                        ):

                            send_ugv(
                                '{"T":1,'
                                '"L":0.0,'
                                '"R":0.0}'
                            )


                            return_phase = None


                            print(
                                "RETURN COMPLETE"
                            )


                            print(
                                "FINAL POSE:",
                                f"x={pose_x:.3f}",
                                f"y={pose_y:.3f}",
                                f"theta="
                                f"{math.degrees(pose_theta):.1f}"
                            )


                            send_hc12(
                                "RETURNED"
                            )


                except Exception as e:

                    print(
                        "T2100 ERROR:",
                        e
                    )


            # =====================
            # T2102
            # TARGET ARRIVED
            # =====================

            elif (
                '"T":2102'
                in line
            ):

                # RETURN 중에는
                # 일반 목표거리 도착 처리 안 함
                if (
                    return_phase
                    is not None
                ):

                    continue


                running = False

                stop_pending = False

                paused_mission = False


                send_ugv(
                    '{"T":1,'
                    '"L":0.0,'
                    '"R":0.0}'
                )


                try:

                    data = json.loads(
                        line
                    )


                    arrived_distance = float(
                        data["distance"]
                    )


                    # 마지막 구간 거리 누적
                    outbound_distance_m += (
                        arrived_distance
                    )


                    send_hc12(
                        f"DISTANCE:"
                        f"{arrived_distance:.2f}"
                    )


                    print(
                        "FINAL DISTANCE:",
                        f"{arrived_distance:.2f}",
                        "m"
                    )


                    print(
                        "OUTBOUND TOTAL:",
                        f"{outbound_distance_m:.2f}",
                        "m"
                    )


                    print(
                        "POSE:",
                        f"x={pose_x:.3f}",
                        f"y={pose_y:.3f}",
                        f"theta="
                        f"{math.degrees(pose_theta):.1f}"
                    )


                except Exception as e:

                    print(
                        "ARRIVAL DISTANCE ERROR:",
                        e
                    )


                send_hc12(
                    "ARRIVED"
                )


                print(
                    "ARRIVED - UGV STOP"
                )


        time.sleep(
            0.2
        )


# =========================
# CTRL + C
# =========================

except KeyboardInterrupt:

    print(
        "\nSTOPPING..."
    )


    stop_flag = True

    running = False

    stop_pending = False

    return_phase = None


    try:

        send_ugv(
            '{"T":1,'
            '"L":0.0,'
            '"R":0.0}'
        )

    except:
        pass


    time.sleep(
        0.2
    )


    hc12.close()

    ugv.close()


    print(
        "STOPPED"
    )