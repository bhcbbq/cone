import serial
import threading
import time
import json


# =========================
# SERIAL SETTINGS
# =========================

HC12_PORT = '/dev/ttyTHS1'
HC12_BAUD = 115200

UGV_PORT = '/dev/ttyACM0'
UGV_BAUD = 115200


# =========================
# DRIVE / RETURN SETTINGS
# =========================

DRIVE_SPEED = 0.5
TURN_SPEED = 0.35

# 실제 테스트해서 맞춘 값
TURN_180_DISTANCE_M = 0.50


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
returning_turn = False
returning_drive = False

stop_flag = False
stop_pending = False

start_odl = None
start_odr = None

target_distance = None

current_odl = None
current_odr = None


# =========================
# RETURN STATE
# =========================

# 실제 처음 주행한 거리
return_drive_distance = 0.0

# 회전 시작 엔코더
return_turn_start_odl = None
return_turn_start_odr = None

# 복귀 직진 시작 엔코더
return_drive_start_odl = None
return_drive_start_odr = None


# =========================
# HC12 RECEIVE THREAD
# =========================

def hc12_receive():

    global running
    global returning_turn
    global returning_drive

    global stop_flag
    global stop_pending

    global start_odl
    global start_odr

    global target_distance

    global current_odl
    global current_odr

    global return_drive_distance

    global return_turn_start_odl
    global return_turn_start_odr

    global return_drive_start_odl
    global return_drive_start_odr


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

                        returning_turn = False
                        returning_drive = False

                        stop_pending = False

                        running = True

                        print("UGV START")

                        hc12.write(
                            b'DRIVING\n'
                        )

                        hc12.flush()


                    # =====================
                    # STOP
                    # =====================

                    elif cmd == "STOP":

                        running = False
                        returning_turn = False
                        returning_drive = False

                        ugv.write(
                            b'{"T":1,"L":0.0,"R":0.0}\n'
                        )

                        ugv.flush()

                        print("UGV STOP")

                        stop_pending = True

                        print(
                            "WAITING FOR FINAL STOP DISTANCE..."
                        )


                    # =====================
                    # EMERGENCY STOP
                    # =====================

                    elif cmd == "EMERGENCY_STOP":

                        running = False
                        returning_turn = False
                        returning_drive = False
                        stop_pending = False

                        ugv.write(
                            b'{"T":0}\n'
                        )

                        ugv.flush()

                        print(
                            "UGV EMERGENCY STOP"
                        )

                        hc12.write(
                            b'EMERGENCY\n'
                        )

                        hc12.flush()


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


                            target_distance = distance

                            start_odl = None
                            start_odr = None

                            stop_pending = False


                            message = (
                                '{"T":2101,"d":'
                                + str(distance)
                                + '}\n'
                            )


                            ugv.write(
                                message.encode(
                                    'utf-8'
                                )
                            )

                            ugv.flush()


                            print(
                                "TARGET SENT:",
                                distance,
                                "m"
                            )


                            hc12.write(
                                (
                                    f"TARGET:"
                                    f"{distance:.2f}\n"
                                ).encode(
                                    'utf-8'
                                )
                            )

                            hc12.flush()


                        except ValueError:

                            print(
                                "INVALID TARGET:",
                                cmd
                            )


                    # =====================
                    # RETURN
                    # =====================

                    elif cmd == "RETURN":

                        running = False
                        stop_pending = False

                        returning_drive = False


                        # 일단 정지
                        ugv.write(
                            b'{"T":1,"L":0.0,"R":0.0}\n'
                        )

                        ugv.flush()

                        time.sleep(0.2)


                        if (
                            current_odl is None
                            or current_odr is None
                        ):

                            print(
                                "RETURN ERROR:"
                                " NO ENCODER DATA"
                            )

                            hc12.write(
                                b'RETURN_ERROR\n'
                            )

                            hc12.flush()

                            continue


                        if (
                            return_drive_distance
                            <= 0
                        ):

                            print(
                                "RETURN ERROR:"
                                " NO DRIVE DISTANCE"
                            )

                            hc12.write(
                                b'RETURN_ERROR\n'
                            )

                            hc12.flush()

                            continue


                        # 회전 시작점 저장
                        return_turn_start_odl = (
                            current_odl
                        )

                        return_turn_start_odr = (
                            current_odr
                        )

                        returning_turn = True


                        print(
                            "RETURN START"
                        )

                        print(
                            "RETURN DISTANCE:",
                            f"{return_drive_distance:.2f}",
                            "m"
                        )

                        print(
                            "180 TURN START"
                        )


                        hc12.write(
                            b'RETURNING\n'
                        )

                        hc12.flush()


                    # =====================
                    # RESET DISTANCE
                    # =====================

                    elif cmd == "RESET_DISTANCE":

                        start_odl = None
                        start_odr = None

                        return_drive_distance = 0.0

                        print(
                            "DISTANCE RESET"
                        )

                        hc12.write(
                            b'DISTANCE:0.00\n'
                        )

                        hc12.flush()


                    # =====================
                    # UNKNOWN
                    # =====================

                    else:

                        print(
                            "UNKNOWN COMMAND:",
                            cmd
                        )


                elif c != '\r':

                    buffer += c


        time.sleep(0.001)


# =========================
# THREAD START
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


        # =====================
        # NORMAL DRIVE
        # =====================

        if running:

            ugv.write(
                (
                    '{"T":1,'
                    f'"L":{DRIVE_SPEED},'
                    f'"R":{DRIVE_SPEED}'
                    '}\n'
                ).encode()
            )

            ugv.flush()


        # =====================
        # RETURN TURN
        # =====================

        elif returning_turn:

            ugv.write(
                (
                    '{"T":1,'
                    f'"L":{-TURN_SPEED},'
                    f'"R":{TURN_SPEED}'
                    '}\n'
                ).encode()
            )

            ugv.flush()


        # =====================
        # RETURN DRIVE
        # =====================

        elif returning_drive:

            ugv.write(
                (
                    '{"T":1,'
                    f'"L":{DRIVE_SPEED},'
                    f'"R":{DRIVE_SPEED}'
                    '}\n'
                ).encode()
            )

            ugv.flush()


        # =====================
        # UGV RECEIVE
        # =====================

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
                    # NORMAL DRIVE DISTANCE
                    # =====================

                    if (
                        not returning_turn
                        and
                        not returning_drive
                    ):

                        if start_odl is None:
                            start_odl = odl

                        if start_odr is None:
                            start_odr = odr


                        delta_l = (
                            odl - start_odl
                        )

                        delta_r = (
                            odr - start_odr
                        )

                        distance_m = (
                            (
                                delta_l
                                + delta_r
                            )
                            / 2.0
                        ) / 100.0


                        if distance_m < 0:
                            distance_m = 0.0


                        if running:

                            hc12.write(
                                (
                                    f"DISTANCE:"
                                    f"{distance_m:.2f}\n"
                                ).encode()
                            )

                            hc12.flush()

                            print(
                                "CURRENT DISTANCE:",
                                f"{distance_m:.2f}",
                                "m"
                            )


                        elif stop_pending:

                            hc12.write(
                                (
                                    f"DISTANCE:"
                                    f"{distance_m:.2f}\n"
                                ).encode()
                            )

                            hc12.flush()

                            print(
                                "STOP DISTANCE:",
                                f"{distance_m:.2f}",
                                "m"
                            )


                            hc12.write(
                                b'STOPPED\n'
                            )

                            hc12.flush()

                            print(
                                "STOPPED SENT"
                            )

                            stop_pending = False


                    # =====================
                    # RETURN TURN CHECK
                    # =====================

                    if returning_turn:

                        left_m = abs(
                            odl
                            - return_turn_start_odl
                        ) / 100.0

                        right_m = abs(
                            odr
                            - return_turn_start_odr
                        ) / 100.0


                        turn_distance = (
                            left_m
                            + right_m
                        ) / 2.0


                        print(
                            "TURN:",
                            f"{turn_distance:.2f}",
                            "/",
                            f"{TURN_180_DISTANCE_M:.2f}",
                            "m"
                        )


                        if (
                            turn_distance
                            >= TURN_180_DISTANCE_M
                        ):

                            returning_turn = False


                            ugv.write(
                                b'{"T":1,"L":0.0,"R":0.0}\n'
                            )

                            ugv.flush()


                            print(
                                "180 TURN COMPLETE"
                            )


                            time.sleep(
                                0.3
                            )


                            # 복귀 직진 기준 저장
                            return_drive_start_odl = odl
                            return_drive_start_odr = odr

                            returning_drive = True


                            print(
                                "RETURN DRIVE START"
                            )


                    # =====================
                    # RETURN DRIVE CHECK
                    # =====================

                    elif returning_drive:

                        left_m = abs(
                            odl
                            - return_drive_start_odl
                        ) / 100.0

                        right_m = abs(
                            odr
                            - return_drive_start_odr
                        ) / 100.0


                        returned_m = (
                            left_m
                            + right_m
                        ) / 2.0


                        print(
                            "RETURN DISTANCE:",
                            f"{returned_m:.2f}",
                            "/",
                            f"{return_drive_distance:.2f}",
                            "m"
                        )


                        if (
                            returned_m
                            >= return_drive_distance
                        ):

                            returning_drive = False


                            ugv.write(
                                b'{"T":1,"L":0.0,"R":0.0}\n'
                            )

                            ugv.flush()


                            print(
                                "RETURN COMPLETE"
                            )


                            hc12.write(
                                b'RETURNED\n'
                            )

                            hc12.flush()


                            print(
                                "RETURNED SENT"
                            )


                except Exception as e:

                    print(
                        "T2100 ERROR:",
                        e
                    )


            # =====================
            # T2102
            # ARRIVED
            # =====================

            elif '"T":2102' in line:

                running = False
                stop_pending = False


                ugv.write(
                    b'{"T":1,"L":0.0,"R":0.0}\n'
                )

                ugv.flush()


                try:

                    data = json.loads(
                        line
                    )

                    arrived_distance = float(
                        data["distance"]
                    )


                    # 이 거리를 RETURN 때 사용
                    return_drive_distance = (
                        arrived_distance
                    )


                    hc12.write(
                        (
                            f"DISTANCE:"
                            f"{arrived_distance:.2f}\n"
                        ).encode()
                    )

                    hc12.flush()


                    print(
                        "FINAL DISTANCE:",
                        f"{arrived_distance:.2f}",
                        "m"
                    )

                    print(
                        "RETURN DISTANCE SAVED:",
                        f"{return_drive_distance:.2f}",
                        "m"
                    )


                except Exception as e:

                    print(
                        "ARRIVAL DISTANCE ERROR:",
                        e
                    )


                hc12.write(
                    b'ARRIVED\n'
                )

                hc12.flush()


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
    returning_turn = False
    returning_drive = False

    stop_pending = False


    try:

        ugv.write(
            b'{"T":1,"L":0.0,"R":0.0}\n'
        )

        ugv.flush()

    except:
        pass


    time.sleep(0.2)

    hc12.close()
    ugv.close()

    print(
        "STOPPED"
    )