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

# STOP 직후 최종 엔코더값 기다리는 상태
stop_pending = False

start_odl = None
start_odr = None

target_distance = None


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

                        stop_pending = False
                        running = True

                        print(
                            "UGV START"
                        )

                        hc12.write(
                            b'DRIVING\n'
                        )

                        hc12.flush()


                    # =====================
                    # STOP
                    # =====================

                    elif cmd == "STOP":

                        # 모터는 즉시 정지
                        running = False

                        ugv.write(
                            b'{"T":1,"L":0.0,"R":0.0}\n'
                        )

                        ugv.flush()

                        print(
                            "UGV STOP"
                        )

                        # 바로 STOPPED를 보내지 않음.
                        # 다음 T2100을 받아 실제 최종거리를
                        # 앱에 먼저 전달한 뒤 STOPPED 전송.
                        stop_pending = True

                        print(
                            "WAITING FOR FINAL STOP DISTANCE..."
                        )


                    # =====================
                    # EMERGENCY STOP
                    # =====================

                    elif cmd == "EMERGENCY_STOP":

                        running = False
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
                    # ex) TARGET:0.50
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


                            target_distance = (
                                distance
                            )

                            # 새로운 구간이므로
                            # 엔코더 기준점 초기화
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


                            # ESP32/앱에도 목표거리 확인
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
                    # RESET DISTANCE
                    # =====================

                    elif cmd == "RESET_DISTANCE":

                        start_odl = None
                        start_odr = None

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

        # =====================
        # DRIVE HEARTBEAT
        # =====================

        if running:

            ugv.write(
                b'{"T":1,"L":0.5,"R":0.5}\n'
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
            # ENCODER DISTANCE
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


                    # 첫 T2100을 현재 구간의
                    # 기준점으로 저장
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
                    # 정상 주행 중
                    # =====================

                    if running:

                        msg = (
                            f"DISTANCE:"
                            f"{distance_m:.2f}\n"
                        )

                        hc12.write(
                            msg.encode(
                                'utf-8'
                            )
                        )

                        hc12.flush()


                        print(
                            "CURRENT DISTANCE:",
                            f"{distance_m:.2f}",
                            "m"
                        )


                    # =====================
                    # STOP 직후
                    # 최종거리 전달
                    # =====================

                    elif stop_pending:

                        msg = (
                            f"DISTANCE:"
                            f"{distance_m:.2f}\n"
                        )

                        hc12.write(
                            msg.encode(
                                'utf-8'
                            )
                        )

                        hc12.flush()


                        print(
                            "STOP DISTANCE:",
                            f"{distance_m:.2f}",
                            "m"
                        )


                        # 최종거리 전달 후
                        # STOPPED 전달
                        hc12.write(
                            b'STOPPED\n'
                        )

                        hc12.flush()


                        print(
                            "STOPPED SENT"
                        )


                        stop_pending = False


                except Exception as e:

                    print(
                        "DISTANCE ERROR:",
                        e
                    )


            # =====================
            # T2102
            # TARGET ARRIVED
            # =====================

            elif '"T":2102' in line:

                running = False
                stop_pending = False


                # 안전을 위해 정지 명령 한번 더
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


                    # 최종거리 먼저 앱으로 전달
                    hc12.write(
                        (
                            f"DISTANCE:"
                            f"{arrived_distance:.2f}\n"
                        ).encode(
                            'utf-8'
                        )
                    )

                    hc12.flush()


                    print(
                        "FINAL DISTANCE:",
                        f"{arrived_distance:.2f}",
                        "m"
                    )


                except Exception as e:

                    print(
                        "ARRIVAL DISTANCE ERROR:",
                        e
                    )


                # 그 다음 ARRIVED 전달
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
    stop_pending = False


    try:

        ugv.write(
            b'{"T":1,"L":0.0,"R":0.0}\n'
        )

        ugv.flush()

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