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

start_odl = None
start_odr = None

target_distance = None


# =========================
# HC12 RECEIVE
# =========================

def hc12_receive():
    global running
    global stop_flag
    global start_odl
    global start_odr
    global target_distance

    buffer = ""

    while not stop_flag:

        if hc12.in_waiting:

            data = hc12.read(hc12.in_waiting)

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

                    print("\nHC12 RX:", cmd)


                    # =====================
                    # START
                    # =====================

                    if cmd == "START":

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

                        ugv.write(
                            b'{"T":1,"L":0.0,"R":0.0}\n'
                        )

                        ugv.flush()

                        print("UGV STOP")

                        hc12.write(
                            b'STOPPED\n'
                        )

                        hc12.flush()


                    # =====================
                    # EMERGENCY STOP
                    # =====================

                    elif cmd == "EMERGENCY_STOP":

                        running = False

                        ugv.write(
                            b'{"T":0}\n'
                        )

                        ugv.flush()

                        print("UGV EMERGENCY STOP")

                        hc12.write(
                            b'EMERGENCY\n'
                        )

                        hc12.flush()


                    # =====================
                    # TARGET
                    # ex)
                    # TARGET:0.20
                    # =====================

                    elif cmd.startswith("TARGET:"):

                        try:

                            distance = float(
                                cmd.split(":", 1)[1]
                            )

                            if distance <= 0:

                                print("INVALID TARGET")

                                continue

                            target_distance = distance

                            message = (
                                '{"T":2101,"d":'
                                + str(distance)
                                + '}\n'
                            )

                            ugv.write(
                                message.encode('utf-8')
                            )

                            ugv.flush()

                            # 다음 T2100 값을
                            # 이번 주행 기준점으로 사용
                            start_odl = None
                            start_odr = None

                            print(
                                "TARGET SENT:",
                                distance,
                                "m"
                            )

                            hc12.write(
                                f"TARGET:{distance:.2f}\n".encode()
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

                        print("DISTANCE RESET")

                        hc12.write(
                            b'DISTANCE:0.00\n'
                        )

                        hc12.flush()


                    else:

                        print(
                            "UNKNOWN COMMAND:",
                            cmd
                        )


                elif c != '\r':

                    buffer += c


        time.sleep(0.001)


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
        # DRIVE
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

            print("UGV RX:", line)


            # =====================
            # T2100
            # ENCODER DISTANCE
            # =====================

            if line.startswith('{"T":2100'):

                try:

                    data = json.loads(line)

                    odl = float(
                        data["odl"]
                    )

                    odr = float(
                        data["odr"]
                    )

                    # T2100은 cm 단위
                    # 첫 값을 이번 주행 기준점으로 저장
                    if start_odl is None:
                        start_odl = odl

                    if start_odr is None:
                        start_odr = odr

                    delta_l = odl - start_odl
                    delta_r = odr - start_odr

                    distance_cm = (
                        delta_l + delta_r
                    ) / 2.0

                    distance_m = (
                        distance_cm / 100.0
                    )

                    if distance_m < 0:
                        distance_m = 0.0

                    msg = (
                        f"DISTANCE:{distance_m:.2f}\n"
                    )

                    hc12.write(
                        msg.encode('utf-8')
                    )

                    hc12.flush()

                    print(
                        "CURRENT DISTANCE:",
                        f"{distance_m:.2f}",
                        "m"
                    )

                except Exception as e:

                    print(
                        "DISTANCE ERROR:",
                        e
                    )


            # =====================
            # T2102
            # ARRIVED
            # =====================

            elif '"T":2102' in line:

                running = False

                # 도착 후 확실하게 STOP
                ugv.write(
                    b'{"T":1,"L":0.0,"R":0.0}\n'
                )

                ugv.flush()

                print(
                    "ARRIVED - UGV STOP"
                )

                hc12.write(
                    b'ARRIVED\n'
                )

                hc12.flush()


        time.sleep(0.2)


# =========================
# CTRL + C
# =========================

except KeyboardInterrupt:

    print("\nSTOPPING...")

    stop_flag = True
    running = False

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

    print("STOPPED")