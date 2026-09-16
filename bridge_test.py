import serial
import threading
import time

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

print("BRIDGE TEST READY")
print("HC12 :", HC12_PORT, HC12_BAUD)
print("UGV  :", UGV_PORT, UGV_BAUD)


# =========================
# GLOBAL STATE
# =========================

running = False
stop_flag = False


# =========================
# HC-12 RECEIVE THREAD
# =========================

def hc12_receive():
    global running
    global stop_flag

    buffer = ""

    while not stop_flag:

        if hc12.in_waiting:

            data = hc12.read(hc12.in_waiting)

            text = data.decode(
                'utf-8',
                errors='ignore'
            )

            for c in text:

                # 한 줄 명령 완성
                if c == '\n':

                    cmd = buffer.strip()
                    buffer = ""

                    if not cmd:
                        continue

                    print("\nHC12 RX:", cmd)

                    # -------------------------
                    # START
                    # -------------------------
                    if cmd == "START":

                        running = True

                        print("UGV START")


                    # -------------------------
                    # STOP
                    # -------------------------
                    elif cmd == "STOP":

                        running = False

                        ugv.write(
                            b'{"T":1,"L":0.0,"R":0.0}\n'
                        )

                        ugv.flush()

                        print("UGV STOP")


                    # -------------------------
                    # EMERGENCY STOP
                    # -------------------------
                    elif cmd == "EMERGENCY_STOP":

                        running = False

                        ugv.write(
                            b'{"T":0}\n'
                        )

                        ugv.flush()

                        print("UGV EMERGENCY STOP")


                    # -------------------------
                    # TARGET DISTANCE
                    # ex) TARGET:0.30
                    # -------------------------
                    elif cmd.startswith("TARGET:"):

                        try:

                            distance = float(
                                cmd.split(":", 1)[1]
                            )

                            if distance <= 0:
                                print("INVALID TARGET")
                                continue

                            message = (
                                '{"T":2101,"d":'
                                + str(distance)
                                + '}\n'
                            )

                            ugv.write(
                                message.encode('utf-8')
                            )

                            ugv.flush()

                            print(
                                "TARGET SENT:",
                                distance,
                                "m"
                            )

                        except ValueError:

                            print(
                                "INVALID TARGET:",
                                cmd
                            )


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

        # ---------------------------------
        # UGV DRIVE
        # ---------------------------------
        # heartbeat 때문에 START 상태에서는
        # 계속 속도 명령 전송
        if running:

            ugv.write(
                b'{"T":1,"L":0.5,"R":0.5}\n'
            )

            ugv.flush()


        # ---------------------------------
        # UGV RECEIVE
        # ---------------------------------
        while ugv.in_waiting:

            line = ugv.readline().decode(
                'utf-8',
                errors='ignore'
            ).strip()

            if not line:
                continue

            print("UGV RX:", line)


            # -----------------------------
            # ARRIVED
            # T2102 = target distance reached
            # -----------------------------
            if '"T":2102' in line:

                running = False

                print("ARRIVED")

                # 외부 ESP32에도 도착 전달
                hc12.write(
                    b'ARRIVED\n'
                )

                hc12.flush()


        time.sleep(0.2)


# =========================
# CTRL+C
# =========================

except KeyboardInterrupt:

    print("\nSTOPPING...")

    stop_flag = True
    running = False

    # 안전하게 정지
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