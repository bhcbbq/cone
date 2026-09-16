import serial
import threading
import time

# -------------------------
# HC-12
# -------------------------
hc12 = serial.Serial(
    '/dev/ttyTHS1',
    115200,
    timeout=0.05
)

# -------------------------
# UGV USB
# -------------------------
ugv = serial.Serial(
    '/dev/ttyACM0',
    115200,
    timeout=0.05
)

print("BRIDGE TEST READY")

running = False
stop_flag = False


def hc12_receive():
    global running, stop_flag

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

                    print("HC12 RX:", cmd)

                    if cmd == "START":
                        running = True
                        print("UGV START")

                    elif cmd == "STOP":
                        running = False

                        ugv.write(
                            b'{"T":1,"L":0.0,"R":0.0}\n'
                        )
                        ugv.flush()

                        print("UGV STOP")

                elif c != '\r':
                    buffer += c

        time.sleep(0.001)


thread = threading.Thread(
    target=hc12_receive,
    daemon=True
)

thread.start()


try:

    while True:

        # START 상태면 0.2초마다 계속 전진 명령
        if running:

            ugv.write(
                b'{"T":1,"L":0.5,"R":0.5}\n'
            )

            ugv.flush()

        # UGV 데이터 수신
        while ugv.in_waiting:

            line = ugv.readline().decode(
                'utf-8',
                errors='ignore'
            ).strip()

            if line:
                print("UGV RX:", line)

        time.sleep(0.2)


except KeyboardInterrupt:

    print("\nSTOPPING...")

    stop_flag = True
    running = False

    # 종료 전에 UGV 정지
    ugv.write(
        b'{"T":1,"L":0.0,"R":0.0}\n'
    )

    ugv.flush()

    time.sleep(0.2)

    hc12.close()
    ugv.close()

    print("STOPPED")