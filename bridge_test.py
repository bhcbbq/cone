import serial
import threading
import time

# HC-12
hc12 = serial.Serial(
    '/dev/ttyTHS1',
    115200,
    timeout=0.05
)

# UGV02 USB
ugv = serial.Serial(
    '/dev/ttyACM0',
    115200,
    timeout=0.05
)

print("BRIDGE TEST READY")

running = False


def hc12_receive():
    global running

    buffer = ""

    while True:
        if hc12.in_waiting:
            data = hc12.read(hc12.in_waiting)
            text = data.decode('utf-8', errors='ignore')

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


threading.Thread(
    target=hc12_receive,
    daemon=True
).start()


try:
    while True:

        # START 상태면 heartbeat 때문에 계속 속도 명령 전송
        if running:
            ugv.write(
                b'{"T":1,"L":0.5,"R":0.5}\n'
            )
            ugv.flush()

        # UGV가 보내는 데이터 확인
        if ugv.in_waiting:
            line = ugv.readline().decode(
                'utf-8',
                errors='ignore'
            ).strip()

            if line:
                print("UGV RX:", line)

        time.sleep(0.2)

except KeyboardInterrupt:
    ugv.write(
        b'{"T":1,"L":0.0,"R":0.0}\n'
    )
    ugv.flush()

    hc12.close()
    ugv.close()

    print("\nSTOPPED")