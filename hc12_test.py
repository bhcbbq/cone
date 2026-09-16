import serial
import threading
import time

PORT = '/dev/ttyTHS1'
BAUD = 115200

hc12 = serial.Serial(
    PORT,
    BAUD,
    timeout=0.1
)

print("JETSON HC12 READY")
print(f"PORT: {PORT}")
print(f"BAUD: {BAUD}")
print()

last_send_time = None


def receive():
    global last_send_time

    while True:
        if hc12.in_waiting:
            data = hc12.read(hc12.in_waiting)

            if data:
                recv_time = time.time()

                text = data.decode(
                    'utf-8',
                    errors='ignore'
                ).strip()

                if text:
                    print()
                    print("RX:", text)
                    print("RX TIME:", f"{recv_time:.6f}")

                    if last_send_time is not None:
                        delay = recv_time - last_send_time
                        print("DELAY:", f"{delay * 1000:.1f} ms")

                    print("SEND >", end=" ", flush=True)

        time.sleep(0.001)


threading.Thread(
    target=receive,
    daemon=True
).start()


while True:
    try:
        text = input("SEND > ")

        if not text:
            continue

        last_send_time = time.time()

        hc12.write(
            (text + '\n').encode('utf-8')
        )

        hc12.flush()

        print("TX TIME:", f"{last_send_time:.6f}")

    except KeyboardInterrupt:
        print("\nSTOP")
        break


hc12.close()