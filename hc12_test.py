import serial
import threading
import time

PORT = '/dev/ttyTHS1'
BAUD = 115200

hc12 = serial.Serial(
    PORT,
    BAUD,
    timeout=0.05
)

print("JETSON HC12 READY")
print(f"PORT: {PORT}")
print(f"BAUD: {BAUD}")
print()

def receive_loop():
    while True:
        try:
            if hc12.in_waiting > 0:
                data = hc12.read(hc12.in_waiting)

                if data:
                    text = data.decode(
                        'utf-8',
                        errors='ignore'
                    )

                    print(f"\nRX: {text}", end="", flush=True)
                    print("SEND > ", end="", flush=True)

            time.sleep(0.001)

        except Exception as e:
            print("\nRX ERROR:", e)
            break


def send_loop():
    while True:
        try:
            text = input("SEND > ")

            if text:
                hc12.write(
                    (text + '\n').encode('utf-8')
                )
                hc12.flush()

                print("TX:", text)

        except KeyboardInterrupt:
            break


rx_thread = threading.Thread(
    target=receive_loop,
    daemon=True
)

rx_thread.start()

try:
    send_loop()

finally:
    hc12.close()
    print("\nHC12 CLOSED")