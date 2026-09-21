import serial
import threading
import time

PORT = '/dev/ttyTHS1'
BAUD = 115200

hc12 = serial.Serial(PORT, BAUD, timeout=0.05)

def receive_loop():
    while True:
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

def send_loop():
    while True:
        text = input("SEND > ")

        if text:
            hc12.write(
                (text + '\n').encode()
            )
            hc12.flush()

            print("TX:", text)

threading.Thread(
    target=receive_loop,
    daemon=True
).start()

send_loop()