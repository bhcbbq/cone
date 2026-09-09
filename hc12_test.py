import serial
import threading

hc12 = serial.Serial(
    '/dev/ttyTHS1',
    115200,
    timeout=0.1
)

print("JETSON HC12 READY")

def receive():
    while True:
        if hc12.in_waiting:
            data = hc12.read(hc12.in_waiting)

            try:
                text = data.decode('utf-8', errors='ignore')
                if text:
                    print("\nRX:", repr(text))
                    print("SEND >", end=" ", flush=True)
            except Exception as e:
                print("RX ERROR:", e)

threading.Thread(target=receive, daemon=True).start()

while True:
    text = input("SEND > ")
    hc12.write((text + '\n').encode('utf-8'))
    hc12.flush()