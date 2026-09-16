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

            if data:
                text = data.decode('utf-8', errors='ignore')
                print("\nRX:", text, end="")
                print("SEND >", end=" ", flush=True)

threading.Thread(target=receive, daemon=True).start()

while True:
    text = input("SEND > ")
    hc12.write((text + '\n').encode('utf-8'))
    hc12.flush()