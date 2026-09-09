import serial
import threading

hc12 = serial.Serial('/dev/ttyTHS1', 115200, timeout=0.1)

def recv():
    while True:
        line = hc12.readline().decode('utf-8', errors='ignore').strip()
        if line:
            print("\nRX:", line)
            print("SEND >", end=" ", flush=True)

threading.Thread(target=recv, daemon=True).start()

while True:
    text = input("SEND > ")
    hc12.write((text + '\n').encode())
    hc12.flush()