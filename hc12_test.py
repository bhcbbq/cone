import serial

hc12 = serial.Serial('/dev/ttyTHS1', 115200, timeout=1)

print("HC12 READY")

while True:
    line = hc12.readline().decode('utf-8', errors='ignore').strip()
    if line:
        print("RX:", line)