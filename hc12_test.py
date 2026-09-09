import serial
import time

hc12 = serial.Serial('/dev/ttyTHS1', 115200, timeout=0.1)

print("HC12 RAW TEST")

while True:
    if hc12.in_waiting:
        data = hc12.read(hc12.in_waiting)
        print("RX RAW:", repr(data))

    time.sleep(0.01)