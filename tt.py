import serial
import time

ugv = serial.Serial('/dev/ttyACM0', 115200, timeout=0.2)

print("UGV connected")
time.sleep(3)

ugv.write(b'{"T":2101,"d":0.30}\n')
ugv.flush()

print("TARGET SENT")

end = time.time() + 3

while time.time() < end:
    line = ugv.readline().decode('utf-8', errors='ignore').strip()
    if line:
        print("RX:", line)

ugv.close()