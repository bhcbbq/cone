import serial
import threading

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1, dsrdtr=None)
ser.setRTS(False)
ser.setDTR(False)

def read_serial():
    while True:
        data = ser.readline()
        if data:
            print("RX:", data.decode(errors="replace").strip())

threading.Thread(target=read_serial, daemon=True).start()

while True:
    cmd = input("TX> ")
    ser.write((cmd + "\n").encode())
## 이두원바보
