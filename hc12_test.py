import serial
import time

PORT = "/dev/ttyTHS1"
BAUD = 115200

hc12 = serial.Serial(
    PORT,
    BAUD,
    timeout=0.1
)

print("JETSON HC12 RECEIVE TEST")
print("PORT:", PORT)
print("BAUD:", BAUD)
print("waiting...\n")

try:
    while True:
        if hc12.in_waiting > 0:
            data = hc12.read(hc12.in_waiting)

            print("RX RAW:", repr(data))

            try:
                print("RX TEXT:", data.decode("utf-8"))
            except:
                pass

        time.sleep(0.01)

except KeyboardInterrupt:
    pass

finally:
    hc12.close()
    print("CLOSED")