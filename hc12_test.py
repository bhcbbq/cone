import serial

hc12 = serial.Serial('/dev/ttyTHS1', 9600, timeout=2)

print("HC12 TEST")

while True:
    data = hc12.readline()
    if data:
        print("RX:", repr(data))