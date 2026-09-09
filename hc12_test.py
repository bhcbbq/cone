import serial
import time

ser = serial.Serial('/dev/ttyTHS1', 9600, timeout=1)

time.sleep(1)

ser.reset_input_buffer()

test = b'HELLO12345678\n'

ser.write(test)
ser.flush()

time.sleep(0.2)

data = ser.read(100)

print("SENT:", repr(test))
print("RECV:", repr(data))

ser.close()