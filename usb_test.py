import serial
import time

ugv = serial.Serial(
    '/dev/ttyACM0',
    115200,
    timeout=0.1
)

print("UGV CONNECTED")

time.sleep(2)

# 입력 버퍼 비우기
ugv.reset_input_buffer()

# 5초 동안 UGV가 보내는 데이터만 확인
end = time.time() + 5

while time.time() < end:
    if ugv.in_waiting:
        data = ugv.read(ugv.in_waiting)
        print("RX RAW:", repr(data))

    time.sleep(0.01)

ugv.close()
print("END")