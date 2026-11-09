# imu_test.py
import serial
import json
import time

# 포트는 환경에 맞게 확인 (/dev/ttyACM0)
ugv = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)

# 연속 피드백 모드 ON 명령어 전송
ugv.write(b'{"T":131, "cmd":1}\n')
print("연속 피드백 모드를 켰습니다. 데이터를 10초간 읽어옵니다...\n로봇을 손으로 잡고 좌우로 돌려보세요!")

start_time = time.time()
while time.time() - start_time < 10.0:
    line = ugv.readline().decode('utf-8', errors='ignore').strip()
    if line:
        print("수신된 원본 데이터:", line)
        
# 연속 피드백 모드 OFF 명령어 전송 (원상복구)
ugv.write(b'{"T":131, "cmd":0}\n')
print("테스트 종료.")