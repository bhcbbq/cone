import serial
import time

# 아두이노 연결 포트 및 보레이트 설정
# 포트가 ttyUSB0인 경우 PORT = '/dev/ttyUSB0'로 변경하세요.
PORT = '/dev/ttyUSB0'
BAUDRATE = 9600

def main():
    try:
        # 시리얼 포트 열기
        ser = serial.Serial(PORT, BAUDRATE, timeout=1)
        time.sleep(2)  # 아두이노 시리얼 재부팅 대기
        
        print(f"[{PORT}] 포트에 성공적으로 연결되었습니다.")
        print("데이터 수신을 시작합니다. (종료하려면 Ctrl+C를 누르세요)\n")
        print("-" * 60)

        while True:
            if ser.in_waiting > 0:
                # 시리얼 데이터 읽기 및 문자열 변환
                raw_data = ser.readline()
                line = raw_data.decode('utf-8', errors='replace').rstrip()
                
                if line:
                    print(f"[수신 완료] {line}")

    except serial.SerialException as e:
        print(f"\n[오류] 시리얼 통신 에러: {e}")
        print("팁: 연결 포트명이 맞는지 확인하고 'sudo chmod 666 /dev/ttyACM0' 명령어로 권한을 부여하세요.")
    except KeyboardInterrupt:
        print("\n[종료] 수신을 중단합니다.")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("시리얼 포트가 닫혔습니다.")

if __name__ == '__main__':
    main()