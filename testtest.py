import serial
import time

PORT = '/dev/ttyUSB0'
BAUDRATE = 115200


def main():
    try:
        with serial.Serial(PORT, BAUDRATE, timeout=1) as ser:
            time.sleep(2)  # 우노 재시작 대기

            print(f"[연결] {PORT} / {BAUDRATE} baud")
            print("데이터 수신을 시작합니다. 종료: Ctrl+C")
            print("-" * 60)

            while True:
                # 데이터가 없으면 최대 1초 기다림
                raw_data = ser.readline()

                if not raw_data:
                    continue

                line = raw_data.decode(
                    'utf-8', errors='replace'
                ).rstrip()

                if line:
                    print(line, flush=True)

    except serial.SerialException as e:
        print(f"\n[오류] 시리얼 통신 실패: {e}")
        print("포트 이름과 다른 프로그램의 포트 사용 여부를 확인하세요.")

    except KeyboardInterrupt:
        print("\n[종료] 수신을 중단했습니다.")


if __name__ == '__main__':
    main()