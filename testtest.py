"""Uno USB 수신 전용 시험. UGV 모터 명령을 보내지 않습니다.

실행: python3 test_arduino_ultrasonic.py --port /dev/ttyACM0
Arduino의 Serial.begin 값이 다르면 --baud 9600 등으로 지정합니다.
"""
import argparse
import math
import time

import serial


def display_line(raw):
    text = raw.decode("utf-8", errors="replace").strip()
    if not text:
        return
    parts = text.split(",")
    # US,left,front,right 또는 US,sequence,left,front,right 지원
    if parts[0] == "US" and len(parts) in (4, 5):
        try:
            distances = [float(value) for value in parts[-3:]]
            if not all(math.isfinite(value) for value in distances):
                raise ValueError("non-finite distance")
            values = [f"{value:.1f} cm" if value >= 0 else "측정 실패"
                      for value in distances]
            sequence = f" [순번 {parts[1]}]" if len(parts) == 5 else ""
            print(f"왼쪽: {values[0]} | 정면: {values[1]} | 오른쪽: {values[2]}{sequence}",
                  flush=True)
            return
        except ValueError:
            pass
    # 기존 Arduino 스케치의 출력 형식을 몰라도 원문 확인 가능
    print(f"수신 원문: {text}", flush=True)


def main():
    parser = argparse.ArgumentParser(description="Arduino 초음파 거리 수신 전용 시험")
    parser.add_argument("--port", required=True, help="Arduino의 USB 시리얼 장치 경로")
    parser.add_argument("--baud", type=int, default=115200,
                        help="Arduino Serial.begin과 같은 속도 (기본 115200)")
    args = parser.parse_args()
    try:
        with serial.Serial(args.port, args.baud, timeout=0.2) as ser:
            print(f"포트 열림: {ser.port} / {ser.baudrate} baud")
            print("우노는 포트를 열 때 재시작될 수 있습니다. 데이터를 기다립니다.")
            print("종료: Ctrl+C (이 프로그램은 모터를 제어하지 않습니다.)")
            buffer = bytearray()
            last_line_at = time.monotonic()
            while True:
                chunk = ser.read(min(max(ser.in_waiting, 1), 256))
                buffer.extend(chunk)
                while b"\n" in buffer:
                    line, _, rest = buffer.partition(b"\n")
                    buffer = bytearray(rest)
                    display_line(line)
                    last_line_at = time.monotonic()
                if len(buffer) > 4096:
                    print("[경고] 줄바꿈 없는 데이터가 너무 깁니다. 송신 형식/속도를 확인하세요.")
                    buffer.clear()
                if time.monotonic() - last_line_at >= 3.0:
                    print("[대기] 완성된 수신 줄이 없습니다. 포트, baud, Serial.println()을 확인하세요.")
                    last_line_at = time.monotonic()
    except KeyboardInterrupt:
        print("\n수신 시험을 종료했습니다.")
    except (serial.SerialException, OSError, ValueError) as exc:
        print(f"[오류] USB 시리얼 연결/수신 실패: {exc}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())