"""Passive IMU yaw sign check. No nonzero motor commands.
Run alone, with the main driving program stopped.
Default yaw unit matches the uploaded main code (rad); not auto-detected.
"""
import argparse
import json
import math
import time
import serial


def normalize(angle):
    return (angle + 180.0) % 360.0 - 180.0


def send(ser, packet):
    data = (json.dumps(packet, allow_nan=False) + '\n').encode('utf-8')
    if ser.write(data) != len(data):
        raise IOError('일부 명령만 전송됨')


def main():
    parser = argparse.ArgumentParser(description='손으로 차체를 돌려 IMU Yaw 부호 확인 (자동 회전 없음)')
    parser.add_argument('--port', default='/dev/ttyACM0')
    parser.add_argument('--unit', choices=['rad', 'deg'], default='rad')
    parser.add_argument('--seconds', type=float, default=30.0)
    args = parser.parse_args()
    ser = None
    try:
        ser = serial.Serial(args.port, 115200, timeout=0.1, write_timeout=0.2)
        send(ser, {'T': 1, 'L': 0.0, 'R': 0.0})
        send(ser, {'T': 131, 'cmd': 1})
        print(f'[설정] yaw 단위={args.unit} (기존 메인 코드와 동일한 기본값 rad)')
        print('자동 회전 명령은 없습니다. 다른 주행 프로그램은 종료하세요.')
        print('차체 전방을 기준으로, 위에서 볼 때 왼쪽=반시계 / 오른쪽=시계입니다.')
        print('처음에는 가만히 두고, 이후 왼쪽 약 20도 회전 후 정지, 다시 오른쪽으로 돌려보세요.')
        print('delta는 직전 출력 대비 변화, total은 최초 수신각 대비 누적 변화입니다. Ctrl+C 종료.\n', flush=True)
        start = time.monotonic()
        previous = None
        printed_yaw = None
        total = 0.0
        last_print = 0.0
        last_valid = start
        while time.monotonic() - start < args.seconds:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            now = time.monotonic()
            if now - last_valid > 2.0:
                raise RuntimeError('2초 이상 유효한 yaw 없음: 포트, 피드백 형식 또는 연결 확인')
            if not line:
                continue
            try:
                packet = json.loads(line)
                if not isinstance(packet, dict):
                    continue
                raw = packet.get('yaw', packet.get('y'))
                if raw is None:
                    continue
                raw = float(raw)
                if not math.isfinite(raw):
                    continue
            except (ValueError, TypeError, OverflowError):
                continue
            yaw = math.degrees(raw) if args.unit == 'rad' else raw
            last_valid = now
            if previous is not None:
                total += normalize(yaw - previous)
            previous = yaw
            if now - last_print >= 0.25:
                delta = 0.0 if printed_yaw is None else normalize(yaw - printed_yaw)
                trend = '증가(+)' if delta > 0.3 else '감소(-)' if delta < -0.3 else '거의 정지'
                print(f'[YAW] raw={raw:+.5f} yaw={yaw:+.2f}deg '
                      f'delta={delta:+.2f}deg total={total:+.2f}deg {trend}', flush=True)
                printed_yaw = yaw
                last_print = now
    except KeyboardInterrupt:
        print('\n사용자 종료')
    except Exception as exc:
        print(f'\n[오류] {type(exc).__name__}: {exc}', flush=True)
    finally:
        if ser is not None:
            for packet in ({'T': 1, 'L': 0.0, 'R': 0.0}, {'T': 131, 'cmd': 0}):
                try:
                    send(ser, packet)
                except Exception as exc:
                    print(f'[경고] 종료 명령 전송 실패: {exc}')
            ser.close()
        print('종료. 이 시험으로 T:13의 Z 명령 방향까지 판정되지는 않습니다.')


if __name__ == '__main__':
    main()