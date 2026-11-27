"""Standalone UGV T:13 Z sign test. Requires pyserial.
Stop other UGV programs before running. X is always zero.
Default: Z=+0.20 / -0.20, 1 second each, separately armed by Enter.
No camera, lane tracking, or automatic obstacle avoidance.
Software stop cannot guarantee safety if communication fails.
"""
import argparse
import json
import math
import threading
import time

import serial


STALE_S = 0.35
MAX_TEST_YAW_DEG = 20.0


def norm(angle):
    return (angle + 180.0) % 360.0 - 180.0


class ImuReader:
    def __init__(self, ser, unit):
        self.ser = ser
        self.unit = unit
        self.lock = threading.Lock()
        self.done = threading.Event()
        self.yaw = None
        self.total = 0.0
        self.stamp = None
        self.error = None
        self.thread = threading.Thread(target=self.run, daemon=True)

    def snapshot(self):
        with self.lock:
            return self.yaw, self.total, self.stamp, self.error

    def run(self):
        buffer = b''
        try:
            while not self.done.is_set():
                chunk = self.ser.read(min(max(self.ser.in_waiting, 1), 4096))
                if not chunk:
                    continue
                buffer += chunk
                lines = buffer.split(b'\n')
                buffer = lines[-1][-8192:]
                # Use the newest valid yaw in each read; never compete with another reader.
                for line in reversed(lines[:-1]):
                    try:
                        packet = json.loads(line.decode('utf-8', errors='ignore'))
                        if not isinstance(packet, dict):
                            continue
                        value = packet.get('yaw', packet.get('y'))
                        if value is None:
                            continue
                        value = float(value)
                        if not math.isfinite(value):
                            continue
                        yaw = math.degrees(value) if self.unit == 'rad' else value
                    except (ValueError, TypeError, OverflowError):
                        continue
                    with self.lock:
                        if self.yaw is not None:
                            step = norm(yaw - self.yaw)
                            if abs(step) > 45.0:
                                self.error = 'IMU angle jump >45deg; test stopped'
                                return
                            self.total += step
                        self.yaw = yaw
                        self.stamp = time.monotonic()
                    break
        except Exception as exc:
            with self.lock:
                self.error = f'{type(exc).__name__}: {exc}'


def send(ser, packet):
    data = (json.dumps(packet, allow_nan=False) + '\n').encode('utf-8')
    if ser.write(data) != len(data):
        raise IOError('일부 명령만 전송됨')


def stop(ser):
    # Attempt both command formats independently, even if the first fails.
    ok = True
    for packet in ({'T': 13, 'X': 0.0, 'Z': 0.0},
                   {'T': 1, 'L': 0.0, 'R': 0.0}):
        try:
            send(ser, packet)
        except Exception as exc:
            ok = False
            print(f'[정지 전송 실패] {exc}', flush=True)
    return ok


def fresh(imu):
    yaw, total, stamp, error = imu.snapshot()
    if error:
        raise RuntimeError(error)
    if stamp is None or time.monotonic() - stamp > STALE_S:
        raise RuntimeError('IMU yaw 수신 없음 또는 0.35초 이상 갱신 지연')
    return yaw, total


def settle(ser, imu, seconds, base=None):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if not stop(ser):
            raise IOError('정지 명령 전송 실패: 전원을 직접 차단하세요')
        _, total = fresh(imu)
        if base is not None and abs(total - base) > MAX_TEST_YAW_DEG:
            raise RuntimeError('시험 누적 회전량 20도 초과: 시험 중단')
        time.sleep(0.05)


def trial(ser, imu, z, duration):
    input(f'\n[대기] Z={z:+.2f}, X=0, {duration:.2f}초 시험. '
          '안전 확보 후 Enter (취소 Ctrl+C): ')
    settle(ser, imu, 0.5)
    yaw0, total0 = fresh(imu)
    print(f'[START] yaw={yaw0:+.2f}deg / T=13 X=0 Z={z:+.2f}', flush=True)
    start = time.monotonic()
    next_log = 0.0
    try:
        while True:
            now = time.monotonic()
            elapsed = now - start
            if elapsed >= duration:
                break
            yaw, total = fresh(imu)
            delta = total - total0
            if abs(delta) >= MAX_TEST_YAW_DEG:
                raise RuntimeError('시험 누적 회전량 20도 도달: 시험 중단')
            send(ser, {'T': 13, 'X': 0.0, 'Z': z})
            if elapsed >= next_log:
                print(f'[RUN] t={elapsed:.2f}s Z={z:+.2f} '
                      f'yaw={yaw:+.2f}deg delta={delta:+.2f}deg', flush=True)
                next_log = elapsed + 0.2
            time.sleep(0.05)
    finally:
        stop(ser)
    # Include residual motion in the measured result while keeping a stop command active.
    settle(ser, imu, 0.8, base=total0)
    yaw1, total1 = fresh(imu)
    delta = total1 - total0
    if abs(delta) < 1.0:
        meaning = '변화 작음: 방향 판정 보류 (명령 무응답/작은 회전/마찰 등 확인)'
    elif delta > 0:
        meaning = 'Yaw 증가 -> 앞선 수동 시험 기준 왼쪽 회전'
    else:
        meaning = 'Yaw 감소 -> 앞선 수동 시험 기준 오른쪽 회전'
    print(f'[RESULT] Z={z:+.2f} yaw_before={yaw0:+.2f} '
          f'yaw_after={yaw1:+.2f} delta={delta:+.2f}deg | {meaning}', flush=True)
    print('실제 눈으로 본 방향도 함께 기록하세요. 바퀴를 띄우면 차체 Yaw 판정은 불가합니다.', flush=True)
    return z, delta


def main():
    parser = argparse.ArgumentParser(description='T:13 양/음 Z 제자리 회전 방향 시험')
    parser.add_argument('--port', default='/dev/ttyACM0')
    parser.add_argument('--unit', choices=['rad', 'deg'], default='rad')
    parser.add_argument('--z', type=float, default=0.20)
    parser.add_argument('--duration', type=float, default=1.0)
    args = parser.parse_args()
    if not math.isfinite(args.z) or not 0 < args.z <= 0.30:
        parser.error('--z는 0 초과 0.30 이하')
    if not math.isfinite(args.duration) or not 0 < args.duration <= 1.5:
        parser.error('--duration은 0 초과 1.5초 이하')
    ser = imu = None
    try:
        print('[주의] 다른 주행/IMU 프로그램을 종료하세요. 장애물 없는 여유 공간에서 시험하세요.')
        print('모든 주행 명령은 X=0입니다. 통신 장애 시 소프트웨어 정지는 보장되지 않습니다.')
        print('즉시 전원 차단 가능 상태를 확보하세요. yaw 단위 기본값 rad (자동 판별 아님).', flush=True)
        ser = serial.Serial(args.port, 115200, timeout=0.05, write_timeout=0.1)
        if not stop(ser):
            raise IOError('초기 정지 전송 실패')
        ser.reset_input_buffer()
        imu = ImuReader(ser, args.unit)
        imu.thread.start()
        send(ser, {'T': 131, 'cmd': 1})
        deadline = time.monotonic() + 3.0
        while imu.snapshot()[2] is None:
            if imu.snapshot()[3] or time.monotonic() >= deadline:
                raise RuntimeError('초기 IMU 수신 실패')
            time.sleep(0.02)
        fresh(imu)
        results = []
        for z in (args.z, -args.z):
            results.append(trial(ser, imu, z, args.duration))
        print('\n[SUMMARY] 실제 육안 관찰과 아래 결과를 함께 공유하세요.')
        for z, delta in results:
            print(f'Z={z:+.2f} -> delta_yaw={delta:+.2f}deg')
        print('이 시험은 X=0의 방향 확인만 수행합니다. 전진 중 응답이나 이동 거리를 보장하지 않습니다.')
    except KeyboardInterrupt:
        print('\n[중단] Ctrl+C')
    except Exception as exc:
        print(f'\n[중단] {type(exc).__name__}: {exc}', flush=True)
    finally:
        if ser is not None:
            for _ in range(3):
                stop(ser)
                time.sleep(0.05)
            try:
                send(ser, {'T': 131, 'cmd': 0})
            except Exception as exc:
                print(f'[경고] 피드백 종료 전송 실패: {exc}')
            if imu is not None:
                imu.done.set()
                imu.thread.join(timeout=0.5)
            ser.close()
        print('[종료] 정지 명령 전송 시도 및 포트 닫기 완료', flush=True)


if __name__ == '__main__':
    main()