#!/usr/bin/env python3
"""
rot_test.py - UGV02 제자리 회전 실측 도구 (ugv_control.py 와 같은 폴더에서 실행)

실행 전: main_highway_contour_avoid_final.py 등 UGV 포트(/dev/ttyACM0)를 쓰는 프로그램을 모두 종료.
로봇 주변 1m 정도 비워두기. Ctrl-C 로 언제든 정지.ttt

사용 예
  python3 rot_test.py                          # 1.8 rad/s 로 0.45초 펄스 x5회 (TURN_OUT 과 동일 조건)
  python3 rot_test.py --target 60              # 목표 60도가 되는 TURN_OUT_TIME_S 추천까지 출력
  python3 rot_test.py --dir right              # 우회전 테스트
  python3 rot_test.py --mode spin --spin-seconds 10   # 긴 회전으로 정상상태 각속도 측정
  python3 rot_test.py --manual                 # 피드백 yaw 를 무시하고 각도기로 잰 값을 직접 입력

각도 측정 방법
  1) 피드백 JSON 에 yaw("y") 가 있으면 자동 측정 (시작 시 피드백 키 목록을 출력함)
  2) 없으면 각도기/테이프로 잰 각도를 입력하면 됨gg
"""
import argparse
import json
import math
import statistics
import sys
import time

import ugv_control

CONTROL_PERIOD_S = 0.02   # main 의 제어 스레드와 동일한 50Hz 전송


# ------------------------------------------------------------
# 피드백 수신 (init_ugv 는 timeout=0 이라 readline 이 부분 데이터를 줄 수 있어 직접 버퍼링)
# ------------------------------------------------------------
class FeedbackReader:
    def __init__(self, ser):
        self.ser = ser
        self.buf = b""
        self.keys = set()
        self.last = None

    def poll(self):
        """새로 도착한 T==1001 피드백 dict 리스트를 반환"""
        out = []
        n = min(self.ser.in_waiting, 4096)
        if n:
            self.buf += self.ser.read(n)
        if len(self.buf) > 16384:          # 개행 없이 쌓이는 경우 방어
            self.buf = self.buf[-4096:]
        while b"\n" in self.buf:
            line, self.buf = self.buf.split(b"\n", 1)
            try:
                d = json.loads(line.decode("utf-8", errors="ignore").strip())
            except Exception:
                continue
            if isinstance(d, dict) and d.get("T") == 1001:
                self.keys |= set(d.keys())
                self.last = d
                out.append(d)
        return out


class YawTracker:
    """yaw 샘플을 이어붙여(unwrap) 누적 회전각(deg)을 계산"""
    def __init__(self, unit="deg"):
        self.unit = unit
        self.last = None
        self.total = 0.0
        self.count = 0

    def feed(self, y):
        y = float(y)
        if self.unit == "rad":
            y = math.degrees(y)
        if self.last is not None:
            self.total += (y - self.last + 180.0) % 360.0 - 180.0
        self.last = y
        self.count += 1


def pump(fb, yaw, use_yaw):
    for d in fb.poll():
        if use_yaw and "y" in d:
            try:
                yaw.feed(d["y"])
            except (TypeError, ValueError):
                pass


def hold(ser, fb, yaw, use_yaw, z, duration):
    """각속도 z 명령을 duration초 동안 50Hz 로 보내며 피드백 수신. 실제 경과시간 반환."""
    t0 = time.monotonic()
    while time.monotonic() - t0 < duration:
        ugv_control.send_driving_command(ser, 0.0, z)
        pump(fb, yaw, use_yaw)
        time.sleep(CONTROL_PERIOD_S)
    return time.monotonic() - t0


def rotate_once(ser, fb, yaw, use_yaw, z, duration, settle):
    """z 로 duration초 회전 -> 정지 후 settle초 대기. (실제 명령시간, yaw 변화량 또는 None) 반환."""
    hold(ser, fb, yaw, use_yaw, 0.0, 0.3)          # 시작 yaw 확보
    yaw_before = yaw.total
    count_before = yaw.count
    actual = hold(ser, fb, yaw, use_yaw, z, duration)
    hold(ser, fb, yaw, use_yaw, 0.0, settle)       # 정지 명령 유지 + 관성으로 더 도는 것까지 포함
    if use_yaw and yaw.count - count_before >= 2:
        return actual, abs(yaw.total - yaw_before)
    return actual, None


def ask_float(prompt):
    s = input(prompt).strip()
    if not s:
        return None
    try:
        return float(s)
    except ValueError:
        print("  숫자가 아니라서 건너뜁니다.")
        return None


def geometry_report(theta_deg, pass_speed, diag_times):
    th = math.radians(theta_deg)
    print(f"\n[대각선 이동량] 회전각 {theta_deg:.1f}도, PASS_SPEED={pass_speed} m/s 기준 (명령값 기준 이상치)")
    for t in diag_times:
        d = pass_speed * t
        print(f"  DIAGONAL_TIME_S={t:.1f}s -> 옆 {d * math.sin(th) * 100:.1f}cm / 앞 {d * math.cos(th) * 100:.1f}cm")


# ------------------------------------------------------------
# 모드
# ------------------------------------------------------------
def run_pulse(args, ser, fb, yaw, use_yaw):
    z = args.omega if args.dir == "left" else -args.omega
    nominal = math.degrees(args.omega * args.time)
    print(f"\n[펄스 테스트] 명령 {z:+.2f} rad/s x {args.time:.2f}s x {args.repeat}회 (계산상 이론 각도 {nominal:.1f}도)")
    print("회전 전 로봇 진행방향을 바닥에 테이프로 표시하고, 회전 후 새 진행방향과의 각도를 재세요.")
    angles, times = [], []
    for i in range(args.repeat):
        s = input(f"\n[{i + 1}/{args.repeat}] 로봇을 시작 위치/방향에 두고 엔터 (q = 중단): ").strip().lower()
        if s == "q":
            break
        actual, dyaw = rotate_once(ser, fb, yaw, use_yaw, z, args.time, args.settle)
        times.append(actual)
        print(f"  실제 명령 시간: {actual:.3f}s")
        measured = None
        if dyaw is not None:
            print(f"  피드백 yaw 변화: {dyaw:.1f}도")
            measured = dyaw
        else:
            measured = ask_float("  잰 회전 각도(도, 건너뛰려면 엔터): ")
        if measured is not None:
            angles.append(abs(measured))

    if not angles:
        print("\n측정된 각도가 없습니다.")
        return
    mean = statistics.mean(angles)
    sd = statistics.stdev(angles) if len(angles) > 1 else 0.0
    t_mean = statistics.mean(times)
    print("\n================ 결과 ================")
    print(f"측정 {len(angles)}회: " + ", ".join(f"{a:.1f}" for a in angles))
    print(f"평균 {mean:.1f}도 / 표준편차 {sd:.1f}도 / 범위 {min(angles):.1f}~{max(angles):.1f}도")
    print(f"이론 {nominal:.1f}도 대비 실측 비율: {mean / nominal * 100:.0f}%")
    deg_per_s = mean / t_mean
    print(f"유효 회전율(짧은 펄스 기준): {deg_per_s:.1f} deg/s  (명령 {math.degrees(args.omega):.1f} deg/s)")
    if args.target:
        rec = args.target / deg_per_s
        print(f"\n목표 {args.target:.0f}도 -> TURN_OUT_TIME_S ~= {rec:.2f}s (선형 가정, 바꾼 뒤 다시 측정해서 확인하세요)")
    if sd > 0.15 * mean:
        print("\n주의: 편차가 평균의 15% 이상입니다. 바닥/배터리/시작 자세 영향이 커서 시간 기반 회전은 오차가 큽니다.")
    geometry_report(mean, args.pass_speed, args.diag_times)


def run_spin(args, ser, fb, yaw, use_yaw):
    z = args.omega if args.dir == "left" else -args.omega
    print(f"\n[긴 회전 테스트] 명령 {z:+.2f} rad/s x {args.spin_seconds:.1f}s")
    input("준비되면 엔터 (바닥에 시작 방향 표시, 몇 바퀴 도는지 세세요): ")
    actual, dyaw = rotate_once(ser, fb, yaw, use_yaw, z, args.spin_seconds, args.settle)
    print(f"실제 명령 시간: {actual:.3f}s")
    cmd_deg_s = math.degrees(args.omega)
    if dyaw is not None:
        w = dyaw / actual
        print(f"피드백 yaw 변화 {dyaw:.1f}도 -> 평균 각속도 {w:.1f} deg/s ({math.radians(w):.2f} rad/s), 명령 대비 {w / cmd_deg_s * 100:.0f}%")
    laps = ask_float("몇 바퀴 돌았나요? (소수 가능, 예 3.5 / 건너뛰려면 엔터): ")
    if laps is not None:
        w = laps * 360.0 / actual
        print(f"바퀴 수 기준 평균 각속도 {w:.1f} deg/s ({math.radians(w):.2f} rad/s), 명령 대비 {w / cmd_deg_s * 100:.0f}%")
    print("(긴 회전의 평균값은 정상상태 각속도입니다. 짧은 펄스는 가속 구간 때문에 이보다 작게 나옵니다.)")


# ------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="UGV02 제자리 회전 실측")
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--mode", choices=("pulse", "spin"), default="pulse")
    ap.add_argument("--omega", type=float, default=1.8, help="각속도 크기 rad/s (TURN_ANGULAR_SPEED)")
    ap.add_argument("--time", type=float, default=0.45, help="펄스 시간 s (TURN_OUT_TIME_S)")
    ap.add_argument("--repeat", type=int, default=5)
    ap.add_argument("--settle", type=float, default=1.0, help="정지 후 대기 s")
    ap.add_argument("--dir", choices=("left", "right"), default="left")
    ap.add_argument("--spin-seconds", type=float, default=10.0)
    ap.add_argument("--target", type=float, default=0.0, help="목표 회전각(도). 주면 TURN_OUT_TIME_S 추천 출력")
    ap.add_argument("--pass-speed", type=float, default=0.20)
    ap.add_argument("--diag-times", type=float, nargs="+", default=[1.0, 2.0])
    ap.add_argument("--yaw-unit", choices=("deg", "rad"), default="deg")
    ap.add_argument("--manual", action="store_true", help="피드백 yaw 를 쓰지 않고 각도를 직접 입력")
    args = ap.parse_args()

    if not (0 < args.omega <= 3.0 and 0 < args.time <= 15 and 0 < args.spin_seconds <= 30):
        print("omega/time/spin-seconds 범위를 확인하세요 (omega<=3.0, time<=15, spin<=30).")
        return 1

    ser = ugv_control.init_ugv(port=args.port, baudrate=args.baud)
    if ser is None:
        print("UGV02 연결 실패: 포트를 쓰는 다른 프로그램이 있는지, 포트 이름이 맞는지 확인하세요.")
        return 1

    fb = FeedbackReader(ser)
    yaw = YawTracker(args.yaw_unit)
    try:
        ugv_control.send_driving_command(ser, 0.0, 0.0)
        print("피드백 확인 중 (1.5초, 로봇 정지)...")
        hold(ser, fb, yaw, not args.manual, 0.0, 1.5)
        if fb.keys:
            print(f"피드백 키: {sorted(fb.keys)}")
            print(f"샘플: {json.dumps(fb.last, ensure_ascii=False)}")
        else:
            print("T=1001 피드백이 수신되지 않았습니다 -> 각도는 직접 재서 입력하세요.")
        use_yaw = (not args.manual) and ("y" in fb.keys) and yaw.count > 0
        if use_yaw:
            print(f"yaw('y') 필드로 자동 측정합니다 (단위 {args.yaw_unit} 가정). 첫 몇 번은 직접 잰 각도와 비교해 단위를 확인하세요.")
        else:
            print("yaw 자동 측정을 쓰지 않습니다 -> 각도기/테이프로 잰 값을 입력하세요.")

        if args.mode == "pulse":
            run_pulse(args, ser, fb, yaw, use_yaw)
        else:         print("\n중단됨")
    finally:
        ugv_control.stop_ugv(ser)   # 정지 명령 후 포트 닫기
    return 0


if __name__ == "__main__":
    sys.exit(main())
