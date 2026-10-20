"""UGV02 + HC-12 + lane_detection_ver1 통합 실행 파일.

앱/중계기 수정 없이 기존 명령과 T2100/T2101/T2102를 사용합니다.
수동 이동은 한 번의 명령으로 최대 2초. 계속하려면 버튼을 다시 누릅니다.
기존 중계기는 BLE 단절을 알리지 않으므로 즉시 단절 감지는 불가능합니다.
모터 명령은 제어 스레드 한 곳에서 전송합니다.
실행: python3 -u main_integrated_ver2.py
"""
import argparse
import collections
import json
import math
import os
import threading
import time
import traceback

import cv2
import numpy as np
import serial

import lane_detection_ver1 as lane_detection
import ugv_control


BASE_SPEED = 0.15
FRAME_WIDTH = 640
FRAME_TIMEOUT = 0.5
ODOM_TIMEOUT = 1.0
MAX_CAMERA_FAILURES = 3
UGV_PORT = "/dev/ttyACM0"
HC12_PORT = "/dev/ttyTHS1"
HC12_BAUD = 115200

MANUAL_SPEED = 0.30
MANUAL_SPEED_MIN = 0.20
MANUAL_SPEED_MAX = 0.50
MANUAL_INNER_RATIO = 0.50
# 기존 앱은 누를 때 1회만 전송. 수동 이동의 무제한 지속을 막습니다.
# 길게 눌러도 2초 후 정지하므로 손을 떼었다 다시 눌러야 계속 이동합니다.
# 실제 BLE 단절 시점을 감지하는 기능은 아닙니다.
MANUAL_COMMAND_TIMEOUT = 2.0
APP_STATUS_INTERVAL = 1.0
BLE_MESSAGE_MAX_BYTES = 20

TURN_SPEED = 0.25
ALIGN_TURN_SPEED = 0.18
COARSE_TURN_DISTANCE_M = 0.44
RETURN_TURN_TIMEOUT = 15.0
RETURN_ALIGN_TIMEOUT = 20.0
RETURN_PREPARE_SECONDS = 0.20
ALIGN_ERROR_THRESHOLD = 15.0
ALIGN_STABLE_FRAMES = 8

# 아래 단위는 기존 팀원 코드와 동일: odl/odr=cm, T2101/T2102 거리=m.
# 하위 ESP32 펌웨어가 다른 단위를 사용하면 반드시 변경할 것.
ODOM_UNITS_PER_METER = 100.0
STOP_SETTLE_SECONDS = 0.25
BASELINE_SETTLE_SECONDS = 0.10
STOP_FEEDBACK_TIMEOUT = 3.0
DISTANCE_REPORT_INTERVAL = 0.20

# 외부 검출 모듈을 수정하지 않고 주행 허용 조건을 추가.
LANE_CONFIRM_FRAMES = 3
SINGLE_SIDE_MAX_SECONDS = 0.60
# 화면의 목표 지점 부근에서 측정한 실제 좌우 차선 간격에 맞춰 조정.
MIN_LANE_WIDTH_RATIO = 0.15

SHOW_WINDOWS = bool(os.environ.get("DISPLAY")) and not bool(
    os.environ.get("SSH_CONNECTION")
    or os.environ.get("SSH_CLIENT")
    or os.environ.get("SSH_TTY")
)
AUTO_MODES = ("DRIVE", "RETURN_TURN", "RETURN_ALIGN", "RETURN_DRIVE")
RETURN_MODES = ("RETURN_TURN", "RETURN_ALIGN", "RETURN_DRIVE")


class LineBuffer:
    """분할된 USB/UART 데이터를 줄바꿈까지 보관. 메모리 사용은 제한."""
    def __init__(self, maximum=8192):
        self.data = bytearray()
        self.maximum = maximum
        self.discarding = False

    def feed(self, chunk):
        lines = []
        for value in chunk:
            if value == 10:
                if not self.discarding:
                    line = self.data.decode("utf-8", errors="replace").strip()
                    if line:
                        lines.append(line)
                self.data.clear()
                self.discarding = False
            elif not self.discarding:
                self.data.append(value)
                if len(self.data) > self.maximum:
                    self.data.clear()
                    self.discarding = True
                    print("[경고] 너무 긴 수신 줄을 버립니다.", flush=True)
        return lines


class SmartConeController:
    def __init__(self, ugv_serial, hc12_serial, clock=None):
        self.ugv_serial = ugv_serial
        self.hc12_serial = hc12_serial
        self.clock = clock or time.monotonic
        self.state_lock = threading.RLock()
        self.tx_lock = threading.Lock()
        self.running = threading.Event()
        self.running.set()
        self.control_wake = threading.Event()
        self.tx_wake = threading.Event()
        self.threads = []
        self.closing = False
        self.events = collections.deque(maxlen=128)
        self.distance_message = None
        self.last_app_report = -math.inf
        self.app_status = None
        # STOP 직후 앱이 보낸 TARGET/START/RETURN을 정지 확인 후 처리.
        self.deferred_target = None
        self.deferred_action = None
        self.mission_goal_distance = None
        # 앱이 재출발 전에 고정한 offset과 실제 정지 거리의 차이.
        self.display_distance_bias = 0.0

        self.mode = "IDLE"
        self.fault = None
        self.emergency_latched = False
        self.emergency_sent = False
        self.manual_speed = MANUAL_SPEED
        self.manual_left_speed = 0.0
        self.manual_right_speed = 0.0
        self.manual_received_at = None

        self.current_odl = None
        self.current_odr = None
        self.odom_received_at = None
        self.start_odl = None
        self.start_odr = None
        self.stop_pending = False
        self.stop_requested_at = None
        self.stop_sent_at = None
        self.paused_mission = False
        self.mission_outbound_total = 0.0
        self.return_drive_distance = 0.0
        self.target_distance = None
        self.target_armed = False
        self.pending_target = None
        self.target_ready_at = None
        self.outbound_baseline_after = None
        self.last_distance_report = -math.inf

        self.return_turn_start_odl = None
        self.return_turn_start_odr = None
        self.return_drive_start_odl = None
        self.return_drive_start_odr = None
        self.return_prepare_until = None
        self.stage_started_at = None
        self.paused_return_mode = None
        self.align_stable_count = 0
        self.align_last_frame = None

        self.vision_epoch = 0
        self.frame_id = 0
        self.frame_time = None
        self.last_camera_frame_time = None
        self.shared_error = 0.0
        self.lane_detected = False
        self.lane_status = "LOST"
        self.lane_reason = "영상 대기"
        self.lane_inferred = None
        self.both_count = 0
        self.last_both_time = None
        self.vision_confirmed = False
        self.previous_error = None
        self.previous_frame_time = None
        self.last_angular = 0.0
        self.last_pd_mode = None
        self.lane_lost_printed = False
        self.last_control_report = -math.inf

    def send_hc12(self, message):
        # HC-12 송신이 제어/엔코더 스레드를 붙잡지 않도록 별도 큐 사용.
        with self.tx_lock:
            if message.startswith("DISTANCE:"):
                self.distance_message = message
            else:
                if len(self.events) == self.events.maxlen:
                    print("[경고] HC12 상태 송신 큐가 가득 찼습니다.", flush=True)
                self.events.append(message)
        self.tx_wake.set()

    @staticmethod
    def app_wire_message(message):
        """중계기의 단일 BLE notify에서 메시지가 잘리지 않도록 제한."""
        aliases = {
            "ERROR:NO_FRESH_ODOM": "ERROR:NO_ODOM",
            "ERROR:INVALID_TARGET": "ERROR:NO_TARGET",
            "RETURN_ERROR:NO_DISTANCE": "RETURN_ERROR:NO_DIST",
            "RETURN_ERROR:NO_PAUSED_RETURN": "RETURN_ERROR:PAUSED",
            "ERROR:FAULT_RESTART_REQUIRED": "ERROR:RESTART",
            "ERROR:UNKNOWN_COMMAND": "ERROR:UNKNOWN_CMD",
        }
        message = aliases.get(message, message)
        if len(message.encode("ascii")) <= BLE_MESSAGE_MAX_BYTES:
            return message
        if message.startswith("MANUAL:"):
            return "MANUAL:OK"
        if message.startswith("RETURN_ERROR"):
            return "RETURN_ERROR"
        if message.startswith("ERROR:"):
            return "ERROR:FAULT"
        # 숫자 메시지가 비정상적으로 길어져도 잘린 숫자를 전송하지 않음.
        if message.startswith(("DISTANCE:", "TARGET:")):
            return "ERROR:VALUE_RANGE"
        return "ERROR:MSG_LENGTH"

    def app_status_messages(self):
        """앱이 이미 처리하는 짧은 메시지로 현재 상태를 반복 안내."""
        with self.state_lock:
            result = ["HC12_CONNECTED"]
            if self.fault:
                status = "STOPPED"
            elif self.mode in RETURN_MODES:
                status = "RETURNING"
            elif self.mode == "DRIVE":
                status = "DRIVING"
            elif self.paused_return_mode is not None:
                status = "RETURN_PAUSED"
            elif self.stop_pending:
                status = None  # 최종 거리 확정 전 STOPPED를 보내지 않음.
            else:
                status = self.app_status
            if status:
                result.append(status)
            return result

    def _clear_deferred(self):
        self.deferred_target = None
        self.deferred_action = None

    def _display_segment_distance(self):
        return max(0.0, self._segment_distance() + self.display_distance_bias)

    def _accept_target(self, requested_distance):
        """실제 남은 거리와 앱의 이전 구간 offset을 별도로 관리."""
        distance = requested_distance
        self.display_distance_bias = 0.0
        if self.paused_mission and self.mission_goal_distance is not None:
            # 앱은 '전체 목표 - 화면 이동 거리'를 TARGET으로 전송한다.
            app_offset = self.mission_goal_distance - requested_distance
            self.display_distance_bias = self.mission_outbound_total - app_offset
            distance = max(0.0, self.mission_goal_distance - self.mission_outbound_total)
        else:
            self.mission_outbound_total = self.return_drive_distance = 0.0
            self.mission_goal_distance = requested_distance
        if distance <= 0:
            self.pending_target = None
            self.target_distance = None
            self.target_armed = False
            self.app_status = "ARRIVED"
            self.paused_mission = False
            self._send_final_distance(requested_distance, "ARRIVED")
            return
        self.target_distance = distance
        self.pending_target = distance
        self.target_armed = False
        self.start_odl = self.start_odr = None
        self.app_status = None
        self.control_wake.set()

    def _reset_mission(self):
        # RESET도 이동 정지를 포함. 복귀 중 앱만 초기화되는 상황 방지.
        self._set_mode("IDLE")
        self.manual_left_speed = self.manual_right_speed = 0.0
        self.stop_pending = False
        self.stop_requested_at = self.stop_sent_at = None
        self._clear_deferred()
        self.start_odl = self.start_odr = None
        self.mission_outbound_total = self.return_drive_distance = 0.0
        self.mission_goal_distance = None
        self.display_distance_bias = 0.0
        self.target_distance = self.pending_target = None
        self.target_armed = False
        self.paused_mission = False
        self.paused_return_mode = None
        self.return_turn_start_odl = self.return_turn_start_odr = None
        self.return_drive_start_odl = self.return_drive_start_odr = None
        self.app_status = None
        self._reset_vision()
        with self.tx_lock:
            # 초기화 전에 큐에 남은 상태/거리로 앱이 다시 바뀌지 않게 함.
            self.events.clear()
            self.distance_message = None
        self.send_hc12("DISTANCE:0.00")
        # 기존 앱에는 RESET 확인 응답이 없으므로 기존 '목표 없음' 응답으로
        # idle을 확정. 초기화 뒤에는 실제로 목표가 없는 상태입니다.
        self.send_hc12("ERROR:NO_TARGET")

    def _send_final_distance(self, distance, status):
        # 앱이 최종 거리 다음에 STOPPED/ARRIVED를 받도록 순서를 보장.
        with self.tx_lock:
            self.distance_message = None
            self.events.append(f"DISTANCE:{distance:.2f}")
            self.events.append(status)
        self.tx_wake.set()

    def _reset_pd(self):
        self.previous_error = None
        self.previous_frame_time = None
        self.last_angular = 0.0

    def _reset_vision(self):
        # 실제 tracker.reset()은 카메라 스레드에서 수행.
        self.vision_epoch += 1
        self.frame_time = None
        self.lane_detected = False
        self.lane_status = "LOST"
        self.lane_reason = "차선 재확인"
        self.both_count = 0
        self.last_both_time = None
        self.vision_confirmed = False
        self.align_stable_count = 0
        self.align_last_frame = None
        self._reset_pd()

    def _set_mode(self, mode):
        self.mode = mode
        self._reset_pd()
        self.control_wake.set()

    def set_fault(self, reason):
        with self.state_lock:
            if self.fault is not None:
                return
            self.fault = reason
            self.mode = "IDLE"
            self.manual_left_speed = self.manual_right_speed = 0.0
            self.pending_target = None
            self._clear_deferred()
            self.app_status = "STOPPED"
            self._reset_pd()
            print(f"[정지 오류] {reason} / 원인 해결 후 프로그램을 다시 실행하세요.", flush=True)
            self.send_hc12(f"ERROR:{reason}")
            self.send_hc12("STOPPED")
        self.control_wake.set()

    def _odom_fresh(self, now):
        return (self.current_odl is not None and self.current_odr is not None
                and self.odom_received_at is not None
                and 0 <= now - self.odom_received_at <= ODOM_TIMEOUT)

    def _camera_fresh(self, now):
        return (self.last_camera_frame_time is not None
                and 0 <= now - self.last_camera_frame_time <= FRAME_TIMEOUT)

    def _segment_distance(self):
        if self.start_odl is None or self.start_odr is None:
            return 0.0
        delta_l = self.current_odl - self.start_odl
        delta_r = self.current_odr - self.start_odr
        return max(0.0, (delta_l + delta_r) / (2 * ODOM_UNITS_PER_METER))

    def _return_progress(self, left_start, right_start):
        return (abs(self.current_odl - left_start)
                + abs(self.current_odr - right_start)) / (2 * ODOM_UNITS_PER_METER)

    def _motion_allowed(self, now, automatic=False, returning=False):
        if self.fault:
            self.send_hc12("ERROR:FAULT_RESTART_REQUIRED")
            return False
        if self.stop_pending:
            self.send_hc12("ERROR:STOP_PENDING")
            return False
        if automatic and not self._odom_fresh(now):
            self.app_status = "RETURN_ERROR:NO_ODOM" if returning else "ERROR:NO_ODOM"
            self.send_hc12("RETURN_ERROR:NO_ODOM" if returning else "ERROR:NO_ODOM")
            return False
        if automatic and not self._camera_fresh(now):
            self.app_status = "STOPPED"
            self.send_hc12("CAMERA_ERROR")
            self.send_hc12("STOPPED")
            return False
        return True

    def _release_emergency(self):
        # 별도의 새 이동 명령을 받아야 비상정지에서 벗어남.
        self.emergency_latched = False
        self.emergency_sent = False

    def handle_command(self, command):
        cmd = command.strip()
        if not cmd:
            return
        now = self.clock()
        print(f"[HC12 RX] {cmd}", flush=True)
        with self.state_lock:
            if self.closing:
                return
            if cmd == "EMERGENCY_STOP":
                self._set_mode("IDLE")
                self.emergency_latched = True
                self.emergency_sent = False
                self.pending_target = None
                self.stop_pending = False
                self.paused_return_mode = None
                self.manual_left_speed = self.manual_right_speed = 0.0
                # 비상정지 후 기존 임무는 재개하지 않음.
                self.target_armed = False
                self.target_distance = None
                self.paused_mission = False
                self.start_odl = self.start_odr = None
                self.mission_outbound_total = self.return_drive_distance = 0.0
                self.mission_goal_distance = None
                self.display_distance_bias = 0.0
                self._clear_deferred()
                self.app_status = "STOPPED"
                self._reset_vision()
                self.send_hc12("EMERGENCY")
                return

            if cmd == "STOP":
                self._clear_deferred()
                self.pending_target = None
                self.target_armed = False
                with self.tx_lock:
                    self.distance_message = None
                old_mode = self.mode
                if old_mode in RETURN_MODES:
                    self.paused_return_mode = old_mode
                    self.app_status = "RETURN_PAUSED"
                    self.send_hc12("RETURN_PAUSED")
                elif old_mode == "DRIVE":
                    self.stop_pending = True
                    self.stop_requested_at = now
                    self.stop_sent_at = None
                    self.paused_mission = True
                    self.app_status = "STOPPED"
                elif old_mode == "MANUAL":
                    self.send_hc12("MANUAL_STOPPED")
                    self.app_status = None
                elif not self.stop_pending:
                    self.app_status = "RETURN_PAUSED" if self.paused_return_mode else "STOPPED"
                    self.send_hc12(self.app_status)
                self.manual_left_speed = self.manual_right_speed = 0.0
                self.align_stable_count = 0
                self._set_mode("IDLE")
                return

            if cmd.startswith("SPEED:"):
                try:
                    value = float(cmd.split(":", 1)[1])
                except ValueError:
                    value = math.nan
                if not math.isfinite(value) or not MANUAL_SPEED_MIN <= value <= MANUAL_SPEED_MAX:
                    self.send_hc12("ERROR:INVALID_SPEED")
                elif self.mode in AUTO_MODES:
                    self.send_hc12("MANUAL_BLOCKED")
                else:
                    self.manual_speed = value
                    self.send_hc12(f"SPEED:{value:.2f}")
                return

            if cmd.startswith("MANUAL:"):
                direction = cmd.split(":", 1)[1].strip().upper()
                if self.mode in AUTO_MODES or self.paused_return_mode is not None:
                    self.send_hc12("MANUAL_BLOCKED")
                    return
                if direction == "STOP":
                    if self.mode == "MANUAL":
                        self._set_mode("IDLE")
                    self.manual_left_speed = self.manual_right_speed = 0.0
                    self.send_hc12("MANUAL_STOPPED")
                    return
                if not self._motion_allowed(now):
                    return
                speed, inner = self.manual_speed, self.manual_speed * MANUAL_INNER_RATIO
                manual_map = {
                    "FORWARD": (speed, speed), "FORWARD_LEFT": (inner, speed),
                    "FORWARD_RIGHT": (speed, inner), "LEFT": (-speed, speed),
                    "RIGHT": (speed, -speed), "BACKWARD": (-speed, -speed),
                    "BACKWARD_LEFT": (-inner, -speed), "BACKWARD_RIGHT": (-speed, -inner),
                }
                if direction not in manual_map:
                    self.send_hc12("ERROR:INVALID_MANUAL")
                    return
                self._release_emergency()
                self.manual_left_speed, self.manual_right_speed = manual_map[direction]
                self.manual_received_at = now
                self.app_status = None
                self._set_mode("MANUAL")
                self.send_hc12(f"MANUAL:{direction}")
                return

            if cmd.startswith("TARGET:"):
                try:
                    distance = float(cmd.split(":", 1)[1])
                except ValueError:
                    distance = math.nan
                if not math.isfinite(distance) or distance <= 0:
                    self.send_hc12("ERROR:INVALID_TARGET")
                    return
                if self.stop_pending:
                    self.deferred_target = distance
                    print(f"[대기] 정지 거리 확정 후 TARGET:{distance} 처리", flush=True)
                    return
                if self.mode != "IDLE" or self.paused_return_mode is not None:
                    self.send_hc12("ERROR:BUSY")
                    return
                self._accept_target(distance)
                return

            if cmd == "START":
                if self.stop_pending:
                    self.deferred_action = "START"
                    return
                if self.app_status == "ARRIVED" and self.target_distance is None:
                    self.send_hc12("ARRIVED")
                    return
                if self.mode != "IDLE" or self.paused_return_mode is not None:
                    self.send_hc12("ERROR:BUSY")
                    return
                if not self._motion_allowed(now, automatic=True):
                    return
                if self.pending_target is None and not self.target_armed:
                    self.app_status = "ERROR:NO_TARGET"
                    self.send_hc12("ERROR:NO_TARGET")
                    return
                self._release_emergency()
                # TARGET 처리 직후 하위보드가 카운터를 재설정하는 경우도 고려.
                # 새 피드백으로 기준값을 잡기 전에는 제어 스레드가 정지를 유지.
                self.start_odl = self.start_odr = None
                self.outbound_baseline_after = max(now, self.target_ready_at or now)
                self._reset_vision()
                self._set_mode("DRIVE")
                self.app_status = "DRIVING"
                self.send_hc12("DRIVING")
                return

            if cmd == "RETURN":
                if self.stop_pending:
                    self.deferred_action = "RETURN"
                    return
                if self.mode != "IDLE" or self.paused_return_mode is not None:
                    self.send_hc12("ERROR:BUSY")
                    return
                if self.return_drive_distance <= 0:
                    self.send_hc12("RETURN_ERROR:NO_DISTANCE")
                    return
                if not self._motion_allowed(now, automatic=True, returning=True):
                    return
                self._release_emergency()
                self.return_turn_start_odl = self.return_turn_start_odr = None
                self.return_prepare_until = now + RETURN_PREPARE_SECONDS
                self.stage_started_at = now
                self.target_armed = False
                self.pending_target = None
                self._reset_vision()
                self._set_mode("RETURN_TURN")
                self.app_status = "RETURNING"
                self.send_hc12("RETURNING")
                return

            if cmd == "RESUME_RETURN":
                if self.mode != "IDLE" or self.paused_return_mode is None:
                    self.send_hc12("RETURN_ERROR:NO_PAUSED_RETURN")
                    return
                if not self._motion_allowed(now, automatic=True, returning=True):
                    return
                self._release_emergency()
                resume_mode = self.paused_return_mode
                self.paused_return_mode = None
                self.stage_started_at = now
                self._reset_vision()
                self._set_mode(resume_mode)
                self.app_status = "RETURNING"
                self.send_hc12("RETURNING")
                return

            if cmd == "RESET_DISTANCE":
                self._reset_mission()
                print("[RESET] 이동 정지 및 임무/거리 초기화", flush=True)
                return
            self.send_hc12("ERROR:UNKNOWN_COMMAND")

    def handle_feedback(self, line):
        try:
            data = json.loads(line)
        except (ValueError, TypeError):
            print(f"[경고] 잘못된 UGV JSON: {line[:160]}", flush=True)
            return
        if not isinstance(data, dict):
            return
        now = self.clock()
        with self.state_lock:
            if data.get("T") == 2100:
                try:
                    odl, odr = float(data["odl"]), float(data["odr"])
                    if not all(math.isfinite(v) for v in (odl, odr)):
                        raise ValueError("non-finite encoder")
                except (KeyError, TypeError, ValueError):
                    print("[경고] T2100 엔코더 데이터 오류", flush=True)
                    return
                self.current_odl, self.current_odr = odl, odr
                self.odom_received_at = now
                if (self.mode == "DRIVE" and self.start_odl is None
                        and self.pending_target is None and self.outbound_baseline_after is not None
                        and now >= self.outbound_baseline_after):
                    self.start_odl, self.start_odr = odl, odr
                if self.mode == "DRIVE" and now - self.last_distance_report >= DISTANCE_REPORT_INTERVAL:
                    self.send_hc12(f"DISTANCE:{self._display_segment_distance():.2f}")
                    self.last_distance_report = now
                if (self.mode == "IDLE" and self.stop_pending and self.stop_sent_at is not None
                        and now >= self.stop_sent_at + STOP_SETTLE_SECONDS):
                    distance = self._segment_distance()
                    display_distance = self._display_segment_distance()
                    self.mission_outbound_total += distance
                    self.return_drive_distance = self.mission_outbound_total
                    self.stop_pending = False
                    self.start_odl = self.start_odr = None
                    requested = self.deferred_target
                    action = self.deferred_action
                    self._clear_deferred()
                    if requested is None:
                        self._send_final_distance(display_distance, "STOPPED")
                    else:
                        # 이전 구간 최종 거리를 새 앱 offset에 또 더하지 않음.
                        with self.tx_lock:
                            self.distance_message = None
                        self._accept_target(requested)
                    if action is not None:
                        if action == "RETURN":
                            # TARGET/RETURN이 함께 들어와도 복귀에 잔류 목표를 넘기지 않음.
                            self.pending_target = None
                            self.target_armed = False
                        self.handle_command(action)
                    print(f"[STOP] 구간={distance:.2f}m / 누적={self.return_drive_distance:.2f}m", flush=True)

            elif data.get("T") == 2102:
                if self.mode != "DRIVE" or not self.target_armed:
                    return
                try:
                    distance = float(data["distance"])
                    if not math.isfinite(distance) or distance < 0:
                        raise ValueError("invalid distance")
                except (KeyError, TypeError, ValueError):
                    self.set_fault("INVALID_ARRIVAL_DISTANCE")
                    return
                self._set_mode("IDLE")
                self.target_armed = False
                self.stop_pending = False
                self.mission_outbound_total += distance
                self.return_drive_distance = self.mission_outbound_total
                self.paused_mission = False
                self.start_odl = self.start_odr = None
                self.app_status = "ARRIVED"
                self._send_final_distance(max(0.0, distance + self.display_distance_bias), "ARRIVED")
                print(f"[도착] 정지. RETURN 명령 대기 / 복귀거리={self.return_drive_distance:.2f}m", flush=True)

    def publish_vision(self, error, detected, status, inferred, pair, shape, frame_time, epoch):
        with self.state_lock:
            if epoch != self.vision_epoch:
                return
            self.last_camera_frame_time = frame_time
            self.frame_time = frame_time
            self.frame_id += 1
            self.shared_error = float(error)
            self.lane_status, self.lane_inferred = status, inferred
            valid = bool(detected) and math.isfinite(error) and pair is not None
            reason = status
            if valid:
                h, w = shape[:2]
                pair = np.asarray(pair)
                valid = pair.shape == (2, 3) and np.isfinite(pair).all()
                if valid:
                    near = .94 if lane_detection.ROAD_MODE == "highway" else .90
                    samples = [.85, near]
                    widths = np.polyval(pair[1] - pair[0], samples)
                    minimum = w * MIN_LANE_WIDTH_RATIO
                    valid = bool(widths.min() >= minimum)
                    if valid and not self.vision_confirmed:
                        left, right = [np.polyval(c, near) for c in pair]
                        valid = bool(left < w / 2 < right)
                if not valid:
                    reason = "INVALID_LANE_GEOMETRY"
            if valid and status == "BOTH" and inferred is None:
                self.both_count += 1
                self.last_both_time = frame_time
                if self.both_count >= LANE_CONFIRM_FRAMES:
                    self.vision_confirmed = True
                valid = self.vision_confirmed
                if not valid:
                    reason = "CONFIRMING"
            elif valid and status in ("LEFT_ONLY", "RIGHT_ONLY"):
                valid = (self.vision_confirmed and self.last_both_time is not None
                         and 0 <= frame_time - self.last_both_time <= SINGLE_SIDE_MAX_SECONDS)
                if not valid:
                    reason = "SINGLE_SIDE_EXPIRED"
            else:
                self.both_count = 0
                self.vision_confirmed = False
                valid = False
            self.lane_detected = bool(valid)
            self.lane_reason = reason

    def camera_failed(self):
        with self.state_lock:
            self.last_camera_frame_time = None
            self.frame_time = None
            self.lane_detected = False
            self.both_count = 0
            self.vision_confirmed = False
            self._reset_pd()
        self.control_wake.set()

    def _write_raw(self, data):
        # 호출자는 제어 스레드이며 state_lock을 유지한 상태.
        packet = (json.dumps(data, allow_nan=False) + "\n").encode("utf-8")
        if self.ugv_serial.write(packet) != len(packet):
            raise IOError("UGV 명령 일부 전송")

    def _write_drive(self, speed, angular):
        ugv_control.send_driving_command(self.ugv_serial, speed, angular)

    def control_tick(self, now=None):
        now = self.clock() if now is None else now
        # 상태 확인과 실제 전송을 같은 잠금 안에서 수행.
        # STOP을 처리한 뒤 과거 모드로 계산한 명령이 전송되지 않음.
        with self.state_lock:
            if self.emergency_latched:
                if not self.emergency_sent:
                    self._write_raw({"T": 0})
                    self.emergency_sent = True
                return
            if self.fault:
                self._write_drive(0.0, 0.0)
                return
            if self.pending_target is not None:
                distance = self.pending_target
                self._write_raw({"T": 2101, "d": distance})
                self.pending_target = None
                self.target_armed = True
                self.target_ready_at = now + BASELINE_SETTLE_SECONDS
                if self.mode == "DRIVE":
                    self.start_odl = self.start_odr = None
                    self.outbound_baseline_after = self.target_ready_at
                self.send_hc12(f"TARGET:{distance:.2f}")

            mode = self.mode
            if mode != self.last_pd_mode:
                self._reset_pd()
                self.last_pd_mode = mode

            if self.stop_pending and now - self.stop_requested_at > STOP_FEEDBACK_TIMEOUT:
                self.set_fault("STOP_FEEDBACK_TIMEOUT")
                self._write_drive(0.0, 0.0)
                return
            if mode in AUTO_MODES and not self._odom_fresh(now):
                self.set_fault("ODOM_TIMEOUT")
                self._write_drive(0.0, 0.0)
                return

            if mode == "IDLE":
                self._write_drive(0.0, 0.0)
                if self.stop_pending and self.stop_sent_at is None:
                    self.stop_sent_at = now
                return

            if mode == "MANUAL":
                if (MANUAL_COMMAND_TIMEOUT is not None
                        and now - self.manual_received_at > MANUAL_COMMAND_TIMEOUT):
                    self._set_mode("IDLE")
                    self.manual_left_speed = self.manual_right_speed = 0.0
                    self._write_drive(0.0, 0.0)
                    self.send_hc12("MANUAL_STOPPED")
                    print("[수동 정지] 2초 제한. 버튼을 떼었다 다시 누르세요.", flush=True)
                else:
                    self._write_raw({"T": 1, "L": self.manual_left_speed, "R": self.manual_right_speed})
                return

            if mode == "RETURN_TURN":
                if now - self.stage_started_at > RETURN_TURN_TIMEOUT:
                    self.set_fault("RETURN_TURN_TIMEOUT")
                    self._write_drive(0.0, 0.0)
                    return
                if not self._camera_fresh(now):
                    self._write_drive(0.0, 0.0)
                    return
                if self.return_turn_start_odl is None:
                    self._write_drive(0.0, 0.0)
                    if now >= self.return_prepare_until and self.odom_received_at >= self.return_prepare_until:
                        self.return_turn_start_odl = self.current_odl
                        self.return_turn_start_odr = self.current_odr
                    return
                distance = self._return_progress(self.return_turn_start_odl, self.return_turn_start_odr)
                if distance >= COARSE_TURN_DISTANCE_M:
                    self._write_drive(0.0, 0.0)
                    self.stage_started_at = now
                    self._reset_vision()
                    self._set_mode("RETURN_ALIGN")
                    print("[RETURN] 엔코더 대략 회전 완료. 새 영상으로 정렬 시작", flush=True)
                else:
                    self._write_raw({"T": 1, "L": -TURN_SPEED, "R": TURN_SPEED})
                return

            if mode == "RETURN_ALIGN":
                if now - self.stage_started_at > RETURN_ALIGN_TIMEOUT:
                    self.set_fault("RETURN_ALIGN_TIMEOUT")
                    self._write_drive(0.0, 0.0)
                    return
                if not self._camera_fresh(now) or self.frame_time is None:
                    self._write_drive(0.0, 0.0)
                    self.align_stable_count = 0
                    return
                fresh = 0 <= now - self.frame_time <= FRAME_TIMEOUT
                if fresh and self.lane_reason == "CONFIRMING":
                    self._write_drive(0.0, 0.0)
                    self.align_stable_count = 0
                    return
                observed_both = (fresh and self.lane_detected and self.lane_status == "BOTH"
                                 and self.lane_inferred is None)
                if observed_both and abs(self.shared_error) <= ALIGN_ERROR_THRESHOLD:
                    self._write_drive(0.0, 0.0)
                    if self.frame_id != self.align_last_frame:
                        self.align_last_frame = self.frame_id
                        self.align_stable_count += 1
                    if self.align_stable_count >= ALIGN_STABLE_FRAMES:
                        self.return_drive_start_odl = self.current_odl
                        self.return_drive_start_odr = self.current_odr
                        self._set_mode("RETURN_DRIVE")
                        self.send_hc12("RETURN_DRIVING")
                        print("[RETURN] 새 영상 연속 정렬 완료. 복귀 주행", flush=True)
                else:
                    self.align_stable_count = 0
                    self.align_last_frame = self.frame_id
                    # 검출 시 오차 방향으로 보정, 미검출 시 기존 방향으로 탐색.
                    direction = -1 if observed_both and self.shared_error < 0 else 1
                    self._write_raw({"T": 1, "L": -direction * ALIGN_TURN_SPEED,
                                     "R": direction * ALIGN_TURN_SPEED})
                return

            if mode in ("DRIVE", "RETURN_DRIVE"):
                if mode == "DRIVE" and self.start_odl is None:
                    self._reset_pd()
                    self._write_drive(0.0, 0.0)
                    return
                if mode == "RETURN_DRIVE":
                    if self.return_drive_start_odl is None or self.return_drive_start_odr is None:
                        self.set_fault("RETURN_ERROR:NO_START_ODOM")
                        self._write_drive(0.0, 0.0)
                        return
                    distance = self._return_progress(self.return_drive_start_odl, self.return_drive_start_odr)
                    if distance >= self.return_drive_distance:
                        self._set_mode("IDLE")
                        self._write_drive(0.0, 0.0)
                        self.paused_mission = False
                        self.paused_return_mode = None
                        self.mission_outbound_total = self.return_drive_distance = 0.0
                        self.mission_goal_distance = None
                        self.display_distance_bias = 0.0
                        self.app_status = "RETURNED"
                        self.send_hc12("RETURNED")
                        print("[RETURN] 복귀 완료", flush=True)
                        return

                fresh = self.frame_time is not None and 0 <= now - self.frame_time <= FRAME_TIMEOUT
                if not fresh or not self.lane_detected:
                    self._reset_pd()
                    self._write_drive(0.0, 0.0)
                    if not self.lane_lost_printed:
                        self.send_hc12("LANE_LOST")
                        print(f"[정지] 차선/영상 재확인: {self.lane_reason}", flush=True)
                        self.lane_lost_printed = True
                    return
                self.lane_lost_printed = False
                if self.previous_frame_time is None or self.frame_time != self.previous_frame_time:
                    dt = self.frame_time - self.previous_frame_time if self.previous_frame_time is not None else 1.0
                    self.last_angular = ugv_control.calculate_pid(self.shared_error, self.previous_error, dt)
                    self.previous_error = self.shared_error
                    self.previous_frame_time = self.frame_time
                self._write_drive(BASE_SPEED, self.last_angular)
                if now - self.last_control_report >= 1.0:
                    print(f"[제어] {mode}, X={BASE_SPEED:.2f}, Z={self.last_angular:.3f}, "
                          f"error={self.shared_error:.1f}px, lane={self.lane_status}", flush=True)
                    self.last_control_report = now

    def control_thread_task(self):
        try:
            while self.running.is_set():
                self.control_tick()
                self.control_wake.wait(0.02)
                self.control_wake.clear()
        except Exception as exc:
            traceback.print_exc()
            self.set_fault(f"UGV_CONTROL:{type(exc).__name__}")
            self.running.clear()
        finally:
            try:
                if self.emergency_latched:
                    self._write_raw({"T": 0})
                else:
                    self._write_drive(0.0, 0.0)
            except Exception as exc:
                print(f"[에러] 최종 정지 명령 전송 실패: {exc}", flush=True)

    def hc12_thread_task(self):
        buffer = LineBuffer()
        try:
            while self.running.is_set():
                count = min(self.hc12_serial.in_waiting, 2048)
                if not count:
                    time.sleep(0.005)
                    continue
                for command in buffer.feed(self.hc12_serial.read(count)):
                    self.handle_command(command)
        except Exception:
            traceback.print_exc()
            self.set_fault("HC12_RX_ERROR")

    def hc12_tx_thread_task(self):
        try:
            while self.running.is_set():
                now = self.clock()
                if now - self.last_app_report >= APP_STATUS_INTERVAL:
                    self.last_app_report = now
                    for status in self.app_status_messages():
                        self.send_hc12(status)
                with self.tx_lock:
                    if self.events:
                        message = self.events.popleft()
                    else:
                        message, self.distance_message = self.distance_message, None
                if message is None:
                    self.tx_wake.wait(0.05)
                    self.tx_wake.clear()
                    continue
                wire_message = self.app_wire_message(message)
                if wire_message != message:
                    print(f"[HC12 TX] {message} -> {wire_message}", flush=True)
                data = (wire_message + "\n").encode("ascii")
                if self.hc12_serial.write(data) != len(data):
                    raise IOError("HC12 명령 일부 전송")
        except Exception:
            traceback.print_exc()
            self.set_fault("HC12_TX_ERROR")

    def ugv_receive_thread_task(self):
        buffer = LineBuffer()
        try:
            while self.running.is_set():
                count = min(self.ugv_serial.in_waiting, 2048)
                if not count:
                    time.sleep(0.005)
                    continue
                for line in buffer.feed(self.ugv_serial.read(count)):
                    self.handle_feedback(line)
        except Exception:
            traceback.print_exc()
            self.set_fault("UGV_RX_ERROR")

    def start(self):
        for target in (self.control_thread_task, self.ugv_receive_thread_task,
                       self.hc12_thread_task, self.hc12_tx_thread_task):
            thread = threading.Thread(target=target, name=target.__name__, daemon=True)
            thread.start()
            self.threads.append(thread)

    def shutdown(self):
        with self.state_lock:
            self.closing = True
            self.mode = "IDLE"
            self._reset_pd()
        self.control_wake.set()
        # 모터는 정지 상태로 두고 최종 앱 상태 메시지 전송에 짧은 여유 제공.
        deadline = time.monotonic() + 0.20
        while self.threads and self.running.is_set() and time.monotonic() < deadline:
            with self.tx_lock:
                pending = bool(self.events) or self.distance_message is not None
            if not pending:
                break
            self.tx_wake.set()
            time.sleep(0.01)
        self.running.clear()
        self.control_wake.set()
        self.tx_wake.set()
        for thread in self.threads:
            thread.join(timeout=2.0)
        for port in (self.hc12_serial, self.ugv_serial):
            try:
                port.close()
            except Exception as exc:
                print(f"[경고] 포트 닫기 실패: {exc}", flush=True)
        for thread in self.threads:
            if thread.is_alive():
                thread.join(timeout=0.5)
                if thread.is_alive():
                    print(f"[경고] 스레드 종료 지연: {thread.name}", flush=True)


def gstreamer_pipeline(sensor_id=0, capture_width=1640, capture_height=1232,
                       display_width=820, display_height=616, framerate=30, flip_method=0):
    return (
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! video/x-raw, format=(string)BGR ! "
        "appsink drop=true max-buffers=1 sync=false"
        % (sensor_id, capture_width, capture_height, framerate, flip_method,
           display_width, display_height)
    )


def process_lane_frame(frame, tracker, now):
    target_height = max(2, round(frame.shape[0] * FRAME_WIDTH / frame.shape[1]))
    frame = cv2.resize(frame, (FRAME_WIDTH, target_height))
    raw_edges, edges, top, bottom = lane_detection.get_edges(frame, return_raw=True)
    pair, status, inferred = tracker.update(edges, top, bottom, now)
    result_image, error, detected = lane_detection.draw_result(
        frame.copy(), pair, status, inferred, top, bottom)
    return error, detected, result_image, raw_edges, edges, pair, status, inferred


def main():
    parser = argparse.ArgumentParser(description="Smart Cone integrated ver2")
    parser.add_argument("--ugv-port", default=UGV_PORT)
    parser.add_argument("--hc12-port", default=HC12_PORT)
    parser.add_argument("--no-gui", action="store_true")
    args = parser.parse_args()
    show = SHOW_WINDOWS and not args.no_gui
    cap = ugv_serial = hc12_serial = controller = None
    exit_reason = "초기화 종료"
    try:
        if lane_detection.ROAD_MODE not in ("highway", "black_track"):
            raise ValueError("ROAD_MODE 설정 오류")
        ugv_serial = ugv_control.init_ugv(port=args.ugv_port)
        if ugv_serial is None:
            raise RuntimeError("UGV 연결 실패")
        ugv_control.send_driving_command(ugv_serial, 0.0, 0.0)
        hc12_serial = serial.Serial(args.hc12_port, HC12_BAUD, timeout=0, write_timeout=0.1)
        cap = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)
        if not cap.isOpened():
            raise RuntimeError("CSI 카메라 열기 실패")
        tracker = lane_detection.LaneTracker()
        controller = SmartConeController(ugv_serial, hc12_serial)
        local_epoch = controller.vision_epoch
        if show:
            cv2.namedWindow("Smart Cone - Integrated ver2")
            cv2.namedWindow("Canny - Raw | Lane ROI")
        controller.start()
        print("[READY] 앱 START 전에는 주행하지 않습니다.", flush=True)
        print(f"[설정] ROAD_MODE={lane_detection.ROAD_MODE}, BASE_SPEED={BASE_SPEED}, GUI={show}", flush=True)
        failures = 0
        report_at = time.monotonic()
        while controller.running.is_set():
            if not cap.isOpened():
                raise RuntimeError("실행 중 카메라 연결 종료")
            with controller.state_lock:
                epoch = controller.vision_epoch
            if epoch != local_epoch:
                tracker.reset()
                local_epoch = epoch
            frame_time = time.monotonic()
            ok, frame = cap.read()
            if not ok or frame is None or frame.size == 0:
                controller.camera_failed()
                tracker.reset()
                failures += 1
                print(f"[경고] 카메라 수신 실패 {failures}/{MAX_CAMERA_FAILURES}", flush=True)
                if failures == 1:
                    controller.send_hc12("CAMERA_ERROR")
                if failures >= MAX_CAMERA_FAILURES:
                    raise RuntimeError("카메라 프레임 연속 수신 실패")
                time.sleep(0.05)
                continue
            failures = 0
            error, detected, image, raw, edges, pair, status, inferred = process_lane_frame(frame, tracker, frame_time)
            if not math.isfinite(error):
                raise ValueError("유효하지 않은 차선 오차")
            controller.publish_vision(error, detected, status, inferred, pair, image.shape, frame_time, epoch)
            with controller.state_lock:
                mode, ready, reason, fault = controller.mode, controller.lane_detected, controller.lane_reason, controller.fault
            if time.monotonic() - report_at >= 1.0:
                print(f"[상태] {mode}, 차선={status}, 주행차선허용={ready}, 이유={reason}, 오류={fault}", flush=True)
                report_at = time.monotonic()
            if show:
                cv2.putText(image, f"MODE: {mode} / GATE: {ready} / {reason}",
                            (15, 80), cv2.FONT_HERSHEY_SIMPLEX, .45, (0, 200, 255), 1)
                edge_view = cv2.cvtColor(np.hstack([raw, edges]), cv2.COLOR_GRAY2BGR)
                for label, x in (("RAW CANNY", 12), ("LANE ROI", raw.shape[1] + 12)):
                    cv2.putText(edge_view, label, (x, 25), cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 255, 255), 1)
                cv2.imshow("Smart Cone - Integrated ver2", image)
                cv2.imshow("Canny - Raw | Lane ROI", edge_view)
                if (cv2.waitKey(1) & 0xFF) in (ord("q"), ord("Q"), 27):
                    exit_reason = "사용자 종료 키"
                    break
        else:
            exit_reason = "제어 스레드 중단"
    except KeyboardInterrupt:
        exit_reason = "Ctrl+C 종료"
    except Exception as exc:
        exit_reason = f"{type(exc).__name__}: {exc}"
        traceback.print_exc()
        if controller is not None:
            controller.set_fault("CAMERA_OR_MAIN_ERROR")
    finally:
        if controller is not None:
            controller.shutdown()
        else:
            if ugv_serial is not None:
                ugv_control.stop_ugv(ugv_serial)
            if hc12_serial is not None:
                hc12_serial.close()
        if cap is not None:
            cap.release()
        if show:
            cv2.destroyAllWindows()
        print(f"[종료 사유] {exit_reason}", flush=True)


if __name__ == "__main__":
    main()
