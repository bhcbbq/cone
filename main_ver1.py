import cv2
import os
import time
import math
import traceback
import threading
import numpy as np

import lane_detection_ver1 as lane_detection
import ugv_control


BASE_SPEED = 0.3
FRAME_WIDTH = 640

# 이 시간 안에 새 영상 결과가 없으면 정지
FRAME_TIMEOUT = 0.5

# 카메라 프레임을 연속으로 받지 못하면 종료
MAX_CAMERA_FAILURES = 3

# Jetson 로컬 GUI에서는 화면 표시
# SSH 접속에서는 화면 숨김
SHOW_WINDOWS = bool(os.environ.get("DISPLAY")) and not bool(
    os.environ.get("SSH_CONNECTION")
    or os.environ.get("SSH_CLIENT")
    or os.environ.get("SSH_TTY")
)

shared_error = 0.0
shared_speed = 0.0
shared_frame_time = None

error_lock = threading.Lock()
is_running = True
ugv_serial = None
control_failure = None


# ==========================================
# 백그라운드 모터 제어
# ==========================================
def control_thread_task():
    global is_running
    global shared_error, shared_speed
    global control_failure

    previous_error = None
    last_time = time.monotonic()
    stale_reported = False

    print(
        "[알림] 제어 백그라운드 스레드 시작",
        flush=True,
    )

    try:
        while is_running:
            now = time.monotonic()
            dt = max(now - last_time, 0.001)
            last_time = now

            # 현재는 거리 적분 결과를 사용하지 않음.
            # 수신 데이터를 최대 2048바이트까지만 비움.
            ugv_control.clear_feedback_buffer(ugv_serial)

            with error_lock:
                current_error = shared_error
                current_speed = shared_speed
                frame_time = shared_frame_time

            stale = (
                frame_time is None
                or now - frame_time > FRAME_TIMEOUT
            )

            if stale or current_speed <= 0:
                current_speed = 0.0
                angular_z = 0.0

                # 정지 후 재출발 시 미분항 초기화
                previous_error = None

                if (
                    stale
                    and frame_time is not None
                    and not stale_reported
                ):
                    print(
                        "[정지] 새 영상 결과 지연. "
                        "최신 결과를 기다립니다.",
                        flush=True,
                    )
                    stale_reported = True

            else:
                stale_reported = False

                # 차선 재인식 직후 미분항 급변 방지
                if previous_error is None:
                    previous_error = current_error

                angular_z = ugv_control.calculate_pid(
                    current_error,
                    previous_error,
                    dt,
                )
                previous_error = current_error

            ugv_control.send_driving_command(
                ugv_serial,
                current_speed,
                angular_z,
            )

            time.sleep(0.02)

    except Exception as exc:
        control_failure = (
            f"{type(exc).__name__}: {exc}"
        )

        print(
            f"[에러] 제어 스레드 중단: {control_failure}",
            flush=True,
        )
        traceback.print_exc()

        with error_lock:
            shared_error = 0.0
            shared_speed = 0.0

        is_running = False

    finally:
        # 메인이 카메라에서 대기하더라도 정지를 먼저 시도
        try:
            ugv_control.send_driving_command(
                ugv_serial,
                0.0,
                0.0,
            )
        except Exception as exc:
            print(
                "[에러] 제어 스레드 정지 명령 "
                f"전송 실패: {exc}",
                flush=True,
            )


# ==========================================
# 기존 카메라 설정 유지
# ==========================================
def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1640,
    capture_height=1232,
    display_width=820,
    display_height=616,
    framerate=30,
    flip_method=0,
):
    return (
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, "
        "format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! "
        "appsink drop=true max-buffers=1 sync=false"
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )


# ==========================================
# 기존 차선인식 처리 유지
# ==========================================
def process_lane_frame(frame, tracker, now):
    """검출 모듈의 함수를 호출하여 결과를 받습니다."""
    target_height = max(
        2,
        round(
            frame.shape[0]
            * FRAME_WIDTH
            / frame.shape[1]
        ),
    )

    frame = cv2.resize(
        frame,
        (FRAME_WIDTH, target_height),
    )

    raw_edges, edges, top, bottom = (
        lane_detection.get_edges(
            frame,
            return_raw=True,
        )
    )

    pair, status, inferred = tracker.update(
        edges,
        top,
        bottom,
        now,
    )

    result_image, error, is_detected = (
        lane_detection.draw_result(
            frame.copy(),
            pair,
            status,
            inferred,
            top,
            bottom,
        )
    )

    return (
        error,
        result_image,
        is_detected,
        raw_edges,
        edges,
        status,
    )


# ==========================================
# 메인 실행
# ==========================================
def main():
    global ugv_serial, is_running
    global shared_error, shared_speed
    global shared_frame_time, control_failure

    if lane_detection.ROAD_MODE not in (
        "highway",
        "black_track",
    ):
        print(
            "[에러] ROAD_MODE 설정을 확인하세요.",
            flush=True,
        )
        return

    is_running = True
    control_failure = None

    with error_lock:
        shared_error = 0.0
        shared_speed = 0.0
        shared_frame_time = None

    cap = None
    control_thread = None

    exit_reason = "초기화 중 종료"

    window_name = "UGV - ver1 Lane Tracking"
    edge_window_name = "Canny - Raw | Lane ROI"

    try:
        ugv_serial = ugv_control.init_ugv()

        if not ugv_serial:
            exit_reason = (
                "UGV02 연결 실패. 포트를 확인하세요."
            )
            return

        # 카메라 초기화 전 정지 명령
        ugv_control.send_driving_command(
            ugv_serial,
            0.0,
            0.0,
        )

        print(
            "[알림] IMX219 CSI 카메라 초기화 중",
            flush=True,
        )

        cap = cv2.VideoCapture(
            gstreamer_pipeline(flip_method=0),
            cv2.CAP_GSTREAMER,
        )

        if not cap.isOpened():
            exit_reason = (
                "카메라를 열 수 없습니다."
            )
            return

        # 이전 차선과 차선 폭을 기억하도록 한 번만 생성
        tracker = lane_detection.LaneTracker()

        if SHOW_WINDOWS:
            cv2.namedWindow(window_name)
            cv2.namedWindow(edge_window_name)

        control_thread = threading.Thread(
            target=control_thread_task,
            daemon=True,
        )
        control_thread.start()

        print(
            "[알림] 차선 추적 시작 / "
            f"ROAD_MODE={lane_detection.ROAD_MODE}",
            flush=True,
        )
        print(
            f"[알림] 화면 표시={SHOW_WINDOWS} / "
            "종료: Ctrl+C, 화면에서는 Q 또는 Esc",
            flush=True,
        )

        failure_count = 0
        report_time = time.monotonic()

        while is_running:
            if not cap.isOpened():
                exit_reason = (
                    "실행 중 카메라 연결이 닫혔습니다."
                )
                break

            # 영상 읽기와 처리에 걸린 시간까지 포함
            frame_started = time.monotonic()

            ret, frame = cap.read()

            if (
                not ret
                or frame is None
                or frame.size == 0
            ):
                with error_lock:
                    shared_error = 0.0
                    shared_speed = 0.0
                    shared_frame_time = None

                # 영상 공백 이후 이전 차선을 그대로 재사용하지 않음
                tracker.reset()

                failure_count += 1

                print(
                    "[경고] 카메라 프레임 수신 실패 "
                    f"({failure_count}/{MAX_CAMERA_FAILURES}). "
                    "정지 상태.",
                    flush=True,
                )

                if (
                    failure_count
                    >= MAX_CAMERA_FAILURES
                ):
                    exit_reason = (
                        "카메라 프레임 연속 수신 실패"
                    )
                    break

                time.sleep(0.05)
                continue

            failure_count = 0

            (
                error,
                result_image,
                is_detected,
                raw_edges,
                edges,
                status,
            ) = process_lane_frame(
                frame,
                tracker,
                frame_started,
            )

            if not math.isfinite(error):
                raise ValueError(
                    "차선 오차가 유효하지 않습니다: "
                    f"{error}"
                )

            with error_lock:
                if not is_running:
                    break

                shared_error = float(error)

                shared_speed = (
                    BASE_SPEED
                    if is_detected
                    else 0.0
                )

                shared_frame_time = frame_started

            # SSH에서도 상태를 확인할 수 있도록 1초마다 출력
            now = time.monotonic()

            if now - report_time >= 1.0:
                usable = (
                    bool(is_detected)
                    and now - frame_started
                    <= FRAME_TIMEOUT
                )

                print(
                    f"[상태] 차선={status}, "
                    f"오차={error:.1f}px, "
                    "영상 처리="
                    f"{now - frame_started:.3f}s, "
                    f"주행허용={usable}",
                    flush=True,
                )

                report_time = now

            if SHOW_WINDOWS:
                edge_view = cv2.cvtColor(
                    np.hstack([
                        raw_edges,
                        edges,
                    ]),
                    cv2.COLOR_GRAY2BGR,
                )

                for text, x in (
                    ("RAW CANNY", 12),
                    (
                        "LANE ROI",
                        raw_edges.shape[1] + 12,
                    ),
                ):
                    cv2.putText(
                        edge_view,
                        text,
                        (x, 25),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 255),
                        1,
                    )

                cv2.imshow(
                    window_name,
                    result_image,
                )

                cv2.imshow(
                    edge_window_name,
                    edge_view,
                )

                key = cv2.waitKey(1) & 0xFF

                if key in (
                    ord("q"),
                    ord("Q"),
                    27,
                ):
                    exit_reason = (
                        "사용자 종료 키 입력"
                    )
                    break

                if any(
                    cv2.getWindowProperty(
                        name,
                        cv2.WND_PROP_VISIBLE,
                    ) < 1
                    for name in (
                        window_name,
                        edge_window_name,
                    )
                ):
                    exit_reason = (
                        "영상 창 닫힘 또는 "
                        "창 상태 확인 실패"
                    )
                    break

        if control_failure is not None:
            exit_reason = (
                f"제어 오류: {control_failure}"
            )

    except KeyboardInterrupt:
        exit_reason = "Ctrl+C 종료"

    except Exception as exc:
        exit_reason = (
            f"{type(exc).__name__}: {exc}"
        )

        print(
            "[에러] 메인 실행 중 예외 발생",
            flush=True,
        )
        traceback.print_exc()

    finally:
        with error_lock:
            shared_error = 0.0
            shared_speed = 0.0
            shared_frame_time = None

        is_running = False

        # 제어 스레드에서 정지를 먼저 보내고 종료
        # 이후 메인에서 포트를 닫음
        if control_thread is not None:
            control_thread.join()

        ugv_control.stop_ugv(ugv_serial)
        ugv_serial = None

        print(
            f"[종료 사유] {exit_reason}",
            flush=True,
        )

        if cap is not None:
            cap.release()

        if SHOW_WINDOWS:
            cv2.destroyAllWindows()

        print(
            "[알림] 시스템 종료 완료",
            flush=True,
        )


if __name__ == "__main__":
    main()