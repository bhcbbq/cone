import cv2
import time
import threading
import numpy as np

import lane_detection_ver1 as lane_detection
import ugv_control


BASE_SPEED = 0.3
FRAME_WIDTH = 640

# 화면을 띄울 때 True
# SSH에서 화면 없이 실행할 때 False
SHOW_WINDOWS = True

shared_error = 0.0
shared_speed = 0.0

error_lock = threading.Lock()
is_running = True
ugv_serial = None


def control_thread_task():
    global is_running, shared_error, shared_speed

    previous_error = 0.0
    last_time = time.time()

    print("[알림] 제어 백그라운드 스레드가 시작되었습니다.")

    try:
        while is_running:
            current_time = time.time()
            dt = current_time - last_time

            if dt <= 0:
                dt = 0.001

            last_time = current_time

            if ugv_serial:
                ugv_control.read_odometry(ugv_serial, dt)

            with error_lock:
                current_error = shared_error
                current_speed = shared_speed

            angular_z = ugv_control.calculate_pid(
                current_error,
                previous_error,
                dt,
            )
            previous_error = current_error

            if ugv_serial:
                send_angular = (
                    angular_z if current_speed > 0 else 0.0
                )

                ugv_control.send_driving_command(
                    ugv_serial,
                    current_speed,
                    send_angular,
                )

            time.sleep(0.02)

    except Exception as exc:
        print(f"[에러] 제어 스레드 중단: {exc}")

        with error_lock:
            shared_error = 0.0
            shared_speed = 0.0

        is_running = False


def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1280,
    capture_height=720,
    display_width=640,
    display_height=360,
    framerate=30,
    flip_method=0,
):
    return (
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, "
        "format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
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


def process_lane_frame(frame, tracker, now):
    """검출 모듈의 함수를 호출하여 결과를 받습니다."""
    target_height = max(
        2,
        round(frame.shape[0] * FRAME_WIDTH / frame.shape[1]),
    )

    frame = cv2.resize(
        frame,
        (FRAME_WIDTH, target_height),
    )

    raw_edges, edges, top, bottom = lane_detection.get_edges(
        frame,
        return_raw=True,
    )

    pair, status, inferred = tracker.update(
        edges,
        top,
        bottom,
        now,
    )

    result_image, error, is_detected = lane_detection.draw_result(
        frame.copy(),
        pair,
        status,
        inferred,
        top,
        bottom,
    )

    return (
        error,
        result_image,
        is_detected,
        raw_edges,
        edges,
        status,
    )


def main():
    global ugv_serial, is_running
    global shared_error, shared_speed

    if lane_detection.ROAD_MODE not in (
        "highway",
        "black_track",
    ):
        print("[에러] ROAD_MODE 설정을 확인하세요.")
        return

    is_running = True

    with error_lock:
        shared_error = 0.0
        shared_speed = 0.0

    cap = None
    control_thread = None

    window_name = "UGV - ver1 Lane Tracking"
    edge_window_name = "Canny - Raw | Lane ROI"

    try:
        ugv_serial = ugv_control.init_ugv()

        if not ugv_serial:
            print("[종료] UGV02 연결 실패. 포트를 확인하세요.")
            return

        print("[알림] IMX219 CSI 카메라를 초기화 중입니다...")

        cap = cv2.VideoCapture(
            gstreamer_pipeline(flip_method=0),
            cv2.CAP_GSTREAMER,
        )

        if not cap.isOpened():
            print("[에러] 카메라를 열 수 없습니다.")
            return

        # 이전 차선과 폭을 기억하도록 한 번만 생성
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
            f"ROAD_MODE={lane_detection.ROAD_MODE}"
        )
        print("[알림] 종료: Ctrl+C")
        print("[알림] 화면 사용 시 Q 또는 Esc로도 종료")

        while is_running and cap.isOpened():
            ret, frame = cap.read()

            if not ret or frame is None:
                print("[경고] 카메라 프레임 수신 실패.")
                break

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
                time.monotonic(),
            )

            with error_lock:
                if not is_running:
                    break

                shared_error = error
                shared_speed = (
                    BASE_SPEED if is_detected else 0.0
                )

            # 화면 출력은 main에서만 실행
            if SHOW_WINDOWS:
                edge_view = cv2.cvtColor(
                    np.hstack([raw_edges, edges]),
                    cv2.COLOR_GRAY2BGR,
                )

                for text, x in (
                    ("RAW CANNY", 12),
                    ("LANE ROI", raw_edges.shape[1] + 12),
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

                # 카메라 영상 + 차선 인식 결과
                cv2.imshow(window_name, result_image)

                # 전체 Canny + 차선 검출용 에지
                cv2.imshow(edge_window_name, edge_view)

                key = cv2.waitKey(1) & 0xFF

                if key in (ord("q"), ord("Q"), 27):
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
                    break

    except KeyboardInterrupt:
        print("\n[알림] Ctrl+C 종료.")

    finally:
        print("[알림] 시스템을 종료합니다.")

        with error_lock:
            shared_error = 0.0
            shared_speed = 0.0

        is_running = False

        if control_thread is not None:
            control_thread.join()

        ugv_control.stop_ugv(ugv_serial)
        ugv_serial = None

        if cap is not None:
            cap.release()

        if SHOW_WINDOWS:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()