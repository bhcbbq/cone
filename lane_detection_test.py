import cv2
import numpy as np
import math


# ==========================================
# 기존 lane_detection.py 코드
# ==========================================
def region_of_interest(img, vertices):
    """
    영상에서 차선인식 영역만 남깁니다.
    """
    mask = np.zeros_like(img)
    cv2.fillPoly(mask, vertices, 255)
    return cv2.bitwise_and(img, mask)


def calculate_steering(frame, lines):
    """
    차선 분석 후 (error, frame, is_detected)를 반환합니다.
    이 테스트에서는 모터를 제어하지 않습니다.
    """
    height, width = frame.shape[:2]
    camera_center = width // 2

    if lines is None:
        cv2.putText(
            frame,
            "STATUS: No Lines Found! (STOPPED)",
            (30, 90),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 0, 255),
            2
        )
        return 0, frame, False

    left_line_x, left_line_y = [], []
    right_line_x, right_line_y = [], []

    for line in lines:
        x1, y1, x2, y2 = line.flatten()

        if x1 == x2:
            continue

        slope = (y2 - y1) / (x2 - x1)

        if math.fabs(slope) < 0.5:
            continue

        if slope < 0:
            left_line_x.extend([x1, x2])
            left_line_y.extend([y1, y2])
        else:
            right_line_x.extend([x1, x2])
            right_line_y.extend([y1, y2])

    y_bottom = height
    y_top = height // 2 + 50

    left_x_bottom, right_x_bottom = None, None
    left_x_top, right_x_top = None, None

    if len(left_line_x) > 0:
        poly_left = np.polyfit(left_line_y, left_line_x, 1)
        left_x_bottom = int(np.polyval(poly_left, y_bottom))
        left_x_top = int(np.polyval(poly_left, y_top))

        cv2.line(
            frame,
            (left_x_bottom, y_bottom),
            (left_x_top, y_top),
            (0, 255, 0),
            6
        )

    if len(right_line_x) > 0:
        poly_right = np.polyfit(right_line_y, right_line_x, 1)
        right_x_bottom = int(np.polyval(poly_right, y_bottom))
        right_x_top = int(np.polyval(poly_right, y_top))

        cv2.line(
            frame,
            (right_x_bottom, y_bottom),
            (right_x_top, y_top),
            (0, 255, 0),
            6
        )

    if left_x_bottom is None and right_x_bottom is None:
        cv2.putText(
            frame,
            "STATUS: Lane Lost! (STOPPED)",
            (30, 90),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 0, 255),
            2
        )
        return 0, frame, False

    if left_x_bottom is not None and right_x_bottom is not None:
        lane_center_bottom = (left_x_bottom + right_x_bottom) // 2
        lane_center_top = (left_x_top + right_x_top) // 2

    elif left_x_bottom is not None:
        # 기존 코드의 한쪽 차선 보정값 유지
        lane_center_bottom = left_x_bottom + 160
        lane_center_top = left_x_top + 160

    else:
        lane_center_bottom = right_x_bottom - 160
        lane_center_top = right_x_top - 160

    cv2.line(
        frame,
        (lane_center_bottom, y_bottom),
        (lane_center_top, y_top),
        (0, 255, 255),
        3
    )

    error = camera_center - lane_center_bottom
    error_margin = 20

    cv2.circle(
        frame,
        (camera_center, y_bottom - 40),
        8,
        (255, 0, 0),
        -1
    )

    cv2.circle(
        frame,
        (lane_center_bottom, y_bottom - 40),
        8,
        (0, 255, 0),
        -1
    )

    cv2.line(
        frame,
        (camera_center, y_bottom - 40),
        (lane_center_bottom, y_bottom - 40),
        (0, 0, 255),
        4
    )

    status_text = (
        "Status: Driving (On Track)"
        if abs(error) <= error_margin
        else f"Status: Turning ({'Left' if error > 0 else 'Right'})"
    )

    cv2.putText(
        frame,
        f"Error: {error} px",
        (30, 50),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 255, 255),
        2
    )

    cv2.putText(
        frame,
        status_text,
        (30, 90),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 255, 0),
        2
    )

    return error, frame, True


# ==========================================
# 기존 main.py의 카메라 설정
# ==========================================
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
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height
        )
    )


# ==========================================
# 카메라 차선인식 테스트 실행
# ==========================================
if __name__ == "__main__":
    cap = None

    try:
        print("[알림] IMX219 CSI 카메라를 초기화 중입니다...")

        pipeline = gstreamer_pipeline(flip_method=0)
        cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

        if not cap.isOpened():
            print("[에러] 카메라를 열 수 없습니다!")
            raise SystemExit(1)

        window_name = 'Future Makers - UGV02 Autonomous Driving'
        cv2.namedWindow(window_name)

        print("[알림] 차선인식 화면 테스트를 시작합니다.")
        print("[알림] 모터 제어 기능은 없습니다.")
        print("[종료] 영상 창에서 q 또는 터미널에서 Ctrl+C")

        while cap.isOpened():
            ret, frame = cap.read()

            if not ret:
                print("[경고] 카메라 영상을 받아오지 못했습니다.")
                break

            height, width = frame.shape[:2]

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            blur = cv2.GaussianBlur(gray, (5, 5), 0)
            edges = cv2.Canny(blur, 50, 150)

            # 기존 main.py의 인식 범위 유지
            roi_vertices = [
                (0, height),
                (int(width * 0.05), int(height * 0.5)),
                (int(width * 0.95), int(height * 0.5)),
                (width, height)
            ]

            cropped_edges = region_of_interest(
                edges,
                np.array([roi_vertices], np.int32)
            )

            lines = cv2.HoughLinesP(
                cropped_edges,
                rho=1,
                theta=np.pi / 180,
                threshold=40,
                minLineLength=20,
                maxLineGap=10
            )

            error, result_image, is_detected = calculate_steering(
                frame.copy(),
                lines
            )

            # 기존 main.py의 화면 표시 유지
            cv2.imshow(window_name, result_image)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("[알림] 사용자가 'q'를 눌러 프로그램을 종료했습니다.")
                break

    except KeyboardInterrupt:
        print("\n[알림] 강제 종료(Ctrl+C) 신호를 감지했습니다.")

    finally:
        if cap is not None:
            cap.release()

        cv2.destroyAllWindows()
        print("[알림] 차선인식 테스트를 종료했습니다.")