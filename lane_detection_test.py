import os
import sys

import cv2
import numpy as np

import lane_detection


CAMERA_PIPELINE = (
    "nvarguscamerasrc sensor-id=0 ! "
    "video/x-raw(memory:NVMM), width=(int)1280, height=(int)720, "
    "framerate=(fraction)30/1 ! "
    "nvvidconv flip-method=0 ! "
    "video/x-raw, width=(int)640, height=(int)360, format=(string)BGRx ! "
    "videoconvert ! "
    "video/x-raw, format=(string)BGR ! "
    "appsink drop=true max-buffers=1 sync=false"
)

LANE_WINDOW = "Smart Cone - Lane Tracking"
CANNY_WINDOW = "Smart Cone - Raw Canny | Lane ROI"


def process_frame(frame):
    height, width = frame.shape[:2]
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, 50, 150)

    roi_vertices = np.array([[
        (0, height),
        (int(width * 0.1), int(height * 0.5)),
        (int(width * 0.9), int(height * 0.5)),
        (width, height),
    ]], dtype=np.int32)
    cropped_edges = lane_detection.region_of_interest(edges, roi_vertices)
    lines = cv2.HoughLinesP(
        cropped_edges,
        rho=1,
        theta=np.pi / 180,
        threshold=40,
        minLineLength=20,
        maxLineGap=10,
    )
    _, result_image, _ = lane_detection.calculate_steering(frame.copy(), lines)
    return result_image, np.hstack((edges, cropped_edges))


def main():
    if not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        print("[오류] 젯슨 데스크톱 또는 TigerVNC 안의 터미널에서 실행하세요.", file=sys.stderr)
        return 1

    cap = cv2.VideoCapture(CAMERA_PIPELINE, cv2.CAP_GSTREAMER)
    try:
        if not cap.isOpened():
            print("[오류] CSI 카메라를 열 수 없습니다.", file=sys.stderr)
            return 1

        cv2.namedWindow(LANE_WINDOW, cv2.WINDOW_NORMAL)
        cv2.namedWindow(CANNY_WINDOW, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(LANE_WINDOW, 640, 360)
        cv2.resizeWindow(CANNY_WINDOW, 960, 270)
        print("[테스트] 창을 선택하고 Q 또는 Esc를 누르면 종료합니다.")

        while True:
            ok, frame = cap.read()
            if not ok or frame is None or frame.size == 0:
                print("[오류] CSI 카메라 프레임을 읽지 못했습니다.", file=sys.stderr)
                return 1

            result_image, canny_view = process_frame(frame)
            cv2.imshow(LANE_WINDOW, result_image)
            cv2.imshow(CANNY_WINDOW, canny_view)

            if cv2.waitKey(1) & 0xFF in (ord("q"), ord("Q"), 27):
                return 0
    except KeyboardInterrupt:
        return 0
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    raise SystemExit(main())
