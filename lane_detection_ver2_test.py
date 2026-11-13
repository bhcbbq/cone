import os
import sys
import time

import cv2
import numpy as np

import lane_detection_ver2 as lane_detection


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


def main():
    if not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        print("[오류] 화면을 표시할 수 없습니다. Jetson 데스크톱이나 TigerVNC 안의 터미널에서 실행하세요.", file=sys.stderr)
        return 1

    cap = cv2.VideoCapture(CAMERA_PIPELINE, cv2.CAP_GSTREAMER)
    try:
        if not cap.isOpened():
            print("[오류] CSI 카메라를 열 수 없습니다.", file=sys.stderr)
            return 1

        tracker = lane_detection.LaneTracker()
        cv2.namedWindow(LANE_WINDOW, cv2.WINDOW_NORMAL)
        cv2.namedWindow(CANNY_WINDOW, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(LANE_WINDOW, 640, 360)
        cv2.resizeWindow(CANNY_WINDOW, 960, 270)
        print("[테스트] 카메라와 차선 인식 확인 중. 창을 선택하고 Q 또는 Esc를 누르면 종료합니다.")

        while True:
            ok, frame = cap.read()
            if not ok or frame is None or frame.size == 0:
                print("[오류] CSI 카메라 프레임을 읽지 못했습니다.", file=sys.stderr)
                return 1

            raw_edges, lane_edges = lane_detection.lane_edges(frame)
            pair, status = tracker.update(lane_edges, time.monotonic(), frame=frame)
            result, _, _ = lane_detection.draw_result(frame, pair, status)

            cv2.imshow(LANE_WINDOW, result)
            cv2.imshow(CANNY_WINDOW, np.hstack((raw_edges, lane_edges)))

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q"), 27):
                return 0
    except KeyboardInterrupt:
        return 0
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    raise SystemExit(main())
