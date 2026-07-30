import cv2
import numpy as np
import math

# ==========================================
# 1. CSI 카메라 GStreamer 파이프라인 설정 함수
# ==========================================
def gstreamer_pipeline(
    capture_width=1280,
    capture_height=720,
    display_width=640,
    display_height=480,
    framerate=30,
    flip_method=0,
):
    return (
        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM), "
        f"width=(int){capture_width}, height=(int){capture_height}, "
        f"format=(string)NV12, framerate=(fraction){framerate}/1 ! "
        f"nvvidconv flip-method={flip_method} ! "
        f"video/x-raw, width=(int){display_width}, height=(int){display_height}, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! "
        "appsink"
    )

# ==========================================
# 2. 차선 인식 및 조향(Steering) 계산 함수
# ==========================================
def calculate_steering(frame, lines):
    height, width = frame.shape[:2]
    camera_center = width // 2  # 라바콘(카메라)의 현재 위치 (화면 정중앙 기준점)

    if lines is None:
        return 0, frame 

    left_line_x = []
    left_line_y = []
    right_line_x = []
    right_line_y = []

    # 선분들을 왼쪽/오른쪽 차선으로 분류
    for line in lines:
        x1, y1, x2, y2 = line.flatten()
        if x1 == x2: continue 
        
        slope = (y2 - y1) / (x2 - x1)
        if math.fabs(slope) < 0.5: continue

        if slope < 0: 
            left_line_x.extend([x1, x2])
            left_line_y.extend([y1, y2])
        else:        
            right_line_x.extend([x1, x2])
            right_line_y.extend([y1, y2])

    y_bottom = height
    y_top = height // 2 + 50 

    left_x_bottom = None
    right_x_bottom = None
    left_x_top = None
    right_x_top = None

    # 양쪽 초록색 차선 연장선 그리기 및 좌표 추출
    if len(left_line_x) > 0:
        poly_left = np.polyfit(left_line_y, left_line_x, 1) 
        left_x_bottom = int(np.polyval(poly_left, y_bottom))
        left_x_top = int(np.polyval(poly_left, y_top))
        cv2.line(frame, (left_x_bottom, y_bottom), (left_x_top, y_top), (0, 255, 0), 8)
        
    if len(right_line_x) > 0:
        poly_right = np.polyfit(right_line_y, right_line_x, 1)
        right_x_bottom = int(np.polyval(poly_right, y_bottom))
        right_x_top = int(np.polyval(poly_right, y_top))
        cv2.line(frame, (right_x_bottom, y_bottom), (right_x_top, y_top), (0, 255, 0), 8)

    # 차로의 실제 정중앙값(Center) 계산
    lane_center_bottom = camera_center
    lane_center_top = camera_center

    if left_x_bottom is not None and right_x_bottom is not None:
        lane_center_bottom = (left_x_bottom + right_x_bottom) // 2

    if left_x_top is not None and right_x_top is not None:
        lane_center_top = (left_x_top + right_x_top) // 2

    # 차로의 정중앙 노선을 나타내는 노란색 수직선 그리기
    cv2.line(frame, (lane_center_bottom, y_bottom), (lane_center_top, y_top), (0, 255, 255), 3)

    # 오차(Error) 계산
    error = camera_center - lane_center_bottom
    error_margin = 20
    
    # 시각화: 현재 오차 거리를 보여주는 하단 가로 빨간선 및 고정 점들
    cv2.circle(frame, (camera_center, y_bottom - 40), 8, (255, 0, 0), -1)        # 파란 점: 라바콘 위치
    cv2.circle(frame, (lane_center_bottom, y_bottom - 40), 8, (0, 255, 0), -1)  # 초록 점: 실제 차로 중앙
    cv2.line(frame, (camera_center, y_bottom - 40), (lane_center_bottom, y_bottom - 40), (0, 0, 255), 4)

    # 오차범위 판별에 따른 상태 메시지 출력
    if abs(error) <= error_margin:
        status_text = "Status: Stable (On Track)"
        status_color = (0, 255, 0)
    else:
        direction = "Left" if error > 0 else "Right"
        status_text = f"Status: Alert (Turn {direction})"
        status_color = (0, 0, 255)

    # 화면 모니터링 출력
    cv2.putText(frame, f"Error: {error} px (Margin: +/-{error_margin}px)", (30, 50), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
    cv2.putText(frame, status_text, (30, 90), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2)

    return error, frame

# 트랙바 조작을 위한 더미(Dummy) 함수
def nothing(x):
    pass

# 관심 영역(ROI)을 잘라내는 함수
def region_of_interest(img, vertices):
    mask = np.zeros_like(img)
    match_mask_color = 255
    cv2.fillPoly(mask, vertices, match_mask_color)
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image

# ==========================================
# 3. 메인 실행 파트
# ==========================================
if __name__ == "__main__":
    # 윈도우 생성 및 트랙바 부착
    window_name = 'Future Makers - Lane Detection'
    cv2.namedWindow(window_name)
    cv2.createTrackbar('Low_Threshold', window_name, 50, 255, nothing)
    cv2.createTrackbar('High_Threshold', window_name, 150, 255, nothing)

    # 동영상 대신 CSI 카메라 파이프라인 연결
    print("[알림] CSI 카메라를 초기화 중입니다...")
    pipeline = gstreamer_pipeline(flip_method=0)
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

    if not cap.isOpened():
        print("="*60)
        print("[에러] CSI 카메라를 열 수 없습니다!")
        print("카메라 케이블 연결 상태나 nvarguscamerasrc 데몬 상태를 확인하세요.")
        print("="*60)
        exit()

    print("[알림] 카메라가 정상적으로 구동 중입니다. 종료하려면 'q'를 누르세요.")

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            print("[알림] 프레임을 읽어오지 못했습니다. 카메라 연결을 확인하세요.")
            break 

        # 영상 크기 정보 추출
        height, width = frame.shape[:2]

        # Grayscale 변환
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Gaussian Blur 
        blur = cv2.GaussianBlur(gray, (5, 5), 0)

        # 트랙바에서 현재 설정된 Canny 임계값 읽어오기
        low_t = cv2.getTrackbarPos('Low_Threshold', window_name)
        high_t = cv2.getTrackbarPos('High_Threshold', window_name)

        # Canny Edge Detection
        edges = cv2.Canny(blur, low_t, high_t)

        # ROI (관심 영역) 설정 (화면 크기에 맞춰 동적으로 계산됨)
        roi_vertices = [
            (0, height), 
            (width / 2 - 50, height / 2 + 50), 
            (width / 2 + 50, height / 2 + 50), 
            (width, height)
        ]
        cropped_edges = region_of_interest(edges, np.array([roi_vertices], np.int32))

        # Hough Transform (직선 성분 추출)
        lines = cv2.HoughLinesP(cropped_edges, rho=1, theta=np.pi/180, threshold=40, 
                                minLineLength=20, maxLineGap=10)

        # 조향 오차 계산 및 시각화
        error, result_image = calculate_steering(frame.copy(), lines)

        # 결과 화면 출력
        cv2.imshow(window_name, result_image)
        cv2.imshow('Canny Edges (ROI)', cropped_edges) 

        # 'q' 누르면 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("[알림] 사용자가 'q'를 눌러 프로그램을 종료했습니다.")
            break

    cap.release()
    cv2.destroyAllWindows()