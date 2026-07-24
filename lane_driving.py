import cv2
import numpy as np
import math
import os

def calculate_steering(frame, lines):
    height, width = frame.shape[:2]
    camera_center = width // 2  # 라바콘(카메라)의 현재 위치 (화면 정중앙 기준점)

    if lines is None:
        return 0, frame 

    left_line_x = []
    left_line_y = []
    right_line_x = []
    right_line_y = []

    # 1. 선분들을 왼쪽/오른쪽 차선으로 분류
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

    # 2. 양쪽 초록색 차선 연장선 그리기 및 좌표 추출
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

    # 3. 차로의 실제 정중앙값(Center) 계산 (바닥과 상단 소실점 부근 각각 계산)
    lane_center_bottom = camera_center
    lane_center_top = camera_center

    if left_x_bottom is not None and right_x_bottom is not None:
        lane_center_bottom = (left_x_bottom + right_x_bottom) // 2

    if left_x_top is not None and right_x_top is not None:
        lane_center_top = (left_x_top + right_x_top) // 2


    # 차로의 정중앙 노선을 나타내는 노란색 수직선 그리기
    cv2.line(frame, (lane_center_bottom, y_bottom), (lane_center_top, y_top), (0, 255, 255), 3)

    # 4. 라바콘 위치와 차로 정중앙의 오차(Error) 계산
    # 로봇 바로 앞 바닥 제어 시점에서의 오차를 구함
    error = camera_center - lane_center_bottom

    # 임의의 허용 오차범위(Threshold) 설정 (예: +-20 픽셀 이내는 정상 주행으로 판단)
    error_margin = 20
    
    # 시각화: 현재 오차 거리를 보여주는 하단 가로 빨간선 및 고정 점들
    cv2.circle(frame, (camera_center, y_bottom - 40), 8, (255, 0, 0), -1)        # 파란 점: 라바콘 위치
    cv2.circle(frame, (lane_center_bottom, y_bottom - 40), 8, (0, 255, 0), -1)  # 초록 점: 실제 차로 중앙
    cv2.line(frame, (camera_center, y_bottom - 40), (lane_center_bottom, y_bottom - 40), (0, 0, 255), 4) # 빨간 선: 오차 크기

    # 5. 오차범위 판별에 따른 상태 메시지 출력
    if abs(error) <= error_margin:
        status_text = "Status: Stable (On Track)"
        status_color = (0, 255, 0) # 안정 상태는 초록색 텍스트
    else:
        direction = "Left" if error > 0 else "Right"
        status_text = f"Status: Alert (Turn {direction})"
        status_color = (0, 0, 255) # 범위를 벗어나면 빨간색 텍스트

    # 화면 좌측 상단 데이터 모니터링 출력
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

# 1. 윈도우 생성 및 실시간 튜닝용 트랙바 부착
window_name = 'Future Makers - Lane Detection'
cv2.namedWindow(window_name)
cv2.createTrackbar('Low_Threshold', window_name, 50, 255, nothing)
cv2.createTrackbar('High_Threshold', window_name, 150, 255, nothing)

# 2. 동영상 불러오기 (터미널 경로 오류 방지를 위해 절대 경로 사용)
# 파일명이나 폴더 이름이 다르다면 아래 빨간 글씨(경로) 부분을 수정해주세요.
video_path = r'C:\Users\noah4\OneDrive\바탕 화면\cone\test_video.mp4'

# 만약 노트북 웹캠으로 바로 띄워보고 싶다면 위 코드는 주석(#) 처리하고 아래 코드를 쓰시면 됩니다.
# cap = cv2.VideoCapture(0)

cap = cv2.VideoCapture(video_path)

# 영상 파일이 정상적으로 열렸는지 확인하는 안전장치
if not cap.isOpened():
    print("="*60)
    print("[에러] 영상을 불러올 수 없습니다!")
    print(f"원인 1: '{video_path}' 경로에 파일이 없습니다.")
    print("원인 2: 동영상 파일 이름이 'test_video.mp4'가 아닐 수 있습니다.")
    print("="*60)
    exit() # 프로그램 강제 종료

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        print("[알림] 영상 재생이 끝났거나 프레임을 읽을 수 없어 종료합니다.")
        break # 영상이 끝나면 루프 종료

    # 영상 크기 정보 추출
    height, width = frame.shape[:2]

    # 3. Grayscale 변환
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # 4. Gaussian Blur 
    blur = cv2.GaussianBlur(gray, (5, 5), 0)

    # 트랙바에서 현재 설정된 Canny 임계값 읽어오기
    low_t = cv2.getTrackbarPos('Low_Threshold', window_name)
    high_t = cv2.getTrackbarPos('High_Threshold', window_name)

    # 5. Canny Edge Detection (이미지의 윤곽선 추출)
    edges = cv2.Canny(blur, low_t, high_t)

    # 6. ROI (관심 영역) 설정
    # 카메라 시점에 따라 차선이 보이는 바닥 부분만 남기기 위한 좌표 설정
    roi_vertices = [
        (0, height), # 좌측 하단
        (width / 2 - 50, height / 2 + 50), # 중앙 좌측 상단 (소실점 근처)
        (width / 2 + 50, height / 2 + 50), # 중앙 우측 상단
        (width, height) # 우측 하단
    ]
    cropped_edges = region_of_interest(edges, np.array([roi_vertices], np.int32))

# 7. Hough Transform (윤곽선 중에서 직선 성분만 찾아내기)
    lines = cv2.HoughLinesP(cropped_edges, rho=1, theta=np.pi/180, threshold=40, 
                            minLineLength=20, maxLineGap=10)

    # 8. 조향 오차 계산 및 시각화
    error, result_image = calculate_steering(frame.copy(), lines)

    # 결과 화면 출력
    cv2.imshow(window_name, result_image)
    cv2.imshow('Canny Edges (ROI)', cropped_edges) 

    # 키보드 'q'를 누르면 창이 닫힘
    if cv2.waitKey(25) & 0xFF == ord('q'):
        print("[알림] 사용자가 'q'를 눌러 프로그램을 종료했습니다.")
        break

cap.release()
cv2.destroyAllWindows()