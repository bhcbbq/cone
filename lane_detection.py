import cv2
import numpy as np
import math

def region_of_interest(img, vertices):
    """
    [ROI 함수] 영상에서 불필요한 배경(하늘 등)을 지우고 바닥 차선만 남깁니다.
    """
    # 원본 이미지와 똑같은 크기의 까만색 도화지(mask)를 만듦
    mask = np.zeros_like(img)
    match_mask_color = 255
    
    # 우리가 지정한 사다리꼴 구역(vertices)만 하얗게 칠함
    cv2.fillPoly(mask, vertices, match_mask_color)
    
    # 원본 이미지와 마스크를 겹쳐서, 사다리꼴 안쪽 데이터만 살리고 나머지는 잘라냄(AND 연산)
    return cv2.bitwise_and(img, mask)

def calculate_steering(frame, lines):
    """
    [오차 계산 및 시각화 함수] 차선을 분석해 오차(px)를 구하고 화면에 그림을 그립니다.
    """
    height, width = frame.shape[:2]
    camera_center = width // 2  # 카메라 렌즈의 정중앙 x좌표 (우리의 기준점)

    if lines is None:
        return 0, frame # 인식된 선이 아예 없으면 직진(오차 0) 유지

    # 1. 감지된 선들을 왼쪽 차선과 오른쪽 차선으로 분류할 리스트
    left_line_x, left_line_y = [], []
    right_line_x, right_line_y = [], []

    # 2. 허프 변환이 찾아낸 선(lines)들을 하나씩 까보면서 분류 작업
    for line in lines:
        x1, y1, x2, y2 = line.flatten()
        if x1 == x2: continue # 분모가 0이 되어 에러나는 수직선(무한대 기울기) 방지
        
        slope = (y2 - y1) / (x2 - x1) # 기울기 계산
        if math.fabs(slope) < 0.5: continue # 너무 누워있는 가로선(노이즈) 버리기

        if slope < 0: 
            # 기울기가 음수면 왼쪽 차선 (카메라 좌표계는 y축이 아래로 갈수록 커지기 때문)
            left_line_x.extend([x1, x2])
            left_line_y.extend([y1, y2])
        else:        
            # 기울기가 양수면 오른쪽 차선
            right_line_x.extend([x1, x2])
            right_line_y.extend([y1, y2])

    y_bottom = height # 화면 맨 아래 (y좌표 최대)
    y_top = height // 2 + 50 # 화면 중간쯤 (차선이 소실되는 점)

    left_x_bottom, right_x_bottom = None, None
    left_x_top, right_x_top = None, None

    # 3. 점들을 모아 하나의 긴 직선으로 만들기 (1차 함수 추정)
    if len(left_line_x) > 0:
        # np.polyfit: 주어진 점들을 가장 잘 잇는 1차 방정식(선)을 만들어냄
        poly_left = np.polyfit(left_line_y, left_line_x, 1) 
        left_x_bottom = int(np.polyval(poly_left, y_bottom))
        left_x_top = int(np.polyval(poly_left, y_top))
        # 화면에 두꺼운 초록색 선 긋기
        cv2.line(frame, (left_x_bottom, y_bottom), (left_x_top, y_top), (0, 255, 0), 8)
        
    if len(right_line_x) > 0:
        poly_right = np.polyfit(right_line_y, right_line_x, 1)
        right_x_bottom = int(np.polyval(poly_right, y_bottom))
        right_x_top = int(np.polyval(poly_right, y_top))
        cv2.line(frame, (right_x_bottom, y_bottom), (right_x_top, y_top), (0, 255, 0), 8)

    # 4. 양쪽 차선의 딱 '중간 지점(주행 목표점)' 찾기
    lane_center_bottom = camera_center
    lane_center_top = camera_center

    if left_x_bottom is not None and right_x_bottom is not None:
        lane_center_bottom = (left_x_bottom + right_x_bottom) // 2

    if left_x_top is not None and right_x_top is not None:
        lane_center_top = (left_x_top + right_x_top) // 2

    # 주행해야 할 중앙 궤적을 노란색 얇은 선으로 표시
    cv2.line(frame, (lane_center_bottom, y_bottom), (lane_center_top, y_top), (0, 255, 255), 3)

    # 5. 최종 오차(Error) 계산 = 내 카메라 중앙 위치 - 가야 할 차선 중앙 위치
    error = camera_center - lane_center_bottom
    error_margin = 20 # 20픽셀 이내면 아주 잘 가고 있다고 판단
    
    # 6. 화면 예쁘게 꾸미기 (UI 시각화)
    cv2.circle(frame, (camera_center, y_bottom - 40), 8, (255, 0, 0), -1)      # 내 위치 (파란원)  
    cv2.circle(frame, (lane_center_bottom, y_bottom - 40), 8, (0, 255, 0), -1) # 목표 위치 (초록원)
    cv2.line(frame, (camera_center, y_bottom - 40), (lane_center_bottom, y_bottom - 40), (0, 0, 255), 4) # 오차 거리(빨간선)

    # 오차 범위 안이면 초록색 '안정', 벗어나면 빨간색 '경고' 텍스트 설정
    if abs(error) <= error_margin:
        status_text = "Status: Stable (On Track)"
        status_color = (0, 255, 0)
    else:
        direction = "Left" if error > 0 else "Right"
        status_text = f"Status: Alert (Turn {direction})"
        status_color = (0, 0, 255)

    # 화면 좌측 상단에 텍스트 인쇄
    cv2.putText(frame, f"Error: {error} px (Margin: +/-{error_margin}px)", (30, 50), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
    cv2.putText(frame, status_text, (30, 90), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2)

    return error, frame # 계산된 오차값과, 그림이 다 그려진 이미지를 반환!