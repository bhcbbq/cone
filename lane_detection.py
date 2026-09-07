import cv2
import numpy as np
import math

# 수정: 차선을 놓쳤을 때 이전 조향값을 기억하기 위한 전역 변수
last_known_error = 0

def region_of_interest(img, vertices):
    """
    [ROI 함수] 영상에서 불필요한 배경을 지우고 바닥 차선만 남깁니다.
    """
    mask = np.zeros_like(img)
    match_mask_color = 255
    cv2.fillPoly(mask, vertices, match_mask_color)
    return cv2.bitwise_and(img, mask)

def calculate_steering(frame, lines):
    """
    [오차 계산 및 시각화 함수] 차선을 분석해 오차(px)를 구하고 화면에 그림을 그립니다.
    """
    global last_known_error
    
    height, width = frame.shape[:2]
    camera_center = width // 2

    if lines is None:
        # 수정: 선이 안 보이면 직진(0)이 아니라, 직전 오차값을 유지하여 커브를 마저 돌게 함
        cv2.putText(frame, "WARNING: Lane Lost! Maintaining Turn...", (30, 130), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        return last_known_error, frame 

    left_line_x, left_line_y = [], []
    right_line_x, right_line_y = [], []

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

    left_x_bottom, right_x_bottom = None, None
    left_x_top, right_x_top = None, None

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

    lane_center_bottom = camera_center
    lane_center_top = camera_center

    if left_x_bottom is not None and right_x_bottom is not None:
        lane_center_bottom = (left_x_bottom + right_x_bottom) // 2
    if left_x_top is not None and right_x_top is not None:
        lane_center_top = (left_x_top + right_x_top) // 2

    cv2.line(frame, (lane_center_bottom, y_bottom), (lane_center_top, y_top), (0, 255, 255), 3)

    error = camera_center - lane_center_bottom
    
    # 수정: 차선을 정상적으로 찾았을 때의 오차값을 다음 프레임을 위해 백업
    last_known_error = error 
    
    error_margin = 20 
    
    cv2.circle(frame, (camera_center, y_bottom - 40), 8, (255, 0, 0), -1)      
    cv2.circle(frame, (lane_center_bottom, y_bottom - 40), 8, (0, 255, 0), -1) 
    cv2.line(frame, (camera_center, y_bottom - 40), (lane_center_bottom, y_bottom - 40), (0, 0, 255), 4)

    if abs(error) <= error_margin:
        status_text = "Status: Stable (On Track)"
        status_color = (0, 255, 0)
    else:
        direction = "Left" if error > 0 else "Right"
        status_text = f"Status: Alert (Turn {direction})"
        status_color = (0, 0, 255)

    cv2.putText(frame, f"Error: {error} px (Margin: +/-{error_margin}px)", (30, 50), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
    cv2.putText(frame, status_text, (30, 90), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2)

    return error, frame