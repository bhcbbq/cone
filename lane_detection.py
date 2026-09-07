import cv2
import numpy as np
import math

def region_of_interest(img, vertices):
    """
    [ROI 함수] 영상에서 바닥 차선 영역만 남깁니다.
    """
    mask = np.zeros_like(img)
    cv2.fillPoly(mask, vertices, 255)
    return cv2.bitwise_and(img, mask)

def calculate_steering(frame, lines):
    """
    [오차 계산 함수] 
    차선 분석 후 (error, frame, is_detected) 형태로 반환합니다.
    """
    height, width = frame.shape[:2]
    camera_center = width // 2

    # 1. 허프 변환에서 선 자체가 하나도 안 잡힌 경우 -> 즉시 정지
    if lines is None:
        cv2.putText(frame, "STATUS: No Lines Found! (STOPPED)", (30, 90), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        return 0, frame, False

    left_line_x, left_line_y = [], []
    right_line_x, right_line_y = [], []

    for line in lines:
        x1, y1, x2, y2 = line.flatten()
        if x1 == x2: continue 
        
        slope = (y2 - y1) / (x2 - x1)
        if math.fabs(slope) < 0.5: continue  # 가로선(노이즈) 제거

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
        cv2.line(frame, (left_x_bottom, y_bottom), (left_x_top, y_top), (0, 255, 0), 6)
        
    if len(right_line_x) > 0:
        poly_right = np.polyfit(right_line_y, right_line_x, 1)
        right_x_bottom = int(np.polyval(poly_right, y_bottom))
        right_x_top = int(np.polyval(poly_right, y_top))
        cv2.line(frame, (right_x_bottom, y_bottom), (right_x_top, y_top), (0, 255, 0), 6)

    # 2. 좌/우 유효한 차선이 하나도 계산되지 않은 경우 -> 즉시 정지
    if left_x_bottom is None and right_x_bottom is None:
        cv2.putText(frame, "STATUS: Lane Lost! (STOPPED)", (30, 90), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        return 0, frame, False

    # 3. 양쪽 차선이 모두 있는 경우 (정상 주행)
    if left_x_bottom is not None and right_x_bottom is not None:
        lane_center_bottom = (left_x_bottom + right_x_bottom) // 2
        lane_center_top = (left_x_top + right_x_top) // 2
    # 4. 한쪽 차선만 인식된 경우 (반대편 차선 간격을 가정한 보정 주행)
    elif left_x_bottom is not None:
        lane_center_bottom = left_x_bottom + 160  # 표준 도로 폭의 절반 가정
        lane_center_top = left_x_top + 160
    else:
        lane_center_bottom = right_x_bottom - 160
        lane_center_top = right_x_top - 160

    cv2.line(frame, (lane_center_bottom, y_bottom), (lane_center_top, y_top), (0, 255, 255), 3)

    error = camera_center - lane_center_bottom
    error_margin = 20 

    cv2.circle(frame, (camera_center, y_bottom - 40), 8, (255, 0, 0), -1)      
    cv2.circle(frame, (lane_center_bottom, y_bottom - 40), 8, (0, 255, 0), -1) 
    cv2.line(frame, (camera_center, y_bottom - 40), (lane_center_bottom, y_bottom - 40), (0, 0, 255), 4)

    status_text = "Status: Driving (On Track)" if abs(error) <= error_margin else f"Status: Turning ({'Left' if error > 0 else 'Right'})"
    cv2.putText(frame, f"Error: {error} px", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
    cv2.putText(frame, status_text, (30, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

    # 정상 인식 플래그 True 반환
    return error, frame, True