import cv2
import numpy as np
import math
import serial 
import time
import json
import threading

# ========================================
# 1. 통신 및 기본 설정.dddd
# ========================================
try:
    ugv_serial = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
    print("[알림] UGV02와 정상적으로 연결되었습니다!")
except Exception as e:
    print(f"[에러] UGV02 연결 실패: {e}")
    exit()

try:
    hc12_serial = serial.Serial('/dev/ttyUSB1', 9600, timeout=0.1)
    print("[알림] HC-12 무선 통신 모듈이 준비되었습니다!")
except:
    print("[경고] HC-12 포트를 찾을 수 없습니다. 무선 전송이 생략됩니다.")
    hc12_serial = None

BASE_SPEED = 0.3  # 기본 직진 속도 (m/s)

# ==========================================
# 2. 스레드 공유 변수 
# ==========================================
shared_error = 0             
error_lock = threading.Lock() 
is_running = True            

# ==========================================
# 3. 예전 방식 그대로! 차선 인식 및 조향 시각화 함수
# ==========================================
def calculate_steering(frame, lines):
    height, width = frame.shape[:2]
    camera_center = width // 2  

    if lines is None:
        return 0, frame 

    left_line_x = []
    left_line_y = []
    right_line_x = []
    right_line_y = []

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

# 관심 영역(ROI) 함수
def region_of_interest(img, vertices):
    mask = np.zeros_like(img)
    match_mask_color = 255
    cv2.fillPoly(mask, vertices, match_mask_color)
    return cv2.bitwise_and(img, mask)

# ==========================================
# 4. [백그라운드 스레드] 로봇 제어 및 통신 전담
# ==========================================
def control_thread_task():
    global shared_error, is_running
    
    total_distance = 0.0
    previous_error = 0
    last_time = time.time()
    last_hc12_send_time = time.time()
    
    Kp = 0.005  
    Kd = 0.002  

    print("[알림] 제어 및 통신 백그라운드 스레드가 시작되었습니다.")

    while is_running:
        current_time = time.time()
        dt = current_time - last_time
        if dt <= 0: dt = 0.001
        last_time = current_time

        # UGV02 엔코더(Odometry) 피드백 수신 및 거리 계산
        while ugv_serial.in_waiting > 0:
            try:
                raw_data = ugv_serial.readline().decode('utf-8').strip()
                if not raw_data:
                    continue
                feedback = json.loads(raw_data)
                
                # T:1001 및 L/R 키는 제조사 위키 확인 후 수정 필요
                if feedback.get("T") == 1001: 
                    left_speed = feedback.get("L", 0.0)
                    right_speed = feedback.get("R", 0.0)
                    real_speed = (left_speed + right_speed) / 2.0
                    total_distance += real_speed * dt
            except json.JSONDecodeError:
                pass 

        # PID 계산
        with error_lock:
            current_error = shared_error

        derivative = (current_error - previous_error) / dt
        angular_z = -(Kp * current_error + Kd * derivative)
        previous_error = current_error

        # UGV02 주행 명령 송신
        command_dict = {"T": 13, "X": BASE_SPEED, "Z": round(angular_z, 3)}
        ugv_serial.write((json.dumps(command_dict) + '\n').encode('utf-8'))

        # HC-12 중계기 무선 송신
        if hc12_serial and (current_time - last_hc12_send_time) >= 1.0:
            send_msg = f"DIST: {total_distance:.2f} m\n"
            hc12_serial.write(send_msg.encode('utf-8'))
            last_hc12_send_time = current_time

        time.sleep(0.02) 

# ==========================================
# 5. [메인 스레드] 메인 실행 파트
# ==========================================
if __name__ == "__main__":
    
    # 백그라운드 스레드 시작
    control_thread = threading.Thread(target=control_thread_task, daemon=True)
    control_thread.start()

    window_name = 'Future Makers - UGV02 Autonomous Driving'
    cv2.namedWindow(window_name)

    # 젯슨 나노용 CSI 카메라 또는 일반 USB 카메라 연결
    print("[알림] 카메라를 초기화 중입니다...")
    # 만약 CSI 카메라를 사용하신다면 기존의 gstreamer_pipeline 코드로 복구하셔도 됩니다.
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("[에러] 카메라를 열 수 없습니다!")
        exit()

    print("[알림] 카메라가 정상적으로 구동 중입니다. 종료하려면 'q'를 누르세요.")

    try:
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break 

            height, width = frame.shape[:2]

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            blur = cv2.GaussianBlur(gray, (5, 5), 0)
            
            # Canny 임계값 고정 (트랙바 대신 안정적인 값 사용)
            edges = cv2.Canny(blur, 50, 150)

            roi_vertices = [
                (0, height), 
                (int(width * 0.2), int(height * 0.45)),
                (int(width * 0.8), int(height * 0.45)), 
                (width, height)
            ]
            cropped_edges = region_of_interest(edges, np.array([roi_vertices], np.int32))

            lines = cv2.HoughLinesP(cropped_edges, rho=1, theta=np.pi/180, threshold=40, 
                                    minLineLength=20, maxLineGap=10)

            # 조향 오차 계산 및 시각화 (예전 방식 그대로)
            error, result_image = calculate_steering(frame.copy(), lines)

            # 계산된 오차값을 스레드 공유 변수에 업데이트
            with error_lock:
                shared_error = error

            # 결과 화면 출력
            cv2.imshow(window_name, result_image)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("[알림] 사용자가 'q'를 눌러 프로그램을 종료했습니다.")
                break

    except KeyboardInterrupt:
        print("\n[알림] 강제 종료되었습니다.")
        
    finally:
        print("[알림] 시스템을 안전하게 종료합니다.")
        is_running = False  
        control_thread.join() 
        
        stop_command = '{"T":13, "X":0.0, "Z":0.0}\n'
        ugv_serial.write(stop_command.encode('utf-8'))
        
        ugv_serial.close()
        if hc12_serial:
            hc12_serial.close()
        cap.release()
        cv2.destroyAllWindows()