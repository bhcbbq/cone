import cv2
import time
import threading
import numpy as np

# 모듈 임포트
import lane_detection
import ugv_control
import hc12_comm

# 기본 설정 및 스레드 간 데이터 공유 변수
BASE_SPEED = 0.3  
shared_error = 0            
shared_speed = 0.0          
error_lock = threading.Lock()
is_running = True

# 하드웨어 통신 변수 선언 (초기화는 아래 메인문에서 실행되도록 수정)
ugv_serial = None
hc12_serial = None

# ==========================================
# [백그라운드 스레드] 로봇 모터 제어 및 HC-12 통신
# ==========================================
def control_thread_task():
    global shared_error, shared_speed, is_running
    
    total_distance = 0.0
    previous_error = 0
    last_time = time.time()
    last_hc12_send_time = time.time()

    print("[알림] 제어 및 통신 백그라운드 스레드가 시작되었습니다.")

    while is_running:
        current_time = time.time()
        dt = current_time - last_time
        if dt <= 0: dt = 0.001
        last_time = current_time

        # 1. 엔코더 거리 누적
        if ugv_serial:
            total_distance += ugv_control.read_odometry(ugv_serial, dt)

        # 2. 최신 오차값 및 주행 속도 가져오기
        with error_lock:
            current_error = shared_error
            current_speed = shared_speed

        # 3. PID 조향 각속도 계산
        angular_z = ugv_control.calculate_pid(current_error, previous_error, dt)
        previous_error = current_error

        # 4. 하체로 주행 명령 하달 
        if ugv_serial:
            send_angular = angular_z if current_speed > 0 else 0.0
            ugv_control.send_driving_command(ugv_serial, current_speed, send_angular)

        # 5. HC-12 거리 데이터 발송 (1초 주기)
        if hc12_serial and (current_time - last_hc12_send_time) >= 1.0:
            hc12_comm.send_distance(hc12_serial, total_distance)
            last_hc12_send_time = current_time

        time.sleep(0.02)

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
        % (sensor_id, capture_width, capture_height, framerate, flip_method, display_width, display_height)
    )

# ==========================================
# [메인 실행 파트]
# ==========================================
if __name__ == "__main__":
    
    # 수정: 모듈 임포트 시 무단 연결 방지를 위해 메인문 내부로 하드웨어 초기화 위치 이동
    ugv_serial = ugv_control.init_ugv()
    hc12_serial = hc12_comm.init_hc12()

    # 1. 시리얼 연결 확인
    if not ugv_serial:
        print("[종료] UGV02가 연결되지 않아 프로그램을 종료합니다. 포트를 확인하세요.")
        exit()

    print("[알림] IMX219 CSI 카메라를 초기화 중입니다...")
    pipeline = gstreamer_pipeline(flip_method=0)
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

    if not cap.isOpened():
        print("[에러] 카메라를 열 수 없습니다! 시스템을 즉시 정지합니다.")
        ugv_control.stop_ugv(ugv_serial) 
        exit()

    control_thread = threading.Thread(target=control_thread_task, daemon=True)
    control_thread.start()

    # 수정: SSH 환경 사용을 위해 GUI 창 생성 코드 주석 처리
    # window_name = 'Future Makers - UGV02 Autonomous Driving'
    # cv2.namedWindow(window_name)

    print("[알림] 카메라 및 자율주행 시스템이 정상 구동 중입니다. (강제 종료는 Ctrl+C)")

    try:
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                print("[경고] 카메라 영상을 받아오지 못했습니다. 기체를 정지합니다.")
                with error_lock:
                    shared_speed = 0.0
                break 

            height, width = frame.shape[:2]
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            blur = cv2.GaussianBlur(gray, (5, 5), 0)
            edges = cv2.Canny(blur, 50, 150)

            roi_vertices = [
                (0, height), 
                (int(width * 0.2), int(height * 0.45)),
                (int(width * 0.8), int(height * 0.45)), 
                (width, height)
            ]
            
            cropped_edges = lane_detection.region_of_interest(edges, np.array([roi_vertices], np.int32))
            lines = cv2.HoughLinesP(cropped_edges, rho=1, theta=np.pi/180, threshold=40, 
                                    minLineLength=20, maxLineGap=10)

            error, result_image, is_detected = lane_detection.calculate_steering(frame.copy(), lines)

            with error_lock:
                shared_error = error
                if is_detected:
                    shared_speed = BASE_SPEED
                else:
                    shared_speed = 0.0

            # 수정: SSH 환경 사용을 위해 GUI 출력 및 키보드 입력 코드 주석 처리
            # cv2.imshow(window_name, result_image)
            # if cv2.waitKey(1) & 0xFF == ord('q'):
            #     print("[알림] 사용자가 'q'를 눌러 프로그램을 종료했습니다.")
            #     break

    except KeyboardInterrupt:
        print("\n[알림] 강제 종료(Ctrl+C) 신호를 감지했습니다.")
        
    finally:
        print("[알림] 시스템을 안전하게 종료합니다.")
        is_running = False
        if 'control_thread' in locals() and control_thread.is_alive():
            control_thread.join(timeout=1.0)
        
        ugv_control.stop_ugv(ugv_serial)
        hc12_comm.close_hc12(hc12_serial)
        cap.release()
        cv2.destroyAllWindows()