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
shared_error = 0            # 카메라가 계산한 오차
shared_speed = 0.0          # [수정] 차선이 보일 때만 BASE_SPEED로 변경, 평소엔 0.0(정지)
error_lock = threading.Lock()
is_running = True

# 하드웨어 통신 시작
ugv_serial = ugv_control.init_ugv()
hc12_serial = hc12_comm.init_hc12()

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

        # 4. 하체로 주행 명령 하달 (차선이 없으면 current_speed가 0.0이 전달됨)
        if ugv_serial:
            # 멈춰있을 때는 불필요하게 조향 모터를 꺾지 않도록 angular_z도 0으로 설정
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
    
    # 1. 시리얼 연결 확인
    if not ugv_serial:
        print("[종료] UGV02가 연결되지 않아 프로그램을 종료합니다. 포트를 확인하세요.")
        exit()

    # 2. [수정] 모터 스레드를 켜기 전에 '카메라'부터 정상 작동하는지 먼저 검증!
    print("[알림] IMX219 CSI 카메라를 초기화 중입니다...")
    pipeline = gstreamer_pipeline(flip_method=0)
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

    if not cap.isOpened():
        print("[에러] 카메라를 열 수 없습니다! 시스템을 즉시 정지합니다.")
        ugv_control.stop_ugv(ugv_serial) # 안전 정지 명령 전송
        exit()

    # 3. [수정] 카메라가 정상적으로 열린 것이 확인된 후에만 제어 스레드 가동
    control_thread = threading.Thread(target=control_thread_task, daemon=True)
    control_thread.start()

    window_name = 'Future Makers - UGV02 Autonomous Driving'
    # cv2.namedWindow(window_name) #ssh주석처리

    print("[알림] 카메라 및 자율주행 시스템이 정상 구동 중입니다. ('q'를 누르면 종료)")

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

            # [수정] lane_detection에서 차선 검출 성공 여부(is_detected)를 함께 받음
            error, result_image, is_detected = lane_detection.calculate_steering(frame.copy(), lines)

            # [수정] 차선을 정상적으로 찾았을 때만 전진, 놓치면 속도 0.0 (즉각 정지)
            with error_lock:
                shared_error = error
                if is_detected:
                    shared_speed = BASE_SPEED
                else:
                    shared_speed = 0.0

            #cv2.imshow(window_name, result_image) #ssh 주석처리

            #if cv2.waitKey(1) & 0xFF == ord('q'): #ssh 주석처리
            #    print("[알림] 사용자가 'q'를 눌러 프로그램을 종료했습니다.") #ssh 주석처리
            #    break  #ssh 주석처리

    except KeyboardInterrupt:
        print("\n[알림] 강제 종료되었습니다.")
        
    finally:
        print("[알림] 시스템을 안전하게 종료합니다.")
        is_running = False
        if 'control_thread' in locals() and control_thread.is_alive():
            control_thread.join(timeout=1.0)
        
        ugv_control.stop_ugv(ugv_serial)
        hc12_comm.close_hc12(hc12_serial)
        cap.release()
        cv2.destroyAllWindows()