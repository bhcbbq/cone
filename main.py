import cv2
import time
import threading
import numpy as np

# 팀원들이 작성한 모듈(파일) 임포트
import lane_detection
import ugv_control
import hc12_comm

# 기본 설정 및 스레드 공유 변수
BASE_SPEED = 0.3  
shared_error = 0            
error_lock = threading.Lock() 
is_running = True            

# 하드웨어 포트 초기화
ugv_serial = ugv_control.init_ugv()
hc12_serial = hc12_comm.init_hc12()

# [백그라운드 스레드] 로봇 모터 제어 및 HC-12 통신 전담
def control_thread_task():
    global shared_error, is_running
    
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

        # 1. 이동 거리 누적 (Odometry)
        if ugv_serial:
            total_distance += ugv_control.read_odometry(ugv_serial, dt)

        # 2. 메인 스레드에서 시각 파트가 구한 오차값(Error) 가져오기
        with error_lock:
            current_error = shared_error

        # 3. PID 모듈을 호출하여 조향 각속도 계산
        angular_z = ugv_control.calculate_pid(current_error, previous_error, dt)
        previous_error = current_error 

        # 4. 하체로 주행 명령 하달
        if ugv_serial:
            ugv_control.send_driving_command(ugv_serial, BASE_SPEED, angular_z)

        # 5. 1초 간격으로 무선 통신(어플) 데이터 발송
        if hc12_serial and (current_time - last_hc12_send_time) >= 1.0:
            hc12_comm.send_distance(hc12_serial, total_distance)
            last_hc12_send_time = current_time

        time.sleep(0.02) 

# ==========================================
# [메인 실행 파트] 시각 정보 처리 관제탑
# ==========================================
if __name__ == "__main__":
    
    # 안전 장치: 하체 연결 없이는 주행 불가능하도록 처리
    if not ugv_serial:
        print("[종료] UGV02가 연결되지 않아 프로그램을 종료합니다. 포트를 확인하세요.")
        exit()

    # 제어 스레드 시작
    control_thread = threading.Thread(target=control_thread_task, daemon=True)
    control_thread.start()

    window_name = 'Future Makers - UGV02 Autonomous Driving'
    cv2.namedWindow(window_name)

    print("[알림] 카메라를 초기화 중입니다...")
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("[에러] 카메라를 열 수 없습니다!")
        is_running = False
        exit()

    print("[알림] 카메라가 정상적으로 구동 중입니다. 종료하려면 'q'를 누르세요.")

    try:
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret: break 

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
            
            # 차선 인식 모듈 호출
            cropped_edges = lane_detection.region_of_interest(edges, np.array([roi_vertices], np.int32))
            lines = cv2.HoughLinesP(cropped_edges, rho=1, theta=np.pi/180, threshold=40, 
                                    minLineLength=20, maxLineGap=10)

            # 조향 오차값 연산 및 시각화 이미지 수신
            error, result_image = lane_detection.calculate_steering(frame.copy(), lines)

            # 스레드가 읽어갈 수 있도록 오차값 금고(Lock)에 업데이트
            with error_lock:
                shared_error = error

            # 화면 출력
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
        
        # 하드웨어 안전 종료 로직 호출
        ugv_control.stop_ugv(ugv_serial)
        hc12_comm.close_hc12(hc12_serial)
        cap.release()
        cv2.destroyAllWindows()