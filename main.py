import cv2
import time
import threading
import numpy as np

# 팀원들이 작성한 모듈(파일) 임포트 (레고 블록 조립)
import lane_detection
import ugv_control
import hc12_comm

# 기본 설정 및 스레드 간 '데이터 공유용' 변수 세팅
BASE_SPEED = 0.3  
shared_error = 0            # 카메라 스레드가 구한 오차를 모터 스레드에 전달할 변수
error_lock = threading.Lock() # 두 스레드가 동시에 변수를 건드려 충돌하지 않도록 지켜주는 '자물쇠'
is_running = True            # 프로그램 전체 루프를 끄고 켤 스위치

# 각 하드웨어 포트 통신 시작
ugv_serial = ugv_control.init_ugv()
hc12_serial = hc12_comm.init_hc12()

# ==========================================
# [백그라운드 스레드] 로봇 모터 제어 및 HC-12 통신 전담 (근육 파트)
# ==========================================
def control_thread_task():
    global shared_error, is_running
    
    total_distance = 0.0
    previous_error = 0
    last_time = time.time()
    last_hc12_send_time = time.time()

    print("[알림] 제어 및 통신 백그라운드 스레드가 시작되었습니다.")

    while is_running:
        # 1루프당 걸린 시간(dt) 측정 (약 0.02초 간격)
        current_time = time.time()
        dt = current_time - last_time
        if dt <= 0: dt = 0.001 # 0으로 나누기 에러 방지
        last_time = current_time

        # 1. 엔코더 읽기 (오도메트리 거리 누적)
        if ugv_serial:
            total_distance += ugv_control.read_odometry(ugv_serial, dt)

        # 2. 카메라가 방금 구한 '따끈따끈한 최신 오차값' 가져오기 (자물쇠 열고 읽음)
        with error_lock:
            current_error = shared_error

        # 3. PID 두뇌 모듈을 호출하여 '핸들 꺾는 각속도' 계산
        angular_z = ugv_control.calculate_pid(current_error, previous_error, dt)
        previous_error = current_error # 다음 미분 계산을 위해 현재 오차를 저장

        # 4. 하체(ESP32)로 주행 명령 하달
        if ugv_serial:
            ugv_control.send_driving_command(ugv_serial, BASE_SPEED, angular_z)

        # 5. 핸드폰(무선 모듈)으로 데이터 발송 (1초마다 1번씩 쏘도록 속도 조절)
        if hc12_serial and (current_time - last_hc12_send_time) >= 1.0:
            hc12_comm.send_distance(hc12_serial, total_distance)
            last_hc12_send_time = current_time

        time.sleep(0.02) # 스레드 과부하 방지용 짧은 휴식

# ==========================================
# [메인 실행 파트] 영상 처리 및 시스템 지휘 (시각 파트)
# ==========================================
if __name__ == "__main__":
    
    # 안전 장치: 통신 케이블이 안 꽂혀있으면 아예 시작을 막음
    if not ugv_serial:
        print("[종료] UGV02가 연결되지 않아 프로그램을 종료합니다. 포트를 확인하세요.")
        exit()

    # 제어 스레드(모터/통신)를 백그라운드에 켜고 메인 작업 시작!
    control_thread = threading.Thread(target=control_thread_task, daemon=True)
    control_thread.start()

    window_name = 'Future Makers - UGV02 Autonomous Driving'
    cv2.namedWindow(window_name)

    print("[알림] 카메라를 초기화 중입니다...")
    cap = cv2.VideoCapture(0) # 웹캠 연결

    if not cap.isOpened():
        print("[에러] 카메라를 열 수 없습니다!")
        is_running = False
        exit()

    print("[알림] 카메라가 정상적으로 구동 중입니다. 종료하려면 'q'를 누르세요.")

    try:
        while cap.isOpened():
            # 1. 렌즈에서 사진 1장 찰칵!
            ret, frame = cap.read()
            if not ret: break 

            # 2. 영상 기초 화장 (차선을 잘 찾도록 대비를 올림)
            height, width = frame.shape[:2]
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # 흑백 변환
            blur = cv2.GaussianBlur(gray, (5, 5), 0)       # 노이즈 제거 (블러)
            edges = cv2.Canny(blur, 50, 150)               # 윤곽선(엣지) 추출

            # 바닥 구역(사다리꼴) 좌표 설정
            roi_vertices = [
                (0, height), 
                (int(width * 0.2), int(height * 0.45)),
                (int(width * 0.8), int(height * 0.45)), 
                (width, height)
            ]
            
            # 3. 영상 모듈(lane_detection) 호출
            # 바닥만 잘라내기 -> 허프 변환으로 선 찾기
            cropped_edges = lane_detection.region_of_interest(edges, np.array([roi_vertices], np.int32))
            lines = cv2.HoughLinesP(cropped_edges, rho=1, theta=np.pi/180, threshold=40, 
                                    minLineLength=20, maxLineGap=10)

            # 조향 오차 계산 및 예쁜 시각화 그림 받아오기
            error, result_image = lane_detection.calculate_steering(frame.copy(), lines)

            # 4. 방금 구한 오차를 백그라운드 스레드(모터)가 가져가도록 업데이트
            with error_lock:
                shared_error = error

            # 5. 완성된 화면 모니터에 출력
            cv2.imshow(window_name, result_image)

            # 사용자 키보드 입력 대기 ('q' 누르면 종료)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("[알림] 사용자가 'q'를 눌러 프로그램을 종료했습니다.")
                break

    except KeyboardInterrupt:
        # 터미널 창에서 Ctrl + C 를 눌러서 강제 종료했을 때 에러 처리
        print("\n[알림] 강제 종료되었습니다.")
        
    finally:
        # [우아한 종료] 로봇이 벽에 부딪히지 않게 뒷정리를 하는 아주 중요한 구간!
        print("[알림] 시스템을 안전하게 종료합니다.")
        is_running = False          # 1. 무한 루프 스위치 끄기
        control_thread.join()       # 2. 백그라운드 스레드가 완전히 끝날 때까지 잠시 대기
        
        ugv_control.stop_ugv(ugv_serial) # 3. 모터 속도 0으로 강제 정지 명령 전송!
        hc12_comm.close_hc12(hc12_serial)# 4. 통신 포트 닫기
        cap.release()                    # 5. 카메라 렌즈 닫기
        cv2.destroyAllWindows()          # 6. 화면 창 닫기