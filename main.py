import cv2
import time
import threading
import numpy as np

# 모듈 임포트
import lane_detection
import ugv_control

# 기본 설정 및 스레드 간 데이터 공유 변수
BASE_SPEED = 0.3  
shared_error = 0            
shared_speed = 0.0          
error_lock = threading.Lock()
is_running = True
# [추가] 영상 지연/제어 오류가 나면 재실행 전까지 주행 금지
FRAME_TIMEOUT = 0.5  # 초. 실제 영상 처리 시간을 확인해 조정
STARTUP_TIMEOUT = 5.0  # 카메라 개방 후 첫 영상 대기 제한
shared_frame_time = None
control_fault = threading.Event()

# 하드웨어 통신 변수 선언
ugv_serial = None

# ==========================================
# [백그라운드 스레드] 로봇 모터 제어
# ==========================================
def control_thread_task():
    global shared_error, shared_speed, is_running
    
    previous_error = 0
    previous_frame_time = None
    angular_z = 0.0
    started_at = time.monotonic()

    print("[알림] 제어 백그라운드 스레드가 시작되었습니다.")

    try:
        while is_running:
            # 1. 거리 제어는 아직 사용하지 않으므로 수신 버퍼만 제한적으로 비움
            if ugv_serial:
                ugv_control.clear_feedback_buffer(ugv_serial)

            # 2. 최신 영상의 오차/속도/시각을 함께 가져옴
            with error_lock:
                current_error = shared_error
                current_speed = shared_speed
                frame_time = shared_frame_time

            now = time.monotonic()
            if frame_time is None:
                if now - started_at > STARTUP_TIMEOUT:
                    raise RuntimeError("첫 카메라 영상 대기 시간 초과")
                current_speed = 0.0
            elif now - frame_time > FRAME_TIMEOUT:
                raise RuntimeError("카메라 영상 갱신 중단 또는 처리 지연")

            # 3. 기존 PD 식/계수 유지. 새 영상이 들어왔을 때만 계산
            if current_speed <= 0:
                angular_z = 0.0
                previous_frame_time = None
            elif frame_time != previous_frame_time:
                if previous_frame_time is None:
                    # 첫 출발/재출발 때는 이전 오차가 없으므로 D항을 0으로 처리
                    previous_error = current_error
                    frame_dt = 0.02
                else:
                    frame_dt = frame_time - previous_frame_time
                angular_z = ugv_control.calculate_pid(
                    current_error, previous_error, frame_dt)
                previous_error = current_error
                previous_frame_time = frame_time

            # 4. 하체로 주행 명령 하달
            if not is_running:
                break
            if ugv_serial:
                ugv_control.send_driving_command(
                    ugv_serial, current_speed, angular_z if current_speed > 0 else 0.0)
            time.sleep(0.02)

    except Exception as e:
        control_fault.set()
        print(f"[에러] 제어 오류로 정지합니다. 원인 해결 후 재실행하세요: {e}")
    finally:
        with error_lock:
            shared_speed = 0.0
            is_running = False
        # 제어 스레드가 정지 명령/포트 닫기까지 담당하여 동시 쓰기를 방지
        ugv_control.stop_ugv(ugv_serial)


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
        "video/x-raw, format=(string)BGR ! appsink max-buffers=1 drop=true sync=false"
        % (sensor_id, capture_width, capture_height, framerate, flip_method, display_width, display_height)
    )

# ==========================================
# [메인 실행 파트]
# ==========================================
if __name__ == "__main__":
    
    ugv_serial = ugv_control.init_ugv()

    if not ugv_serial:
        print("[종료] UGV02가 연결되지 않아 프로그램을 종료합니다. 포트를 확인하세요.")
        exit()

    print("[알림] IMX219 CSI 카메라를 초기화 중입니다...")
    pipeline = gstreamer_pipeline(flip_method=0)
    try:
        ugv_control.send_driving_command(ugv_serial, 0.0, 0.0)
        cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    except BaseException:
        ugv_control.stop_ugv(ugv_serial)
        raise

    if not cap.isOpened():
        print("[에러] 카메라를 열 수 없습니다! 시스템을 즉시 정지합니다.")
        ugv_control.stop_ugv(ugv_serial) 
        exit()

    control_thread = threading.Thread(target=control_thread_task, daemon=True)
    try:
        control_thread.start()
    except BaseException:
        ugv_control.stop_ugv(ugv_serial)
        cap.release()
        raise

    # SSH 환경 테스트를 위한 GUI 출력 관련 주석 유지
    # window_name = 'Future Makers - UGV02 Autonomous Driving'
    # cv2.namedWindow(window_name)

    print("[알림] 카메라 및 자율주행 시스템이 정상 구동 중입니다. (강제 종료는 Ctrl+C)")

    try:
        while is_running and cap.isOpened():
            # 읽기/영상 처리 지연까지 포함해 영상의 유효 시간을 판단
            frame_time = time.monotonic()
            ret, frame = cap.read()
            if not ret or frame is None:
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
                if not is_running or control_fault.is_set():
                    break
                shared_frame_time = frame_time
                shared_error = error
                if is_detected:
                    shared_speed = BASE_SPEED
                else:
                    shared_speed = 0.0

            # cv2.imshow(window_name, result_image)
            # if cv2.waitKey(1) & 0xFF == ord('q'):
            #     print("[알림] 사용자가 'q'를 눌러 프로그램을 종료했습니다.")
            #     break

    except KeyboardInterrupt:
        print("\n[알림] 강제 종료(Ctrl+C) 신호를 감지했습니다.")
        
    finally:
        print("[알림] 시스템을 안전하게 종료합니다.")
        with error_lock:
            shared_speed = 0.0
            is_running = False
        if control_thread.is_alive():
            control_thread.join(timeout=1.0)
        if control_thread.is_alive():
            print("[경고] 제어 스레드 종료가 지연되고 있습니다.")
        # 포트 정지/닫기는 위 제어 스레드의 finally에서 수행
        cap.release()
        cv2.destroyAllWindows()