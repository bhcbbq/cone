        "video/x-raw, format=(string)BGR ! appsink"
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
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

    if not cap.isOpened():
        print("[에러] 카메라를 열 수 없습니다! 시스템을 즉시 정지합니다.")
        ugv_control.stop_ugv(ugv_serial) 
        exit()

    control_thread = threading.Thread(target=control_thread_task, daemon=True)
    control_thread.start()

    # SSH 환경 테스트를 위한 GUI 출력 관련 주석 유지
    window_name = 'Future Makers - UGV02 Autonomous Driving'
    cv2.namedWindow(window_name)

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

            # cv2.imshow(window_name, result_image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                 print("[알림] 사용자가 'q'를 눌러 프로그램을 종료했습니다.")
                 break

    except KeyboardInterrupt:
        print("\n[알림] 강제 종료(Ctrl+C) 신호를 감지했습니다.")
        
    finally:
        print("[알림] 시스템을 안전하게 종료합니다.")
        is_running = False
        if 'control_thread' in locals() and control_thread.is_alive():
            control_thread.join(timeout=1.0)
        
        ugv_control.stop_ugv(ugv_serial)
        cap.release()
        cv2.destroyAllWindows()