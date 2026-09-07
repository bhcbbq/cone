import cv2

def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1280,
    capture_height=720,
    framerate=30,
    display_width=1280,
    display_height=720,
):
    return (
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv ! "
        "video/x-raw, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
        )
    )

def show_camera():
    pipeline = gstreamer_pipeline(sensor_id=0)
    print("Using pipeline:", pipeline)
    
    # GStreamer 백엔드를 명시하여 VideoCapture 생성
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    
    if not cap.isOpened():
        print("카메라를 열 수 없습니다.")
        return

    print("카메라가 열렸습니다. 종료하려면 'q'를 누르세요.")
    while cv2.isWaitKey(1) < 0:
        ret, frame = cap.read()
        if not ret:
            print("프레임을 읽지 못했습니다.")
            break
            
        cv2.imshow("CSI Camera", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    show_camera()