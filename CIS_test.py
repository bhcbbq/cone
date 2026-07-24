import cv2

def gstreamer_pipeline(
    capture_width=1280,
    capture_height=720,
    display_width=640,
    display_height=480,
    framerate=30,
    flip_method=0,
):
    return (
        "nvarguscamerasrc !"
        "video/x-raw(memory:NVMM), "
        f"width=(int){capture_width}, height=(int){capture_height}, "
        f"format=(string)NV12, framerate=(fraction){framerate}/1 !"
        f"nvvidconv flip-method={flip_method} !"
        f"video/x-raw, width=(int){display_width}, height=(int){display_height}, format=(string)BGRx !"
        "videoconvert !"
        "video/x-raw, format=(string)BGR !"
        "appsink"
    )

def show_csi_camera():
    # GStreamer 파이프라인을 사용해 CSI 카메라 객체 생성
    pipeline = gstreamer_pipeline(flip_method=0)
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    
    if not cap.isOpened():
        print("에러: CSI 카메라를 열 수 없습니다. 케이블 연결 상태나 nvarguscamerasrc 상태를 확인하세요.")
        return

    print("CSI 카메라가 정상적으로 구동 중입니다. 종료하려면 키보드의 'q'를 누르세요.")

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            print("프레임을 읽어오지 못했습니다.")
            break

        # 화면에 영상 출력
        cv2.imshow("CSI Camera Test", frame)

        # 'q'를 누르면 반복문 탈출
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    show_csi_camera()