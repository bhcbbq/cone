import time
from ultrasonic_sensor import UltrasonicSensor

def main():
    print("=========================================")
    print(" 🚀 아두이노 초음파 센서 통신 테스트 시작")
    print("=========================================")
    
    # 1. 초음파 센서 객체 생성 및 통신 시작
    # 포트 이름이 다를 경우 '/dev/ttyUSB0' 등으로 수정하세요.
    ultrasonic = UltrasonicSensor(port='/dev/ttyACM0', baudrate=9600)
    ultrasonic.start()
    
    # 아두이노가 재부팅되고 통신이 안정화될 때까지 잠시 대기
    time.sleep(2.0)
    
    print("\n[알림] 데이터 수신 중... (종료하려면 Ctrl+C를 누르세요)\n")

    try:
        while True:
            # 2. 실시간 거리 데이터 가져오기
            dist_L, dist_C, dist_R = ultrasonic.get_distances()
            
            # 3. 터미널에 깔끔하게 정렬하여 출력
            print(f"좌측(45도): {dist_L:>6.1f} cm  |  정면(센터): {dist_C:>6.1f} cm  |  우측(45도): {dist_R:>6.1f} cm")
            
            # 0.1초마다 화면 갱신 (너무 빠르면 보기 힘드므로 조절)
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n[알림] 사용자에 의해 테스트가 종료되었습니다.")
        
    finally:
        # 4. 안전하게 포트 닫기
        ultrasonic.stop()

if __name__ == "__main__":
    main()