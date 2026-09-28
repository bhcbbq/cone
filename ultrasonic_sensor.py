import serial
import threading
import time
import re

class UltrasonicSensor:
    def __init__(self, port='/dev/ttyACM0', baudrate=9600):
        # 아두이노 코드에 맞춰 baudrate를 9600으로 설정
        self.dist_left = 999.0   # 센서1
        self.dist_center = 999.0 # 센서2
        self.dist_right = 999.0  # 센서3
        
        self.running = True
        self.ser = None
        
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
            print(f"[Ultrasonic] 아두이노 연결 성공: {port}")
        except Exception as e:
            print(f"[Ultrasonic ERROR] 포트 연결 실패: {e}")

    def start(self):
        if self.ser and self.ser.is_open:
            threading.Thread(target=self._read_loop, daemon=True).start()

    def _read_loop(self):
        while self.running:
            if self.ser and self.ser.in_waiting > 0:
                try:
                    # 한글 인코딩 처리를 위해 utf-8 사용
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    
                    if "센서" in line:
                        # "측정 실패" 텍스트를 -1로 임시 변환
                        clean_line = line.replace("측정 실패", "-1")
                        
                        # 정규식을 이용해 문자열 안에서 숫자(소수점 포함) 3개만 추출
                        numbers = re.findall(r"[-+]?(?:\d*\.\d+|\d+)", clean_line)
                        
                        # 센서 번호(1, 2, 3)를 제외한 거리값 3개가 정상 추출되었는지 확인
                        distances = [float(n) for n in numbers if "." in n or n == "-1"]
                        
                        if len(distances) >= 3:
                            # 거리가 -1(측정 실패)이거나 0보다 작으면 안전한 먼 거리(999.0)로 간주
                            self.dist_left = distances[0] if distances[0] > 0 else 999.0
                            self.dist_center = distances[1] if distances[1] > 0 else 999.0
                            self.dist_right = distances[2] if distances[2] > 0 else 999.0

                except Exception as e:
                    pass
            time.sleep(0.01)

    def get_distances(self):
        """(좌측, 중앙, 우측) 거리 반환"""
        return self.dist_left, self.dist_center, self.dist_right

    def stop(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()