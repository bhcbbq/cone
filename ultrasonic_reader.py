import serial
import threading
import time

class UltrasonicReader:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200):
        """
        아두이노 초음파 센서 시리얼 수신 클래스
        * UGV 하체(/dev/ttyACM0)와 아두이노(/dev/ttyUSB0) 포트 분리 확인 필요
        """
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.is_running = False
        
        # [Left, Front, Right] 거리 데이터 (단위: cm, 실패 시 -1.0)
        self.distances = [-1.0, -1.0, -1.0]
        self.lock = threading.Lock()
        self.thread = None

    def start(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            self.is_running = True
            self.thread = threading.Thread(target=self._read_loop, daemon=True)
            self.thread.start()
            print(f"[알림] 아두이노 초음파 센서 포트 연결 완료 ({self.port})")
            return True
        except Exception as e:
            print(f"[에러] 아두이노 연결 실패 ({self.port}): {e}")
            print("팁: 'ls /dev/ttyACM*' 로 포트 번호를 확인하고 아두이노 포트를 지정하세요.")
            return False

    def _read_loop(self):
        time.sleep(2)  # 아두이노 재부팅 대기
        while self.is_running:
            if self.ser and self.ser.in_waiting > 0:
                try:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line.startswith("US,"):
                        parts = line.split(',')
                        if len(parts) >= 5:
                            left = float(parts[2])
                            front = float(parts[3])
                            right = float(parts[4])
                            
                            with self.lock:
                                self.distances = [left, front, right]
                except Exception:
                    pass
            time.sleep(0.01)

    def get_distances(self):
        """최신 [Left, Front, Right] 거리값 반환"""
        with self.lock:
            return list(self.distances)

    def stop(self):
        self.is_running = False
        if self.ser and self.ser.is_open:
            self.ser.close()