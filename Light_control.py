cd ~/cone
cat > light_control.py << 'EOF'
import threading


class LightControl:
    CMD_ON = b"1"
    CMD_OFF = b"0"

    def __init__(self, send, lock=None):
        self._send = send
        self._lock = lock or threading.Lock()
        self.is_on = False

    @classmethod
    def from_serial(cls, ser, lock=None):
        return cls(ser.write, lock)

    def _write(self, data: bytes):
        with self._lock:
            self._send(data)

    def on(self):
        self._write(self.CMD_ON)
        self.is_on = True

    def off(self):
        self._write(self.CMD_OFF)
        self.is_on = False

    def toggle(self):
        self.off() if self.is_on else self.on()
EOF
ls light_control.py
python3 main_test.py
