import serial

class SerialComm:
    def __init__(self, port: str = "COM8", baud: int = 115200, timeout: float = 0.2):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self._ser = None

    def is_open(self) -> bool:
        return self._ser is not None and self._ser.is_open

    def open(self) -> bool:
        if self.is_open(): return True
        try:
            self._ser = serial.Serial(self.port, self.baud, timeout=self.timeout); return True
        except Exception as e:
            print(f"[SerialComm] No se pudo abrir {self.port}: {e}"); self._ser = None; return False

    def close(self):
        try:
            if self._ser and self._ser.is_open: self._ser.close()
        except Exception: pass
        self._ser = None

    def send(self, text: str):
        try:
            if not self.is_open():
                if not self.open():
                    print("[SerialComm] Puerto no disponible, no se envía."); return
            self._ser.write(text.encode("utf-8"))
        except Exception as e:
            print(f"[SerialComm] Error enviando: {e}")
