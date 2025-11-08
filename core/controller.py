import serial
import time
import math

class ArmController:
    def __init__(self):
        self.port = "COM8"
        self._connected = False
        self.serial = None

        # Ángulos actuales del brazo (°)
        self.angles_deg = {
            "base": 90,
            "shoulder": 0,
            "elbow": 90,
            "forearm": 90,
            "grip": 90
        }

        # Límites seguros (°)
        self.limits = {
            "base": (0, 180),
            "shoulder": (0, 180),
            "elbow": (0, 180),
            "forearm": (0, 180),
            "grip": (30, 150)
        }

        # Pose inicial segura
        self.calibration_pose = {
            "base": 90,
            "shoulder": 30,
            "elbow": 110,
            "forearm": 90,
            "grip": 90
        }

    # =====================================================
    # 🔌 Conexión
    # =====================================================
    def connect(self):
        try:
            self.serial = serial.Serial(self.port, 115200, timeout=1)
            self._connected = True
            print(f"✅ Conectado a {self.port}")
            print("⚙️ Ejecutando auto-calibración suave...")
            self.smooth_move_to(self.calibration_pose, duration=2.5)
            print("✅ Calibración completa.")
            return True
        except Exception as e:
            print(f"❌ Error al conectar con {self.port}: {e}")
            self._connected = False
            return False

    def disconnect(self):
        if self._connected:
            try:
                self.serial.close()
                print("🔌 Desconectado.")
            except:
                pass
        self._connected = False

    def is_connected(self):
        return self._connected

    # =====================================================
    # 🧠 Interpolación suave
    # =====================================================
    def smooth_move_to(self, target_pose, duration=2.0, steps=50):
        """
        Mueve suavemente el brazo de su pose actual a la nueva pose.
        - target_pose: dict con los nuevos ángulos
        - duration: tiempo total (segundos)
        - steps: cantidad de interpolaciones
        """
        current = {k: self.angles_deg[k] for k in target_pose.keys()}
        for i in range(steps + 1):
            t = i / steps
            # Curva tipo ease-in-out (más natural)
            factor = 0.5 - 0.5 * math.cos(math.pi * t)
            new_pose = {}
            for k in target_pose.keys():
                start = current[k]
                end = target_pose[k]
                val = start + (end - start) * factor
                self.set_angle(k, val)
            self.send_current_pose()
            time.sleep(duration / steps)

    # =====================================================
    # ⚙️ Calibración directa
    # =====================================================
    def calibrate(self):
        """Recalibra con interpolación suave."""
        print("🦾 Moviendo brazo a posición segura...")
        self.smooth_move_to(self.calibration_pose, duration=2.5)
        print("✅ Brazo calibrado.")

    # =====================================================
    # 🧩 Control individual
    # =====================================================
    def set_angle(self, name, value):
        """Actualiza un ángulo respetando sus límites físicos."""
        if name not in self.angles_deg:
            return
        lo, hi = self.limits.get(name, (0, 180))
        safe_val = max(lo, min(hi, value))
        if safe_val != value:
            print(f"⚠️ {name} limitado de {value:.1f}° a {safe_val:.1f}°")
        self.angles_deg[name] = safe_val

    def get_angle(self, name):
        return self.angles_deg.get(name, 0)

    # =====================================================
    # 🚀 Envío de datos
    # =====================================================
    def send_current_pose(self):
        """Envía la pose actual al ESP32 o la simula."""
        msg = "B,{},{},{},{},{}\n".format(
            int(self.angles_deg["base"]),
            int(self.angles_deg["shoulder"]),
            int(self.angles_deg["elbow"]),
            int(self.angles_deg["forearm"]),
            int(self.angles_deg["grip"])
        )
        if self._connected:
            try:
                self.serial.write(msg.encode("utf-8"))
                time.sleep(0.02)
            except Exception as e:
                print(f"❌ Error enviando datos: {e}")
        else:
            print(f"(simulación) {msg.strip()}")
