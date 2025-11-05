
import sys
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QSlider, QPushButton
)
from PyQt5.QtCore import Qt, QTimer
import pyqtgraph.opengl as gl
from pyqtgraph.opengl import GLViewWidget, GLMeshItem
from core.controller import ArmController
import json, os
from core.kinematics import IK3DOF
from PyQt5.QtWidgets import QLineEdit


BG_GREY = (245, 246, 248)

class MainWindow(QMainWindow):
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robotic Arm — PyQt5")
        self.resize(1024, 720)
        self._rec_buffer = []  # lista de poses
        central = QWidget(); self.setCentralWidget(central)
        root = QHBoxLayout(); root.setContentsMargins(12,12,12,12); root.setSpacing(12)
        central.setLayout(root)
        self.ik = IK3DOF(l1=20.0, l2=14.0)
        # 3D View
        self.view = GLViewWidget()
        self.view.setBackgroundColor(BG_GREY)
        self.view.setCameraPosition(distance=40)
        root.addWidget(self.view, stretch=3)

        self.ctrl = ArmController()
        # --- Envío en tiempo real (anti-flood) ---
        self._send_timer = QTimer(self)
        self._send_timer.setInterval(30)   # 30 ms (~33 FPS)
        self._send_timer.setSingleShot(True)
        self._send_timer.timeout.connect(self.ctrl.send_current_pose)
    
        # Compensaciones de ángulos (offsets)
        self.offsets = {
            "base": 0,
            "shoulder": -90,
            "elbow": 45,
            "forearm": 0,
            "grip": 0
        }


        self._build_arm()

        # Right panel
        side = QVBoxLayout(); side.setSpacing(8)
        wrap = QWidget(); wrap.setLayout(side)
        root.addWidget(wrap, stretch=1)

        def add_slider(text, cb):
            side.addWidget(QLabel(text))
            s = QSlider(Qt.Orientation.Horizontal); s.setRange(0,180); s.valueChanged.connect(cb); side.addWidget(s)
            return s

        self.s_base = add_slider("Base (Z)", self.on_base)
        self.s_sh = add_slider("Hombro (Y)", self.on_shoulder)
        self.s_el = add_slider("Codo (Y)", self.on_elbow)
        self.s_fw = add_slider("Brazo (X)", self.on_forearm)
        self.s_grip = add_slider("Pinza (X)", self.on_grip)
        # --- Posición inicial realista del brazo ---
        self.ctrl.angles_deg = {
            "base": 90,
            "shoulder":0,
            "elbow": 45,
            "forearm": 90,
            "grip": 90
        }

        # Ajustar sliders según los valores iniciales
        self.s_base.setValue(self.ctrl.angles_deg["base"])
        self.s_sh.setValue(self.ctrl.angles_deg["shoulder"])
        self.s_el.setValue(self.ctrl.angles_deg["elbow"])
        self.s_fw.setValue(self.ctrl.angles_deg["forearm"])
        self.s_grip.setValue(self.ctrl.angles_deg["grip"])

        # Renderizar la vista inicial del brazo
        self.update_view()


        row = QHBoxLayout()
        self.btn_connect = QPushButton("Conectar"); self.btn_connect.clicked.connect(self.on_connect); row.addWidget(self.btn_connect)
        self.btn_send = QPushButton("Enviar"); self.btn_send.clicked.connect(self.on_send); row.addWidget(self.btn_send)
        self.btn_anim = QPushButton("Animar"); self.btn_anim.clicked.connect(self.on_anim); row.addWidget(self.btn_anim)
        # --- Nuevo botón de recalibración ---
        self.btn_calib = QPushButton("Recalibrar")
        self.btn_calib.clicked.connect(self.on_calibrate)
        row.addWidget(self.btn_calib)

        self.btn_rec = QPushButton("Grabar pose")
        self.btn_save = QPushButton("Guardar secuencia")
        self.btn_play = QPushButton("Reproducir secuencia")
        self.btn_loop = QPushButton("Reproducir en bucle ♻️")
        self.btn_stop = QPushButton("Detener ⏹️")

        self.btn_loop.clicked.connect(self.on_play_loop)
        self.btn_stop.clicked.connect(self.on_stop_seq)

        self.btn_rec.clicked.connect(self.on_rec)
        self.btn_save.clicked.connect(self.on_save_seq)
        self.btn_play.clicked.connect(self.on_play_seq)
        
        self.in_x = QLineEdit(); self.in_x.setPlaceholderText("X (cm)")
        self.in_y = QLineEdit(); self.in_y.setPlaceholderText("Y (cm)")
        go = QPushButton("Ir a (X,Y)")
        def do_go():
            try:
                x = float(self.in_x.text()); y = float(self.in_y.text())
            except:
                self.lbl_status.setText("Valores inválidos")
                return
            sol = self.ik.solve(x, y, 0.0, elbow_up=True)
            for k,v in sol.items():
                if k in self.ctrl.angles_deg:
                    self.ctrl.set_angle(k, v)
            # actualizar sliders visibles
            self.s_base.setValue(int(self.ctrl.get_angle("base")))
            self.s_sh.setValue(int(self.ctrl.get_angle("shoulder")))
            self.s_el.setValue(int(self.ctrl.get_angle("elbow")))
            self.s_fw.setValue(int(self.ctrl.get_angle("forearm")))
            self.s_grip.setValue(int(self.ctrl.get_angle("grip")))
            self.update_view(); self._send_timer.start()

        side.addWidget(self.in_x); side.addWidget(self.in_y); side.addWidget(go)
        go.clicked.connect(do_go)

                # --- Control de velocidad ---
        side.addWidget(QLabel("Velocidad de animación"))
        self.speed_slider = QSlider(Qt.Orientation.Horizontal)
        self.speed_slider.setRange(1, 100)
        self.speed_slider.setValue(50)  # valor medio
        side.addWidget(self.speed_slider)

        self.speed_label = QLabel("Velocidad: 50%")
        side.addWidget(self.speed_label)

        def on_speed_change(v):
            self.speed_label.setText(f"Velocidad: {v}%")
            # convertir el % en un factor de interpolación
            self._speed_factor = max(0.05, v / 100.0)
        self.speed_slider.valueChanged.connect(on_speed_change)

        # valor inicial
        self._speed_factor = 0.5

        side.addWidget(self.btn_rec)
        side.addWidget(self.btn_save)
        side.addWidget(self.btn_play)
        
        side.addWidget(self.btn_loop)
        side.addWidget(self.btn_stop)


        side.addLayout(row)

        self.lbl_status = QLabel("Modo: Simulación"); side.addWidget(self.lbl_status)
        side.addStretch()

        self.timer = None
        self.update_view()

    def on_play_loop(self):
        """Activa la secuencia en modo loop infinito."""
        if not self._rec_buffer:
            self.lbl_status.setText("No hay poses grabadas para reproducir.")
            return
        self.play_sequence(self._rec_buffer, step=2, interval_ms=15, loop=True)
        self.lbl_status.setText("🔁 Reproducción en bucle iniciada...")

    def on_stop_seq(self):
        """Detiene cualquier animación o secuencia en curso."""
        if hasattr(self, "timer") and self.timer:
            self.timer.stop()
        self._idx = 0
        self.lbl_status.setText("⏹️ Reproducción detenida.")

    def current_pose(self):
        return dict(self.ctrl.angles_deg)

    def on_rec(self):
        self._rec_buffer.append(self.current_pose())
        self.lbl_status.setText(f"Pose grabada ({len(self._rec_buffer)})")

    def on_save_seq(self):
        path = os.path.join(os.getcwd(), "poses.json")
        with open(path, "w", encoding="utf-8") as f:
            json.dump(self._rec_buffer, f, indent=2)
        self.lbl_status.setText(f"Secuencia guardada en {path}")

    def on_play_seq(self):
        if not self._rec_buffer:
            self.lbl_status.setText("No hay poses grabadas")
            return
        self.play_sequence(self._rec_buffer, step=2, interval_ms=25)

    def _cyl(self, length, radius, color=(0.2,0.2,0.2,1.0)):
        md = gl.MeshData.cylinder(rows=10, cols=20, radius=[radius, radius], length=length)
        it = GLMeshItem(meshdata=md, smooth=True, color=color, shader='shaded', drawEdges=False)
        # ❌ estaba así:
        # it.translate(0, 0, length/2)
        # ✅ debe quedar así:
        it.translate(0, 0, 0)
        return it


    def _build_arm(self):
        # Dimensiones reales del brazo físico
        self.len_base = 2.0
        self.len_shoulder = 20.0
        self.len_elbow = 14.0
        self.len_forearm = 7.0
        self.len_tip = 3.0

        # Colores minimalistas y metálicos
        GREY  = (0.75, 0.75, 0.75, 1.0)
        ACC1  = (0.18, 0.40, 0.85, 1.0)
        ACC2  = (0.13, 0.65, 0.52, 1.0)
        ACC3  = (0.80, 0.45, 0.25, 1.0)
        TIPC  = (0.85, 0.85, 0.85, 1.0)

        self.seg_base = self._cyl(self.len_base, 0.5, GREY)
        self.seg_sh   = self._cyl(self.len_shoulder, 0.4, ACC1)
        self.seg_el   = self._cyl(self.len_elbow, 0.35, ACC2)
        self.seg_fw   = self._cyl(self.len_forearm, 0.3, ACC3)
        self.seg_tip  = self._cyl(self.len_tip, 0.2, TIPC)

        for seg in [self.seg_base, self.seg_sh, self.seg_el, self.seg_fw, self.seg_tip]:
            self.view.addItem(seg)

    # slider handlers
    def on_base(self, v): self.smooth_set_angle("base", v)
    def on_shoulder(self, v): self.smooth_set_angle("shoulder", v)
    def on_elbow(self, v): self.smooth_set_angle("elbow", v)
    def on_forearm(self, v): self.smooth_set_angle("forearm", v)
    def on_grip(self, v): self.smooth_set_angle("grip", v)
    def smooth_set_angle(self, name, target, duration=0.3, steps=20):
            """Mueve suavemente un solo servo hacia el valor destino."""
            import math, time

            start = float(self.ctrl.get_angle(name))
            target = float(target)
            for i in range(steps + 1):
                t = i / steps
                factor = 0.5 - 0.5 * math.cos(math.pi * t)  # easing
                val = start + (target - start) * factor
                self.ctrl.set_angle(name, val)
                self.update_view()
                if self.ctrl.is_connected():
                    self.ctrl.send_current_pose()
                time.sleep(duration / steps)

    
    def on_connect(self):
        """Conecta o desconecta el brazo y realiza auto-calibración si aplica."""
        if self.ctrl.is_connected():
            # Desconexión
            self.ctrl.disconnect()
            self.lbl_status.setText("Modo: Simulación")
            self.btn_connect.setText("Conectar")
        else:
            # Conexión y calibración
            ok = self.ctrl.connect()
            if ok:
                self.lbl_status.setText(f"Conectado a {self.ctrl.port} — calibrando...")
                QApplication.processEvents()  # refrescar UI

                # Ejecuta calibración física y sincroniza vista 3D
                self.ctrl.calibrate()
                self.update_view()

                self.lbl_status.setText("✅ Calibración completa — listo para operar")
                self.btn_connect.setText("Desconectar")
            else:
                self.lbl_status.setText("❌ No se pudo abrir COM3 (sigue en Simulación)")

    def on_calibrate(self):
        """Envía el brazo a la posición segura inicial."""
        if self.ctrl.is_connected():
            self.lbl_status.setText("Recalibrando brazo...")
            self.ctrl.calibrate()  # <- Mueve físicamente el brazo
            self.update_view()     # <- Actualiza la vista 3D
            self.lbl_status.setText("Brazo recalibrado ✅")
        else:
            self.lbl_status.setText("❌ No hay conexión activa")

    def on_send(self):
        self.ctrl.send_current_pose()

    def on_anim(self):
        # Secuencia más natural: levantar, extender, volver
        seq = [
            # 1️⃣ Posición inicial relajada
            {"base": 90, "shoulder": 30, "elbow": 120, "forearm": 90, "grip": 90},
            # 2️⃣ Levantar el brazo
            {"base": 90, "shoulder": 60, "elbow": 100, "forearm": 100, "grip": 90},
            # 3️⃣ Extender hacia adelante (como si alcanzara algo)
            {"base": 90, "shoulder": 80, "elbow": 60, "forearm": 100, "grip": 60},
            # 4️⃣ Cerrar pinza (simular agarre)
            {"base": 90, "shoulder": 80, "elbow": 60, "forearm": 100, "grip": 30},
            # 5️⃣ Volver al reposo
            {"base": 90, "shoulder": 30, "elbow": 120, "forearm": 90, "grip": 90},
        ]
        self.play_sequence(seq, step=2, interval_ms=25)

    def play_sequence(self, poses, step=1, interval_ms=15, loop=False):
        """Reproduce una lista de poses con animación fluida."""
        self._poses = poses
        self._idx = 0
        self._interval = interval_ms
        self._loop = loop  # 👈 nuevo flag
        self.lbl_status.setText("▶️ Reproduciendo secuencia...")
        self._start_next_pose()

    def _start_next_pose(self):
        """Inicia la siguiente pose en la secuencia."""
        if self._idx >= len(self._poses):
            if self._loop:
                self._idx = 0  # Reiniciar secuencia
            else:
                self.lbl_status.setText("✅ Secuencia completada")
                return

        self._dest = self._poses[self._idx]
        if hasattr(self, "timer") and self.timer:
            self.timer.stop()
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._animate_step)
        self.timer.start(int(self._interval / self._speed_factor))

    def _animate_step(self):
        """Interpolación suave entre la pose actual y la destino."""
        done = True
        pairs = [
            ("base", self.s_base),
            ("shoulder", self.s_sh),
            ("elbow", self.s_el),
            ("forearm", self.s_fw),
            ("grip", self.s_grip)
        ]

        for name, slider in pairs:
            cur = float(self.ctrl.get_angle(name))
            dst = float(self._dest.get(name, cur))
            delta = dst - cur

            if abs(delta) > 0.3:
                cur += delta * (0.05 + self._speed_factor * 0.15) # suavidad proporcional
                done = False

            # Actualiza servo y slider
            self.ctrl.set_angle(name, cur)
            slider.blockSignals(True)
            slider.setValue(int(cur))
            slider.blockSignals(False)

        # Actualiza vista y manda comando (fluido)
        self.update_view()
        self._send_timer.start()

        if done:
            self.timer.stop()
            self._idx += 1
            QTimer.singleShot(300, self._start_next_pose)

    def update_view(self):
        
        # Aplica offsets para igualar la posición física
        a = {k: v + self.offsets.get(k, 0) for k, v in self.ctrl.angles_deg.items()}
        import numpy as np
        from pyqtgraph import Transform3D

        def transform(rot_axis, angle, translate_z=0):
            t = Transform3D()
            t.translate(0, 0, translate_z)
            t.rotate(angle, *rot_axis)
            return t

        # Base
        self.seg_base.resetTransform()
        self.seg_base.rotate(a["base"], 0, 0, 1)

        # Hombro: parte desde el extremo superior de la base
        t_sh = Transform3D()
        t_sh.translate(0, 0, self.len_base)
        t_sh.rotate(a["base"], 0, 0, 1)
        t_sh.rotate(a["shoulder"], 0, 1, 0)
        self.seg_sh.resetTransform()
        self.seg_sh.setTransform(t_sh)

        # Codo: parte desde extremo del hombro
        t_el = Transform3D(t_sh)
        t_el.translate(0, 0, self.len_shoulder)
        t_el.rotate(a["elbow"], 0, 1, 0)
        self.seg_el.resetTransform()
        self.seg_el.setTransform(t_el)

        # Antebrazo: parte desde extremo del codo
        t_fw = Transform3D(t_el)
        t_fw.translate(0, 0, self.len_elbow)
        t_fw.rotate(a["forearm"], 1, 0, 0)
        self.seg_fw.resetTransform()
        self.seg_fw.setTransform(t_fw)

        # Punta: parte desde extremo del antebrazo
        t_tip = Transform3D(t_fw)
        t_tip.translate(0, 0, self.len_forearm)
        t_tip.rotate(a["grip"], 1, 0, 0)
        self.seg_tip.resetTransform()
        self.seg_tip.setTransform(t_tip)


def run():
    app = QApplication(sys.argv); w = MainWindow(); w.show(); sys.exit(app.exec_())
