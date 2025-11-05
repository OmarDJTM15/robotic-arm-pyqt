# 🤖 Robotic Arm (3DOF) — PyQt5 + PyOpenGL + ESP32

Minimalista y profesional: simulación 3D (pyqtgraph.opengl), control por sliders,
animación de poses y envío al ESP32 (COM3, 115200) con protocolo `A1/A2/A3`.

## Instalación
```bash
python -m venv .venv
.venv\Scripts\activate   # Windows
pip install -r requirements.txt
python main.py
```

## Uso
- **Conectar** abre COM3 (cámbialo en `core/serial_comm.py` si es necesario).
- **Enviar** manda `A1,A2,A3` con Base/Hombro/Codo actuales.
- **Animar** reproduce una secuencia de ejemplo.

## Estructura
- `app/main_ui.py` — UI PyQt5 + 3D
- `core/controller.py` — lógica de ángulos y envío
- `core/serial_comm.py` — manejo de puerto serie
- `hardware/servo_map.py` — límites por articulación
