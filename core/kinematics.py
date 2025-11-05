import math

class IK3DOF:
    def __init__(self, l1=20.0, l2=14.0):
        self.l1 = l1
        self.l2 = l2

    def solve(self, x, y, z, elbow_up=True):
        # base (yaw) por atan2 en el plano XY
        base = math.degrees(math.atan2(y, x)) % 360

        # radio en planta y altura (usamos Z como "altura" si lo necesitas)
        r = math.hypot(x, y)

        # 2R planar IK (shoulder, elbow) para alcanzar r
        D = (r**2 - self.l1**2 - self.l2**2) / (2*self.l1*self.l2)
        D = max(-1.0, min(1.0, D))  # clamp numérico

        if elbow_up:
            elbow = math.degrees(math.atan2(-math.sqrt(1 - D*D), D))
        else:
            elbow = math.degrees(math.atan2(+math.sqrt(1 - D*D), D))

        phi = math.degrees(math.atan2(y, x))  # orientación radial en XY
        # hombro relativo al radio; aquí modelamos como plano (sin Z real)
        # si quisieras incluir Z, necesitaríamos 3D exacto (más largo)
        shoulder = math.degrees(math.atan2(0, r))  # 0 en este setup plano
        # mejor: usamos ley de cosenos para el ángulo en el "hombro":
        shoulder = math.degrees(
            math.atan2(0, r)  # 0 si no consideras Z; puedes adaptar aquí
        ) + math.degrees(math.atan2(self.l2*math.sin(math.radians(elbow)),
                                    self.l1 + self.l2*math.cos(math.radians(elbow))))

        # forearm lo dejamos neutro; grip neutro
        forearm = 90
        grip = 90

        # Normaliza base a [0..180] si usas ese rango (ajusta con offsets en UI)
        return {
            "base": base % 360,
            "shoulder": shoulder,
            "elbow": 180 - abs(elbow),  # adecúa tu convención
            "forearm": forearm,
            "grip": grip
        }
