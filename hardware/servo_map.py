class ServoMap:
    def __init__(self):
        self.limits = {
            "base": (0, 180),
            "shoulder": (0, 180),
            "elbow": (20, 160),
            "forearm": (0, 180),
            "grip": (30, 150),
        }
