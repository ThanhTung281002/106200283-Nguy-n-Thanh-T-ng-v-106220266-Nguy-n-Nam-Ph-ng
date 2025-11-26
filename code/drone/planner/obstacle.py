class Obstacle:
    def __init__(self, center_n, center_e, radius: float = 3.5):
        self.cx = center_n
        self.cy = center_e
        self.radius = radius   # safety radius (meters)


