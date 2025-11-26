import math
from drone.planner.obstacle import Obstacle

class AvoidancePlanner:
    def __init__(self, safety_margin=3.0):
        self.safety_margin = safety_margin

    def compute_avoid_point(self, nx, ny, obstacle: Obstacle):
        """
        Compute a temporary waypoint to avoid the obstacle.
        """
        cx, cy = obstacle.cx, obstacle.cy
        r = obstacle.radius

        # === vector from obstacle to drone ===
        vx = nx - cx
        vy = ny - cy
        d = math.sqrt(vx*vx + vy*vy)

        if d < 0.0001:
            return None  # overlap, impossible case

        # === normalized ===
        ux = vx / d
        uy = vy / d

        # === perpendicular for "left" turn ===
        px = -uy
        py = ux

        avoid_distance = r + self.safety_margin

        # project avoid point
        ax = cx + px * avoid_distance
        ay = cy + py * avoid_distance

        return (ax, ay)
