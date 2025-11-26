import math
from drone.planner.obstacle import Obstacle

class ObstacleDetector:
    """
    Performs simple distance-based detection.
    """
    def __init__(self, obstacles):
        self.obstacles = obstacles

    def detect(self, drone_x, drone_y):
        """
        Returns:
            None if no obstacle
            Obstacle object if within safety radius
        """
        for obs in self.obstacles:
            d = math.sqrt((drone_x - obs.cx)**2 + (drone_y - obs.cy)**2)
            if d < obs.radius:
                return obs
        return None
