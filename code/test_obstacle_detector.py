from drone.planner.obstacle import Obstacle
from drone.planner.detector import ObstacleDetector

obstacles = [
    Obstacle(center_n=10, center_e=8, radius=3),
    Obstacle(center_n=22, center_e=5, radius=3),
]


detector = ObstacleDetector(obstacles)

test_points = [
    (0,0),
    (5,7),
    (10,7),
    (10,9),
    (13,8)
]

for p in test_points:
    hit = detector.detect(p[0], p[1])
    print(p, "->", "HIT" if hit else "safe")
