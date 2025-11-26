from drone.planner.obstacle import Obstacle
from drone.planner.avoidance import AvoidancePlanner

obs = Obstacle(10, 8, radius=3)
planner = AvoidancePlanner()

nx, ny = 9, 7  # drone gần obstacle

avoid_point = planner.compute_avoid_point(nx, ny, obs)
print("Avoid point:", avoid_point)
