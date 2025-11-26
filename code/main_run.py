# main_run.py
import asyncio
import os
from drone.core.connection import DroneConnection
from drone.core.controller import Controller
from drone.planner.search_pattern import SearchPattern
from drone.planner.obstacle import Obstacle
from drone.planner.detector import ObstacleDetector
from drone.planner.avoidance import AvoidancePlanner
from drone.mission.mission_runner import MissionRunner

async def main():
    # tạo ra file result
    os.makedirs("results", exist_ok=True)

    conn = DroneConnection(uri="udp://:14550", timeout=15.0)
    drone = await conn.connect()

    control = Controller(drone, default_alt=-3.0)


    sp = SearchPattern()
    # khu vực tìm kiếm 
    waypoints = sp.lawnmower(x_min=0, x_max=20, y_min=0, y_max=10, strip_width=5, z=-3)

    # tạo obstacle thủ công 
    obstacles = [
        Obstacle(center_n=5.0, center_e=10, radius=2.0),
        Obstacle(center_n=7.0, center_e=15, radius=2.0)
        
    ]
    detector = ObstacleDetector(obstacles)
    avoider = AvoidancePlanner(safety_margin=3.0)

    mission = MissionRunner(controller=control,
                            detector=detector,
                            avoid_planner=avoider,
                            drone_system=drone,
                            log_path="results/mission_log.csv")

    # Sequence
    await control.arm()
    await control.takeoff(alt=3.0)
    await control.start_offboard(z=-3.0)

    await mission.run(waypoints)

    # return home & land
    await control.goto(0.0, 0.0, -3.0, speed=2.0)
    await control.stop_offboard()
    await control.land()

    # save log
    mission.save_csv()

if __name__ == "__main__":
    asyncio.run(main())
