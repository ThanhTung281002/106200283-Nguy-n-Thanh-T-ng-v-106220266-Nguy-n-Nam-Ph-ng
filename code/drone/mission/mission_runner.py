# drone/mission/mission_runner.py
import asyncio
import csv
import time
from typing import List, Tuple

from drone.planner.detector import ObstacleDetector
from drone.planner.avoidance import AvoidancePlanner
from drone.planner.obstacle import Obstacle
from drone.core.controller import Controller

class MissionRunner:
    """
    Simple mission runner:
    - fly a list of waypoints (x,y,z)
    - if detector detects an obstacle while approaching a waypoint:
        -> compute avoid_point
        -> goto avoid_point
        -> goto the original waypoint
    - logs telemetry (timestamp, north, east, down, event)
    """
    def __init__(self,
                 controller: Controller,
                 detector: ObstacleDetector,
                 avoid_planner: AvoidancePlanner,
                 drone_system,
                 log_path: str = "results/mission_log.csv"):
        self.ctrl = controller
        self.detector = detector
        self.avoid_planner = avoid_planner
        self.drone = drone_system  # mavsdk.System used for telemetry
        self.log_path = log_path

        self._log_rows = []
        self.avoid_count = 0

    async def _get_position(self) -> Tuple[float, float, float]:
        """Read one sample of NED position (north, east, down)."""
        async for pv in self.drone.telemetry.position_velocity_ned():
            return pv.position.north_m, pv.position.east_m, pv.position.down_m

    def _record(self, event: str = ""):
        """Append current time and a snapshot to internal log (non-async: call after awaiting position)."""
        # NOTE: call after awaiting position to get values
        pass  # replaced below in run to avoid nested async in sync method

    async def run(self, waypoints: List[Tuple[float, float, float]]):
        """Execute mission waypoints with simple avoidance."""
        print("[mission] starting mission with {} waypoints".format(len(waypoints)))
        # header for CSV later
        self._log_rows = []
        self.avoid_count = 0

        for idx, (tx, ty, tz) in enumerate(waypoints, start=1):
            print(f"[mission] waypoint {idx}/{len(waypoints)} -> ({tx:.2f},{ty:.2f},{tz:.2f})")

            # Before going, check current position
            cx, cy, cd = await self._get_position()
            # record current pos
            ts = time.time()
            self._log_rows.append([ts, cx, cy, cd, "pre_waypoint_check", idx, tx, ty])

            # if detector says obstacle nearby relative to current pos or line to target intersects obstacle:
            obs = self.detector.detect(cx, cy)
            if obs:
                print("[mission] obstacle detected near current pos -> compute avoid point")
                # compute avoid point
                ax, ay = self.avoid_planner.compute_avoid_point(cx, cy, obs)
                if ax is not None:
                    print(f"[mission] goto avoid point ({ax:.2f},{ay:.2f})")
                    # go to avoid point
                    await self.ctrl.goto(ax, ay, tz, speed=1.5, sleep_dt=0.05)
                    self.avoid_count += 1
                    ts = time.time()
                    nx, ny, nd = await self._get_position()
                    self._log_rows.append([ts, nx, ny, nd, "avoided_at_start", idx, tx, ty])

            # Now go to target waypoint
            print(f"[mission] heading to waypoint ({tx:.2f},{ty:.2f})")
            await self.ctrl.goto(tx, ty, tz, speed=2.0, sleep_dt=0.05)

            # After reaching waypoint, log
            ts = time.time()
            rx, ry, rd = await self._get_position()
            self._log_rows.append([ts, rx, ry, rd, "reached_waypoint", idx, tx, ty])

            # small hover between waypoints
            await asyncio.sleep(0.5)

        print("[mission] mission finished. avoidance_count =", self.avoid_count)

    def save_csv(self, path: str = None):
        out = path if path else self.log_path
        # ensure folder exists (caller creates folder); write CSV
        header = ["timestamp", "north_m", "east_m", "down_m", "event", "wp_index", "wp_x", "wp_y"]
        with open(out, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(header)
            writer.writerows(self._log_rows)
        print(f"[mission] log saved to {out}")
