import asyncio
from mavsdk.offboard import PositionNedYaw, OffboardError
from mavsdk import System
import time
from typing import Tuple


class Controller:
    """
    điều khiển ở mức cao sau khi đã có một connected system
    điều khiển: arm, takeoff, land, start_offboard, stop_offboard, set_position, goto.
    """

    def __init__(self, drone: System, default_alt: float = -3.0):
        self.drone = drone
        self.default_alt = default_alt
        self._offboard_started = False

    
    # ----------- các điều khiển cơ bản --------------
    async def arm(self):
        print("[control] arming...")
        await self.drone.action.arm()
        # delay nhẹ để commander xử lí 
        await asyncio.sleep(1)
        print("[control] armed")


    async def disarm(self):
        print("[control] disarming...")
        await self.drone.action.disarm()
        await asyncio.sleep(1.0)
        print("[control] disarmed")

    async def takeoff(self, alt: float = 3.0):
        print(f"[control] takeoff to {alt}m")
        await self.drone.action.set_takeoff_altitude(alt)
        await asyncio.sleep(0.5)
        await self.drone.action.takeoff()
        # chờ đến khi cất cánh thành công, trong lúc đó check telemetry (các thông số drone)
        await asyncio.sleep(5.0)
        print("[control] takeoff done")

    async def land(self):
        print("[control] landing...")
        await self.drone.action.land()
        # chờ hạ cánh thành công
        await asyncio.sleep(10.0)
        print("[control] landed")

    
    # ------------- điều khiển offboard ---------
    async def start_offboard(self, z: float = None):
        target_z = z if z is not None else self.default_alt
        print(f"[control] starting offboard at z={target_z}")
        # gửi setpoint khởi tạo (yêu cầu của px4)
        await self.drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, target_z, 0.0))
        try:
            await self.drone.offboard.start()
            self._offboard_started = True
            await asyncio.sleep(0.1)
            print("[control] offboard started")
        except OffboardError as e:
            print(f"[control] failed to start offboard: {e}; trying to abort and disarm")
            # safe fallback
            await self.drone.action.disarm()
            raise   



    async def stop_offboard(self):
        if self._offboard_started:
            print("[control] stopping offboard")
            try:
                await self.drone.offboard.stop()
            except OffboardError as e:
                print(f"[control] warning stopping offboard: {e}")

            self._offboard_started = False
            await asyncio.sleep(0.1)


    async def set_position(self, x: float, y: float, z: float = None, yaw: float = 0.0):
        """Set immediate position setpoint in NED coordinates (meters)."""
        target_z = z if z is not None else self.default_alt
        await self.drone.offboard.set_position_ned(PositionNedYaw(x, y, target_z, yaw))
    

    # ---------- goto: nội suy tuyến tính (đi từ từ) from current pos to (x,y) ----------
    async def _get_current_position(self) -> Tuple[float, float]:
        """
        Read current NED local position (north, east). This uses telemetry stream once.
        Returns (north_m, east_m).
        """
        # lấy mẫu từ telemetry
        async for posvel in self.drone.telemetry.position_velocity_ned():
            # posvel.position has fields north_m, east_m, down_m
            return posvel.position.north_m, posvel.position.east_m


    async def goto(self, x: float, y: float, z: float = None, speed: float = 2.0,
                   sleep_dt: float = 0.05):
        """
        Move to (x,y,z) by interpolating setpoints so PX4 receives consistent offboard setpoints.
        - speed: desired speed in m/s along straight-line distance
        - sleep_dt: time between setpoints in seconds (use 0.05 for 20Hz)
        """
        target_z = z if z is not None else self.default_alt

        curr_x, curr_y = await self._get_current_position()
        dx = x - curr_x
        dy = y - curr_y
        dist = (dx*dx + dy*dy) ** 0.5
        if dist < 0.01:
            print("[control] already at target (distance < 0.01m)")
            return

        # compute number of steps to reach at approx speed
        # time_needed = dist / speed ; steps = time_needed / sleep_dt
        time_needed = max(dist / max(speed, 0.01), sleep_dt)
        steps = max(int(time_needed / sleep_dt), 1)

        print(f"[control] goto target ({x:.2f},{y:.2f}), dist={dist:.2f} m, time~{time_needed:.2f}s, steps={steps}")

        for i in range(1, steps + 1):
            alpha = i / steps
            px = curr_x + alpha * dx
            py = curr_y + alpha * dy
            # send setpoint
            await self.drone.offboard.set_position_ned(PositionNedYaw(px, py, target_z, 0.0))
            await asyncio.sleep(sleep_dt)

        print("[control] goto done")