"""ORBIT - FIXED X-Y MAPPING"""

import asyncio
import math
from mavsdk import System
from mavsdk.offboard import VelocityNedYaw, OffboardError

OFFSET_X = 1.02994
OFFSET_Y = 0.959991

TARGET_GAZEBO_X = 5.0   # X trong Gazebo
TARGET_GAZEBO_Y = 3.0   # Y trong Gazebo

ORBIT_RADIUS = 4.0
ORBIT_ALTITUDE = 4.0
NUM_ORBITS = 3
ORBIT_SPEED = 0.2
APPROACH_GAIN = 1.5
ORBIT_GAIN = 2.0
APPROACH_MAX_SPEED = 5.0
LOOP_RATE = 20

async def smooth_approach_to_orbit(drone, center_n, center_e, radius, altitude):
    print("\nSMOOTH APPROACH\n")
    async for pos in drone.telemetry.position_velocity_ned():
        curr_n, curr_e = pos.position.north_m, pos.position.east_m
        break
    to_center_n, to_center_e = center_n - curr_n, center_e - curr_e
    angle_to_center = math.atan2(to_center_e, to_center_n)
    entry_angle = angle_to_center + math.pi
    hover_radius = radius * 1.5
    hover_n = center_n + hover_radius * math.cos(entry_angle)
    hover_e = center_e + hover_radius * math.sin(entry_angle)
    print(f"Hover point: Gz({OFFSET_X + hover_n:.1f}, {OFFSET_Y + hover_e:.1f})\n")
    kp, dt = APPROACH_GAIN, 1.0 / LOOP_RATE
    timeout, start = 30.0, asyncio.get_event_loop().time()
    while asyncio.get_event_loop().time() - start < timeout:
        async for pos in drone.telemetry.position_velocity_ned():
            curr_n, curr_e, curr_d = pos.position.north_m, pos.position.east_m, pos.position.down_m
            break
        error_n, error_e, error_d = hover_n - curr_n, hover_e - curr_e, -altitude - curr_d
        dist = math.sqrt(error_n**2 + error_e**2)
        vn, ve, vd = kp * error_n, kp * error_e, 1.5 * error_d
        speed = math.sqrt(vn**2 + ve**2)
        if speed > APPROACH_MAX_SPEED:
            vn, ve = vn * APPROACH_MAX_SPEED / speed, ve * APPROACH_MAX_SPEED / speed
        await drone.offboard.set_velocity_ned(VelocityNedYaw(vn, ve, vd, math.degrees(math.atan2(error_e, error_n))))
        print(f"  Dist: {dist:5.1f}m", end='\r')
        if dist < 2.0 and abs(error_d) < 0.5:
            print(f"\nAt hover!\n")
            break
        await asyncio.sleep(dt)
    print("Stabilizing...")
    for _ in range(40):
        await drone.offboard.set_velocity_ned(VelocityNedYaw(0, 0, 0, 0))
        await asyncio.sleep(dt)
    print("Arc entry...")
    spiral_steps = int(5.0 * LOOP_RATE)
    for step in range(spiral_steps):
        progress = step / spiral_steps
        smooth_progress = 1 - (1 - progress) ** 2
        current_radius = hover_radius + (radius - hover_radius) * smooth_progress
        target_n = center_n + current_radius * math.cos(entry_angle)
        target_e = center_e + current_radius * math.sin(entry_angle)
        async for pos in drone.telemetry.position_velocity_ned():
            curr_n, curr_e, curr_d = pos.position.north_m, pos.position.east_m, pos.position.down_m
            break
error_n, error_e = target_n - curr_n, target_e - curr_e
        vn, ve, vd = 1.2 * error_n, 1.2 * error_e, 1.5 * (-altitude - curr_d)
        tangent_speed = ORBIT_SPEED * radius * smooth_progress
        vn += -tangent_speed * math.sin(entry_angle)
        ve += tangent_speed * math.cos(entry_angle)
        await drone.offboard.set_velocity_ned(VelocityNedYaw(vn, ve, vd, math.degrees(math.atan2(center_e - curr_e, center_n - curr_n))))
        await asyncio.sleep(dt)
    print("Entered orbit!\n")
    return entry_angle

async def smooth_orbit(drone, center_n, center_e, radius, altitude, num_orbits, angular_speed, start_angle):
    print("ORBITING\n")
    angle, dt = start_angle, 1.0 / LOOP_RATE
    total, end = num_orbits * 2 * math.pi, start_angle + num_orbits * 2 * math.pi
    last = 0
    while angle < end:
        angle += angular_speed * dt
        curr = int((angle - start_angle) / (2 * math.pi)) + 1
        if curr > last:
            print(f"\nOrbit {curr}/{num_orbits}\n")
            last = curr
        target_n = center_n + radius * math.cos(angle)
        target_e = center_e + radius * math.sin(angle)
        async for pos in drone.telemetry.position_velocity_ned():
            curr_n, curr_e, curr_d = pos.position.north_m, pos.position.east_m, pos.position.down_m
            break
        error_n, error_e = target_n - curr_n, target_e - curr_e
        vn = -radius * angular_speed * math.sin(angle) + ORBIT_GAIN * error_n
        ve = radius * angular_speed * math.cos(angle) + ORBIT_GAIN * error_e
        vd = 1.5 * (-altitude - curr_d)
        await drone.offboard.set_velocity_ned(VelocityNedYaw(vn, ve, vd, math.degrees(math.atan2(center_e - curr_e, center_n - curr_n))))
        if int(math.degrees(angle - start_angle)) % 30 == 0:
            dist = math.sqrt((curr_n - center_n)**2 + (curr_e - center_e)**2)
            print(f"  {((angle-start_angle)/total*100):5.1f}% | Dist:{dist:4.1f}m")
        await asyncio.sleep(dt)
    print(f"\nDone!\n")

async def main():
    print("\nORBIT - FIXED COORDINATE MAPPING\n")
    
    # 🔧 FIX: SWAP X-Y conversion!
    target_n = TARGET_GAZEBO_Y - OFFSET_Y  # Y → N (SWAPPED!)
    target_e = TARGET_GAZEBO_X - OFFSET_X  # X → E (SWAPPED!)
    
    print(f"Target Gazebo: ({TARGET_GAZEBO_X}, {TARGET_GAZEBO_Y})")
    print(f"Target NED:    ({target_n:.2f}, {target_e:.2f}) [FIXED SWAP]\n")
    
    drone = System()
    await drone.connect(system_address="udp://:14540")
    async for state in drone.core.connection_state():
        if state.is_connected:
            break
    async for health in drone.telemetry.health():
        if health.is_global_position_ok:
            break
    await drone.action.arm()
    await drone.action.set_takeoff_altitude(ORBIT_ALTITUDE)
    await drone.action.takeoff()
    await asyncio.sleep(10)
    await drone.offboard.set_velocity_ned(VelocityNedYaw(0, 0, 0, 0))
    await drone.offboard.start()