"""
carla_pid.py — CARLA built-in VehiclePIDController
====================================================
Uses CARLA's own PID controller (agents/navigation/controller.py).
No coordinate frame math. No QP. Just works.

Goals:
  1. Prove the CARLA control loop works end-to-end
  2. Generate a baseline for paper Table II (MPC vs PID)
  3. Visualize the reference path in the viewport

Run:
    python carla_pid.py

Ctrl-C to stop cleanly.
"""

import carla
import math
import random
import time
import sys
import numpy as np

# ── CARLA agents path ─────────────────────────────────────────────
# Path confirmed from your file browser (CARLA_0.9.13_RSS)
CARLA_ROOT = '/Users/Nidhi/Downloads/CARLA_Latest/PythonAPI/carla'
#sys.path.insert(0, f'{CARLA_ROOT}/PythonAPI/carla')
#sys.path.insert(0, f'{CARLA_ROOT}/PythonAPI/carla/agents')
sys.path.insert(0, CARLA_ROOT)

from agents.navigation.controller import VehiclePIDController


# ═══════════════════════════════════════════════════════════
# CONFIG — tweak these
# ═══════════════════════════════════════════════════════════
TARGET_SPEED   = 15.0   # km/h  (PID controller uses km/h, not m/s!)
WAYPOINT_SPACE =  2.0   # metres between reference waypoints
N_WAYPOINTS    =  300   # how many waypoints to build ahead
LOOKAHEAD      =    3   # how many waypoints ahead to target (1 = immediate next)
DT             =  0.05  # seconds per tick (20 Hz)

# PID gains — CARLA defaults are good, tune if needed
LATERAL_GAINS = {
    'K_P': 1.5,    # proportional — higher = sharper turns
    'K_D': 0.1,    # derivative   — higher = less overshoot
    'K_I': 0.05,   # integral     — corrects persistent offset
}
LONGITUDINAL_GAINS = {
    'K_P': 1.0,
    'K_D': 0.1,
    'K_I': 0.05,
}


# ═══════════════════════════════════════════════════════════
# CONNECT
# ═══════════════════════════════════════════════════════════
print("[1] Connecting to CARLA...")
client  = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world   = client.get_world()
# define tm and provide distance and speed info
tm = client.get_trafficmanager(8000)
tm.set_synchronous_mode(True)
tm.set_random_device_seed(100)
tm.set_global_distance_to_leading_vehicle(3.0)
tm.global_percentage_speed_difference(10.0)

bp_lib  = world.get_blueprint_library()
cmap    = world.get_map()
debug   = world.debug
print(f"    Map: {cmap.name}")


# ═══════════════════════════════════════════════════════════
# SYNC MODE
# ═══════════════════════════════════════════════════════════
settings = world.get_settings()
settings.synchronous_mode    = True
settings.fixed_delta_seconds = DT
world.apply_settings(settings)
print(f"[2] Sync mode ON  (dt={DT}s → {1/DT:.0f} Hz)")


# ═══════════════════════════════════════════════════════════
# SPAWN VEHICLE
# ═══════════════════════════════════════════════════════════
print("[3] Spawning vehicle...")
bp       = bp_lib.find('vehicle.tesla.model3')
spawn_pt = cmap.get_spawn_points()[0]
vehicle  = world.spawn_actor(bp, spawn_pt)
#SPAWNING TRAFFIC
traffic = []

spawn_points = cmap.get_spawn_points()[1:]   # skip ego spawn
random.shuffle(spawn_points)

bps = bp_lib.filter("vehicle.*")

for sp in spawn_points[:25]:
    bp = random.choice(bps)
    npc = world.try_spawn_actor(bp, sp)

    if npc:
        # Registers each car with traffic manager so it can drive it
        npc.set_autopilot(True, tm.get_port())
        # control the lights automatically
        tm.update_vehicle_lights(npc, True)
        traffic.append(npc)

vehicle.set_autopilot(False)
for _ in range(5):
    world.tick()

tf  = vehicle.get_transform()
print(f"    ID={vehicle.id}  "
      f"pos=({tf.location.x:.1f}, {tf.location.y:.1f})  "
      f"yaw={tf.rotation.yaw:.1f}°")


# ═══════════════════════════════════════════════════════════
# BUILD REFERENCE WAYPOINTS
# ═══════════════════════════════════════════════════════════
print("[4] Building reference path...")

def build_waypoints(world, vehicle, n=N_WAYPOINTS, spacing=WAYPOINT_SPACE):
    """Follow lane from vehicle's current position."""
    cmap = world.get_map()
    wp   = cmap.get_waypoint(
        vehicle.get_transform().location,
        project_to_road=True,
        lane_type=carla.LaneType.Driving
    )
    waypoints = []
    for _ in range(n):
        waypoints.append(wp)
        nxt = wp.next(spacing)
        if not nxt:
            break
        wp = nxt[0]
    return waypoints

waypoints = build_waypoints(world, vehicle)
print(f"    {len(waypoints)} waypoints built")
print(f"    Start: ({waypoints[0].transform.location.x:.1f}, "
      f"{waypoints[0].transform.location.y:.1f})")
print(f"    End:   ({waypoints[-1].transform.location.x:.1f}, "
      f"{waypoints[-1].transform.location.y:.1f})")

# Draw reference path in CARLA viewport
print("[5] Drawing reference path (green=waypoints, cyan=direction)...")
for i, wp in enumerate(waypoints):
    loc = wp.transform.location
    # Green dot every waypoint
    debug.draw_point(
        carla.Location(x=loc.x, y=loc.y, z=loc.z + 0.5),
        size=0.07,
        color=carla.Color(0, 255, 0),
        life_time=120.0
    )
    # Cyan direction arrow every 20 waypoints
    if i % 20 == 0:
        yaw_rad = math.radians(wp.transform.rotation.yaw)
        end = carla.Location(
            x=loc.x + 3.0 * math.cos(yaw_rad),
            y=loc.y + 3.0 * math.sin(yaw_rad),
            z=loc.z + 0.5
        )
        debug.draw_arrow(
            carla.Location(x=loc.x, y=loc.y, z=loc.z + 0.5),
            end,
            thickness=0.1, arrow_size=0.3,
            color=carla.Color(0, 200, 255),
            life_time=120.0
        )

# Red = start, Blue = end
debug.draw_point(
    carla.Location(x=waypoints[0].transform.location.x,
                   y=waypoints[0].transform.location.y,
                   z=waypoints[0].transform.location.z + 1.0),
    size=0.25, color=carla.Color(255, 0, 0), life_time=120.0
)
debug.draw_point(
    carla.Location(x=waypoints[-1].transform.location.x,
                   y=waypoints[-1].transform.location.y,
                   z=waypoints[-1].transform.location.z + 1.0),
    size=0.25, color=carla.Color(0, 0, 255), life_time=120.0
)
world.tick()  # flush draws


# ═══════════════════════════════════════════════════════════
# PID CONTROLLER
# ═══════════════════════════════════════════════════════════
print("[6] Creating PID controller...")
controller = VehiclePIDController(
    vehicle,
    args_lateral=LATERAL_GAINS,
    args_longitudinal=LONGITUDINAL_GAINS
)


# ═══════════════════════════════════════════════════════════
# SPECTATOR
# ═══════════════════════════════════════════════════════════
spectator = world.get_spectator()

def update_spectator(v, h=28, d=12):
    tf  = v.get_transform()
    yaw = tf.rotation.yaw
    spectator.set_transform(carla.Transform(
        carla.Location(
            x=tf.location.x - d * math.cos(math.radians(yaw)),
            y=tf.location.y - d * math.sin(math.radians(yaw)),
            z=tf.location.z + h),
        carla.Rotation(pitch=-65, yaw=yaw)
    ))

update_spectator(vehicle)


# ═══════════════════════════════════════════════════════════
# HELPER: find closest waypoint index
# ═══════════════════════════════════════════════════════════
def find_closest_wp_idx(vehicle, waypoints, search_from=0):
    """Find the index of the closest waypoint, searching forward."""
    loc = vehicle.get_transform().location
    min_dist = float('inf')
    min_idx  = search_from
    # Search a window ahead to avoid snapping backward
    for i in range(search_from, min(search_from + 50, len(waypoints))):
        wp_loc = waypoints[i].transform.location
        dist   = math.sqrt((loc.x - wp_loc.x)**2 + (loc.y - wp_loc.y)**2)
        if dist < min_dist:
            min_dist = dist
            min_idx  = i
    return min_idx, min_dist


# ═══════════════════════════════════════════════════════════
# MAIN LOOP
# ═══════════════════════════════════════════════════════════
print(f"\n── PID loop running at {TARGET_SPEED} km/h (Ctrl-C to stop) ──\n")

wp_idx    = 0
step      = 0
err_hist  = []
spd_hist  = []
loop_start = time.time()

try:
    while wp_idx < len(waypoints) - LOOKAHEAD - 1:
        world.tick()
        step += 1

        # Find where we are on the path
        wp_idx, crosstrack = find_closest_wp_idx(vehicle, waypoints, wp_idx)

        # Target: look LOOKAHEAD waypoints ahead
        target_idx = min(wp_idx + LOOKAHEAD, len(waypoints) - 1)
        target_wp  = waypoints[target_idx]

        # PID control step — returns a carla.VehicleControl directly
        control = controller.run_step(TARGET_SPEED, target_wp)
        vehicle.apply_control(control)

        update_spectator(vehicle)

        # ── Live debug: magenta target, HUD text ──────────────
        tgt_loc = target_wp.transform.location
        debug.draw_point(
            carla.Location(x=tgt_loc.x, y=tgt_loc.y, z=tgt_loc.z + 1.0),
            size=0.15,
            color=carla.Color(255, 0, 200),  # magenta = current target
            life_time=DT * 2
        )

        tf  = vehicle.get_transform()
        vel = vehicle.get_velocity()
        spd = math.sqrt(vel.x**2 + vel.y**2) * 3.6  # convert to km/h

        hud = (f"step:{step} spd:{spd:.1f}km/h "
               f"err:{crosstrack:.2f}m wp:{wp_idx}/{len(waypoints)}")
        debug.draw_string(
            carla.Location(x=tf.location.x - 2,
                           y=tf.location.y,
                           z=tf.location.z + 5),
            hud,
            color=carla.Color(255, 255, 255),
            life_time=DT * 2
        )

        err_hist.append(crosstrack)
        spd_hist.append(spd)

        # Log every 20 steps
        if step % 20 == 0:
            progress = wp_idx / len(waypoints) * 100
            print(f"step {step:4d} | "
                  f"wp {wp_idx:3d}/{len(waypoints)} ({progress:.0f}%) | "
                  f"spd={spd:.1f}km/h | "
                  f"crosstrack={crosstrack:.2f}m | "
                  f"steer={control.steer:+.3f} "
                  f"throttle={control.throttle:.2f} "
                  f"brake={control.brake:.2f}")

    print("\n✓ Reached end of reference path!")

except KeyboardInterrupt:
    print("\n── Stopped by user ──")

finally:
    elapsed = time.time() - loop_start
    print(f"\n── Summary ──────────────────────────")
    print(f"   Steps:            {step}")
    print(f"   Time:             {elapsed:.1f}s")
    print(f"   Waypoints reached: {wp_idx}/{len(waypoints)}")
    if err_hist:
        print(f"   Mean crosstrack:  {np.mean(err_hist):.3f} m")
        print(f"   Max  crosstrack:  {np.max(err_hist):.3f} m")
    if spd_hist:
        print(f"   Mean speed:       {np.mean(spd_hist):.1f} km/h")

    # Restore CARLA settings and clean up
    settings.synchronous_mode = False
    world.apply_settings(settings)
    vehicle.destroy()
    # loop through all cars and destroy in order to clean up
    for car in traffic:
        if car.is_alive:
            car.destroy()
    print("   Cleaned up. Done.")
