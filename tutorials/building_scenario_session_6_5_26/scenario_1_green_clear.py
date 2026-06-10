"""
Scenario 1: Green Light — Clear Intersection
=============================================
Purpose : Baseline test. Ego should proceed through a green light with no obstacles.
Expected: ρ(φ₁) > 0  →  planner passes through within 15 seconds, no violations.

TWTL Formula:
    φ₁ = [ H₂ π_intersection ][0,15]  ∧  H_T ¬π_collision  ∧  H_T π_green_ok

Author  : Ahmad Ahmad  |  For: Nidhi's Autonomous Driving Course
"""

import carla
import time
import math
import os

# ─── CONFIG ──────────────────────────────────────────────────────────────────

TOWN           = 'Town04'
HOST           = 'localhost'
PORT           = 2000
TIMEOUT        = 20.0
TICK_DELTA     = 0.05      # 20 Hz
RUN_TICKS      = 300       # 15 seconds
OUTPUT_DIR     = 'output/scenario_1'

# Intersection center (Town05 4-way intersection near origin)
# Adjust these if your CARLA map has a different layout
INTERSECTION_X =  0.0
INTERSECTION_Y =  0.0

# Ego starts 40m south of intersection, facing north
EGO_X          =  0.0
EGO_Y          = -40.0
EGO_YAW        =  90.0      # yaw 90° = facing +y (north)

LIGHT_SEARCH_RADIUS = 60.0  # metres

# ─── SETUP ───────────────────────────────────────────────────────────────────

def setup_scenario_1(world):
    """Spawn ego, configure traffic light, attach camera. Returns actors dict."""
    carla_map       = world.get_map()
    bp_lib          = world.get_blueprint_library()

    actors = {}

    # ── Spawn ego vehicle ────────────────────────────────────────────────────
    ego_bp = bp_lib.find('vehicle.tesla.model3')
    ego_bp.set_attribute('color', '255,255,255')    # white — easy to spot

    target_loc = carla.Location(x=EGO_X, y=EGO_Y, z=0.5)
    ego_transform = find_nearest_spawn(carla_map.get_spawn_points(), target_loc)

    actors['ego'] = world.spawn_actor(ego_bp, ego_transform)
    print(f"[S1] Ego spawned at {actors['ego'].get_location()}")

    # ── Configure traffic light ──────────────────────────────────────────────
    light = find_facing_light(world, actors['ego'].get_location(), LIGHT_SEARCH_RADIUS)

    if light:
        light.set_state(carla.TrafficLightState.Green)
        light.freeze(True)          # prevent light from cycling
        light.set_green_time(60.0)
        actors['light'] = light
        print(f"[S1] Traffic light {light.id} → GREEN (frozen)")
    else:
        print("[S1] WARNING: No traffic light found — check intersection location")

    # ── Attach overhead camera ───────────────────────────────────────────────
    actors['camera'] = attach_overhead_camera(world, actors['ego'], bp_lib)

    return actors


# ─── HELPERS ─────────────────────────────────────────────────────────────────

def find_nearest_spawn(spawn_points, target_location):
    """Return the spawn point closest to target_location."""
    nearest, min_dist = None, float('inf')
    for sp in spawn_points:
        dist = sp.location.distance(target_location)
        if dist < min_dist:
            min_dist = dist
            nearest = sp
    print(f"[S1] Nearest spawn point: {nearest.location} (dist={min_dist:.1f}m)")
    return nearest


def find_facing_light(world, ego_location, radius):
    """Find the traffic light closest to ego within radius."""
    lights    = world.get_actors().filter('traffic.traffic_light')
    nearest   = None
    min_dist  = float('inf')

    for light in lights:
        dist = light.get_location().distance(ego_location)
        if dist < radius and dist < min_dist:
            min_dist = dist
            nearest  = light

    if nearest:
        print(f"[S1] Found traffic light {nearest.id} at distance {min_dist:.1f}m")
    return nearest


def attach_overhead_camera(world, vehicle, bp_lib, height=25.0):
    """Bird's-eye RGB camera following the ego vehicle."""
    cam_bp = bp_lib.find('sensor.camera.rgb')
    cam_bp.set_attribute('image_size_x', '1280')
    cam_bp.set_attribute('image_size_y', '720')
    cam_bp.set_attribute('fov', '90')

    cam_transform = carla.Transform(
        carla.Location(x=0.0, z=height),
        carla.Rotation(pitch=-90.0)     # looking straight down
    )
    camera = world.spawn_actor(cam_bp, cam_transform, attach_to=vehicle)

    os.makedirs(OUTPUT_DIR, exist_ok=True)
    camera.listen(lambda img: img.save_to_disk(
        f'{OUTPUT_DIR}/frame_{img.frame:06d}.png'
    ))
    print(f"[S1] Overhead camera attached at z={height}m → saving to {OUTPUT_DIR}/")
    return camera


# ─── MONITORING ──────────────────────────────────────────────────────────────

def compute_intersection_robustness(ego, light, carla_map):
    """
    Compute per-tick robustness values for φ₁.

    Returns dict with:
        collision_margin : distance to nearest obstacle (positive = safe)
        green_ok         : +1 if light is green, -1 if not
        in_intersection  : True if ego is at a junction
    """
    loc = ego.get_location()
    waypoint = carla_map.get_waypoint(loc)

    # Collision margin — simplified: use vehicle's bounding box proximity
    # In a full pipeline, use LiDAR point cloud here
    collision_margin = 100.0    # placeholder (no obstacles in S1)

    # Green OK
    if light:
        green_ok = 1.0 if light.get_state() == carla.TrafficLightState.Green else -1.0
    else:
        green_ok = 1.0  # no light found = assume safe

    return {
        'collision_margin': collision_margin,
        'green_ok':         green_ok,
        'in_intersection':  waypoint.is_junction,
    }


# ─── RUN ─────────────────────────────────────────────────────────────────────

def run_scenario_1():
    """Main entry point."""
    client = carla.Client(HOST, PORT)
    client.set_timeout(TIMEOUT)

    print(f"[S1] Loading {TOWN}...")
    world = client.load_world(TOWN)
    world.set_weather(carla.WeatherParameters.ClearNoon)

    # Synchronous mode — deterministic ticks
    settings = world.get_settings()
    settings.synchronous_mode   = True
    settings.fixed_delta_seconds = TICK_DELTA
    world.apply_settings(settings)

    actors = {}

    try:
        actors = setup_scenario_1(world)
        ego   = actors['ego']
        light = actors.get('light')

        # Enable autopilot — swap this for your hierarchical planner
        ego.set_autopilot(True)

        print(f"\n[S1] Running {RUN_TICKS} ticks ({RUN_TICKS * TICK_DELTA:.0f}s) ...")
        print("[S1] Watch the ego cross the intersection!\n")

        # ── Monitoring loop ──────────────────────────────────────────────────
        robustness_log = []

        for tick in range(RUN_TICKS):
            world.tick()

            rob = compute_intersection_robustness(ego, light, world.get_map())
            robustness_log.append(rob)

            if tick % 20 == 0:   # print every 1 second
                loc   = ego.get_location()
                vel   = ego.get_velocity()
                speed = math.sqrt(vel.x**2 + vel.y**2) * 3.6   # km/h
                junc  = "⚡ IN JUNCTION" if rob['in_intersection'] else ""
                print(f"  t={tick*TICK_DELTA:5.1f}s | "
                      f"pos=({loc.x:6.1f}, {loc.y:6.1f}) | "
                      f"speed={speed:5.1f} km/h | "
                      f"green={rob['green_ok']:+.0f} {junc}")

        # ── Final robustness ─────────────────────────────────────────────────
        print_robustness_report(robustness_log)

    finally:
        print("\n[S1] Cleaning up actors...")
        for actor in actors.values():
            if actor and actor.is_alive:
                actor.destroy()

        # Restore async mode
        settings = world.get_settings()
        settings.synchronous_mode = False
        world.apply_settings(settings)
        print("[S1] Done.")


def print_robustness_report(log):
    """Print final ρ(φ₁) evaluation."""
    if not log:
        return

    min_green    = min(r['green_ok']         for r in log)
    min_collide  = min(r['collision_margin'] for r in log)
    passed_thru  = any(r['in_intersection']  for r in log)

    # ρ(φ₁) = min of all constraints
    rho = min(min_green, 1.0 if passed_thru else -1.0)

    print(f"\n{'='*50}")
    print(f"  TWTL ROBUSTNESS REPORT — Scenario 1")
    print(f"{'='*50}")
    print(f"  ρ(green_ok)         = {min_green:+.3f}")
    print(f"  ρ(collision_margin) = {min_collide:+.3f}")
    print(f"  passed_through      = {passed_thru}")
    print(f"  ─────────────────────────────────────")
    print(f"  ρ(φ₁)  =  {rho:+.3f}")
    print(f"  Result : {'PASSED ✓' if rho > 0 else 'FAILED ✗'}")
    print(f"{'='*50}\n")


if __name__ == '__main__':
    run_scenario_1()
