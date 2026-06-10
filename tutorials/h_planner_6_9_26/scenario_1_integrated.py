"""
scenario_1_integrated.py — Green Light, Clear Intersection
===========================================================
Scenario 1 running with the full hierarchical planner.

Planning loop (replaces ego.set_autopilot(True)):

    Every REPLAN_INTERVAL ticks:
        1. SamplingTrajectoryGenerator  →  N candidate trajectories
        2. TWTLEvaluator                →  ρ_P0, ρ_P1, η_P2 per trajectory
        3. HierarchicalPlanner          →  P0 filter → P1 filter → P2 selection
        4. PathExecutor.set_trajectory  →  load best trajectory

    Every tick:
        5. PathExecutor.step            →  compute VehicleControl
        6. ego.apply_control            →  send to CARLA

TWTL Formula for Scenario 1:
    φ₁ = [ H₂ π_intersection ][0,15]  ∧  H_T ¬π_collision  ∧  H_T π_green_ok

Author: Ahmad Ahmad | For: Nidhi's Autonomous Driving Course
"""

import carla
import math
import os
import sys

# Make sure planner.py is on the path
sys.path.insert(0, os.path.dirname(__file__))
from planner import HierarchicalPlanner, PathExecutor, RobustnessLogger

# ─── CONFIG ──────────────────────────────────────────────────────────────────

TOWN              = 'Town05'
HOST              = 'localhost'
PORT              = 2000
TIMEOUT           = 20.0
TICK_DELTA        = 0.05       # 20 Hz
RUN_TICKS         = 300        # 15 seconds
REPLAN_INTERVAL   = 10         # replan every 10 ticks (every 0.5s)
OUTPUT_DIR        = 'output/scenario_1_integrated'

# ─── SCENARIO SETUP ──────────────────────────────────────────────────────────

def setup(world):
    """Spawn ego, freeze light GREEN, attach overhead camera."""
    bp_lib    = world.get_blueprint_library()
    carla_map = world.get_map()
    actors    = {}

    # Ego
    ego_bp = bp_lib.find('vehicle.tesla.model3')
    ego_bp.set_attribute('color', '255,255,255')

    # Get all spawn points
    spawn_pts  = carla_map.get_spawn_points()

    # Use spawn point index 0 as a reliable fallback.
    # Town05 spawn points are pre-validated road positions — much safer than
    # searching by coordinate, which can miss if no spawn exists near that location.
    # Change the index here to try different starting positions.
    SPAWN_INDEX = 0
    spawn_tf    = spawn_pts[SPAWN_INDEX]
    print(f"[S1] Using spawn point #{SPAWN_INDEX}: {spawn_tf.location}")

    ego        = world.spawn_actor(ego_bp, spawn_tf)
    actors['ego'] = ego
    print(f"[S1] Ego spawned at {ego.get_location()}")

    # Traffic light → GREEN, frozen
    # Search up to 80m — spawn index 0 may be far from an intersection
    lights = world.get_actors().filter('traffic.traffic_light')
    print(f"[S1] Found {len(list(lights))} traffic lights in world")
    light  = min(lights,
                 key=lambda l: l.get_location().distance(ego.get_location()),
                 default=None)
    if light:
        dist = light.get_location().distance(ego.get_location())
        print(f"[S1] Nearest light {light.id} is {dist:.1f}m away")
        light.set_state(carla.TrafficLightState.Green)
        light.freeze(True)
        light.set_green_time(60.0)
        actors['light'] = light
        print(f"[S1] Light {light.id} → GREEN (frozen)")

    # Overhead camera
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    cam_bp = bp_lib.find('sensor.camera.rgb')
    cam_bp.set_attribute('image_size_x', '1280')
    cam_bp.set_attribute('image_size_y', '720')
    cam = world.spawn_actor(
        cam_bp,
        carla.Transform(carla.Location(z=25.0), carla.Rotation(pitch=-90.0)),
        attach_to=ego
    )
    cam.listen(lambda img: img.save_to_disk(f'{OUTPUT_DIR}/frame_{img.frame:06d}.png'))
    actors['camera'] = cam

    return actors

# ─── MAIN ────────────────────────────────────────────────────────────────────

def run():
    client = carla.Client(HOST, PORT)
    client.set_timeout(TIMEOUT)

    print(f"[S1] Loading {TOWN}...")
    world = client.load_world(TOWN)
    world.set_weather(carla.WeatherParameters.ClearNoon)

    settings = world.get_settings()
    settings.synchronous_mode    = True
    settings.fixed_delta_seconds = TICK_DELTA
    world.apply_settings(settings)

    carla_map = world.get_map()
    actors    = {}

    try:
        actors = setup(world)
        ego    = actors['ego']
        light  = actors.get('light')

        # Get speed limit for this stretch of road
        speed_limit_ms = ego.get_speed_limit() / 3.6   # km/h → m/s

        # ── Instantiate planner components ───────────────────────────────────
        planner  = HierarchicalPlanner(carla_map)
        executor = PathExecutor()
        logger   = RobustnessLogger("Scenario 1 — Green, Clear")

        print(f"\n[S1] Starting planning loop "
              f"(replan every {REPLAN_INTERVAL} ticks = {REPLAN_INTERVAL*TICK_DELTA:.2f}s)\n")

        for tick in range(RUN_TICKS):
            world.tick()
            t = tick * TICK_DELTA

            # ── Replan ───────────────────────────────────────────────────────
            if tick % REPLAN_INTERVAL == 0:
                best = planner.plan(ego, world, light, speed_limit_ms)
                executor.set_trajectory(best)

            # ── Execute one step ─────────────────────────────────────────────
            control = executor.step(ego)
            ego.apply_control(control)

            # ── Log robustness ───────────────────────────────────────────────
            logger.record(tick, t, best)

            # ── Console status ───────────────────────────────────────────────
            if tick % 20 == 0:
                loc   = ego.get_location()
                vel   = ego.get_velocity()
                speed = math.sqrt(vel.x**2 + vel.y**2) * 3.6
                wp    = carla_map.get_waypoint(loc)
                junc  = "⚡ IN JUNCTION" if wp.is_junction else ""
                print(f"  t={t:5.1f}s | pos=({loc.x:6.1f},{loc.y:6.1f}) | "
                      f"speed={speed:5.1f} km/h | "
                      f"ρ_P0={best.rho_p0:+.2f} ρ_P1={best.rho_p1:+.2f} "
                      f"η_P2={best.eta_p2:.3f} {junc}")

        logger.report()

    finally:
        print("\n[S1] Cleaning up...")
        for actor in actors.values():
            if actor and actor.is_alive:
                actor.destroy()
        settings = world.get_settings()
        settings.synchronous_mode = False
        world.apply_settings(settings)
        print("[S1] Done.")


if __name__ == '__main__':
    run()
