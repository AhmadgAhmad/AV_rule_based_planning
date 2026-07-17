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
import random
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




# ─── WEATHER ──────────────────────────────────────────────────────────────────

def _debug_weather() -> carla.WeatherParameters:
    """
    Flat, glare-free lighting tuned for debugging.

    The key knobs:
        sun_altitude_angle  — 90° = directly overhead (no long shadows, no glare)
        sun_azimuth_angle   — direction of sun, irrelevant at altitude=90
        cloudiness          — 60–80 softens shadows without going grey
        fog_density         — 0 = crystal clear
        precipitation       — 0 = dry road
        wetness             — 0 = no reflections on road surface (those add glare too)
        scattering_intensity — 0 = disables atmospheric bloom entirely

    Switch sun_altitude_angle down to ~45 for a nicer render once debugging is done.
    """
    w = carla.WeatherParameters(
        cloudiness             = 60.0,   # partial cloud — softens harsh shadows
        precipitation          = 0.0,
        precipitation_deposits = 0.0,
        wind_intensity         = 0.0,
        sun_azimuth_angle      = 0.0,
        sun_altitude_angle     = 90.0,   # ← sun straight up = zero glare on camera
        fog_density            = 0.0,
        fog_distance           = 0.0,
        fog_falloff            = 0.0,
        wetness                = 0.0,    # ← dry road = no mirror reflections
        scattering_intensity   = 0.0,    # ← kills the bloom haze entirely
        mie_scattering_scale   = 0.0,
        rayleigh_scattering_scale = 0.0331,
    )
    return w


# ─── SCENARIO SETUP ───────────────────────────────────────────────────────────


def setup(world, tm):
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
    ego = None
    SPAWN_INDEX = 12
    spawn_tf = spawn_pts[SPAWN_INDEX]

    ego = world.try_spawn_actor(ego_bp, spawn_tf)
    if ego is None:
        raise RuntimeError(f"Could not spawn ego at spawn point {SPAWN_INDEX}")

    print(f"[S1] Ego spawned at spawn point #{SPAWN_INDEX}: {spawn_tf.location}")
    actors['ego'] = ego


    # SPAWN TRAFFIC
    traffic = []

    spawn_points = carla_map.get_spawn_points()

    # Remove the ego spawn point so no NPC takes it
    del spawn_points[SPAWN_INDEX]

    random.shuffle(spawn_points)

    bps = bp_lib.filter("vehicle.*")

    for sp in spawn_points[:25]:
        bp = random.choice(bps)
        npc = world.try_spawn_actor(bp, sp)

        if npc:
            npc.set_autopilot(True, tm.get_port())
            tm.update_vehicle_lights(npc, True)
            traffic.append(npc)

    actors["traffic"] = traffic


    # Give the camera a spectator view angle
    world.get_spectator().set_transform(carla.Transform(
        ego.get_location() + carla.Location(z=20),
        carla.Rotation(pitch=-90)
    ))

    world.tick()

    world.debug.draw_string(
        ego.get_location() + carla.Location(z=3),
        "EGO CAR HERE",
        draw_shadow=False,
        color=carla.Color(255, 0, 0),
        life_time=20.0,
        persistent_lines=True
    )

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
    carla.Transform(
        carla.Location(x=-10.0, y=0.0, z=6.0),
        carla.Rotation(pitch=-25.0, yaw=0.0)
    ),
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

    print("[1] Connecting to CARLA...")
    # define tm and provide distance and speed info
    tm = client.get_trafficmanager(8000)
    tm.set_synchronous_mode(True)
    tm.set_random_device_seed(100)
    tm.set_global_distance_to_leading_vehicle(3.0)
    tm.global_percentage_speed_difference(10.0)

    # ClearNoon causes heavy bloom/lens flare — use a custom weather
    # tuned for visibility during debugging.
    world.set_weather(_debug_weather())

    settings = world.get_settings()
    settings.synchronous_mode    = True
    settings.fixed_delta_seconds = TICK_DELTA
    world.apply_settings(settings)

    carla_map = world.get_map()
    actors    = {}

    try:
        actors = setup(world, tm)
        ego    = actors['ego']
        light  = actors.get('light')

        # Get speed limit for this stretch of road
        speed_limit_ms = ego.get_speed_limit() / 3.6   # km/h → m/s

        # ── Instantiate planner components ───────────────────────────────────
        planner  = HierarchicalPlanner(carla_map)
        executor = PathExecutor(dt=TICK_DELTA)
        logger   = RobustnessLogger("Scenario 1 — Green, Clear")

        print(f"\n[S1] Starting planning loop "
              f"(replan every {REPLAN_INTERVAL} ticks = {REPLAN_INTERVAL*TICK_DELTA:.2f}s)\n")

        for tick in range(RUN_TICKS):
            world.tick()
            t = tick * TICK_DELTA

            # ── Replan ───────────────────────────────────────────────────────
            if tick % REPLAN_INTERVAL == 0:
                best, candidates = planner.plan(ego, world, light, speed_limit_ms)
                # Visualize all candidate trajectories
                for traj in candidates:
                    for i in range(len(traj.waypoints) - 1):
                        world.debug.draw_line(
                            traj.waypoints[i],
                            traj.waypoints[i + 1],
                            thickness=0.1,
                            color=carla.Color(0, 255, 0),
                            life_time=2.0
                        )

                # Highlight the chosen trajectory
                for i in range(len(best.waypoints) - 1):
                    world.debug.draw_line(
                        best.waypoints[i],
                        best.waypoints[i + 1],
                        thickness=0.2,
                        color=carla.Color(0, 0, 255),
                        life_time=2.0
                    )


                executor.set_trajectory(best)
                # Highlight the chosen trajectory in BLUE
                for i in range(len(best.waypoints) - 1):
                    p1 = best.waypoints[i]
                    p2 = best.waypoints[i + 1]
                    world.debug.draw_line(
                        p1,
                        p2,
                        thickness=0.2,
                        color=carla.Color(0, 0, 255),   # blue
                        life_time=0.5
                    )


            # ── Execute one step ─────────────────────────────────────────────
            control = executor.step(ego)
            ego.apply_control(control)
            # Trying a smoother, top down spectator view
            spectator = world.get_spectator()
            tf = ego.get_transform()

            spectator.set_transform(carla.Transform(
                tf.location + carla.Location(x=0, y = 0, z=30),
                carla.Rotation(pitch=-70, yaw=tf.rotation.yaw)
            ))

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
         # loop through all cars and destroy in order to clean up
        for car in actors.get("traffic", []):
            if car.is_alive:
                car.destroy()
        settings = world.get_settings()
        settings.synchronous_mode = False
        world.apply_settings(settings)
        print("[S1] Done.")


if __name__ == '__main__':
    run()
