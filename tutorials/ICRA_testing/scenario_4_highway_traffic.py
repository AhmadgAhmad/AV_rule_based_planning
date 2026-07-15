"""
scenario_4_highway_traffic.py — Dense Highway Traffic, Forced P2 Ranking
=============================================================================
Exercises Stage 3 (P2) of the hierarchical planner.

Setup:
    ~20 autopilot vehicles are spawned around the ego on a highway-speed
    stretch of road (same batch SpawnActor + SetAutopilot pattern as
    CARLA's generate_traffic.py, trimmed to vehicles-only and biased to
    spawn near the ego instead of across the whole map). With the local
    speed limit above ContextDetector.HIGHWAY_SPEED_LIMIT, ContextDetector
    reports Context.HIGHWAY, which shifts TWTLEvaluator._evaluate_p2()'s
    weights toward efficiency=0.5 (vs. 0.2 in INTERSECTION context).

    Because most candidates stay P0-safe and P1-legal in free-flowing
    traffic, Stage 3 is what actually decides the outcome here: among the
    surviving trajectories, the planner should consistently favor the
    higher-speed_factor ones over the slower, more "comfortable" ones —
    the opposite preference from what Scenario 1 (INTERSECTION context)
    would show for the same set of candidates.

TWTL Formula for Scenario 4:
    φ₄ = H_T ¬π_collision  ∧  H_T π_speed_ok  ∧  (soft) argmax η_efficiency

    i.e. hard safety and legality hold throughout, and among the trajectories
    that satisfy them, the soft preference is for efficiency over comfort.

What to watch in the console:
    [Planner] Stage 3 (P2, ctx=highway): selected α=...
    α (speed_factor) should trend toward the higher end (0.9–1.0) here,
    compared to Scenario 1's more conservative selection near an intersection.

Author: Ahmad Ahmad | For: Nidhi's Autonomous Driving Course
"""

import argparse
import math
import os
import sys

import carla
from numpy import random

sys.path.insert(0, os.path.dirname(__file__))
from planner import HierarchicalPlanner, PathExecutor, RobustnessLogger
from scenario_utils import (debug_weather, attach_overhead_camera, top_down_follow,
                             start_recording, stop_recording, spawn_ego)

# ─── CONFIG ──────────────────────────────────────────────────────────────────

TOWN             = 'Town04'   # Town04 has the long highway loop
HOST             = 'localhost'
PORT             = 2000
TIMEOUT          = 20.0
TICK_DELTA       = 0.05
RUN_TICKS        = 400
REPLAN_INTERVAL  = 10
OUTPUT_DIR       = 'output/scenario_4_highway_traffic'
SPAWN_INDEX      = 12
NUM_TRAFFIC_CARS = 20
TRAFFIC_RADIUS_M = 60.0       # only spawn points within this radius of the ego


def spawn_nearby_traffic(world, carla_map, ego, client, traffic_manager):
    """Batch-spawn autopilot vehicles at spawn points near the ego, same
    two-line SpawnActor().then(SetAutopilot(...)) idiom as
    generate_traffic.py, filtered down to nearby spawn points so traffic
    actually surrounds the ego instead of scattering across the whole map."""
    bp_lib = world.get_blueprint_library()
    blueprints = [bp for bp in bp_lib.filter('vehicle.*')
                  if int(bp.get_attribute('number_of_wheels')) == 4]

    all_spawns = carla_map.get_spawn_points()
    ego_loc = ego.get_location()
    nearby = [sp for sp in all_spawns if sp.location.distance(ego_loc) < TRAFFIC_RADIUS_M]
    random.shuffle(nearby)

    if len(nearby) < NUM_TRAFFIC_CARS:
        print(f"[S4] Only {len(nearby)} spawn points within {TRAFFIC_RADIUS_M}m of ego "
              f"(wanted {NUM_TRAFFIC_CARS}) — spawning what's available.")

    SpawnActor = carla.command.SpawnActor
    SetAutopilot = carla.command.SetAutopilot
    FutureActor = carla.command.FutureActor

    batch = []
    for transform in nearby[:NUM_TRAFFIC_CARS]:
        bp = random.choice(blueprints)
        if bp.has_attribute('color'):
            bp.set_attribute('color', random.choice(bp.get_attribute('color').recommended_values))
        bp.set_attribute('role_name', 'autopilot')
        batch.append(SpawnActor(bp, transform)
                     .then(SetAutopilot(FutureActor, True, traffic_manager.get_port())))

    vehicle_ids = []
    for response in client.apply_batch_sync(batch, True):
        if response.error:
            print(f"[S4] Spawn error: {response.error}")
        else:
            vehicle_ids.append(response.actor_id)

    print(f"[S4] Spawned {len(vehicle_ids)} autopilot vehicles near the ego")
    return vehicle_ids


def setup(world, client, traffic_manager, args):
    carla_map = world.get_map()
    actors = {}

    ego = spawn_ego(world, carla_map)
    actors['ego'] = ego
    print(f"[S4] Ego spawned at {ego.get_location()}, "
          f"speed limit {ego.get_speed_limit():.0f} km/h")

    world.get_spectator().set_transform(carla.Transform(
        ego.get_location() + carla.Location(z=25), carla.Rotation(pitch=-90)))
    world.tick()

    world.debug.draw_string(ego.get_location() + carla.Location(z=3), "EGO CAR HERE",
                             draw_shadow=False, color=carla.Color(255, 0, 0),
                             life_time=20.0, persistent_lines=True)

    actors['traffic_ids'] = spawn_nearby_traffic(world, carla_map, ego, client, traffic_manager)
    actors['camera'] = attach_overhead_camera(world, ego, OUTPUT_DIR)
    return actors


def run(args):
    client = carla.Client(HOST, PORT)
    client.set_timeout(TIMEOUT)

    print(f"[S4] Loading {TOWN}...")
    world = client.load_world(TOWN)
    world.set_weather(debug_weather())

    traffic_manager = client.get_trafficmanager()
    traffic_manager.set_global_distance_to_leading_vehicle(2.5)
    traffic_manager.set_synchronous_mode(True)

    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = TICK_DELTA
    world.apply_settings(settings)

    carla_map = world.get_map()
    actors = {}
    traffic_ids = []

    if args.record:
        os.makedirs(OUTPUT_DIR, exist_ok=True)
        start_recording(client, os.path.join(OUTPUT_DIR, 'scenario_4.log'))

    try:
        actors = setup(world, client, traffic_manager, args)
        ego = actors['ego']
        traffic_ids = actors.get('traffic_ids', [])

        speed_limit_ms = ego.get_speed_limit() / 3.6

        planner = HierarchicalPlanner(carla_map)
        executor = PathExecutor(dt=TICK_DELTA)
        logger = RobustnessLogger("Scenario 4 — Dense Highway Traffic, Forced P2 Ranking")

        print(f"\n[S4] Starting planning loop "
              f"(replan every {REPLAN_INTERVAL} ticks = {REPLAN_INTERVAL*TICK_DELTA:.2f}s)\n")

        best = None
        for tick in range(RUN_TICKS):
            world.tick()
            t = tick * TICK_DELTA

            if tick % REPLAN_INTERVAL == 0:
                best, candidates = planner.plan(ego, world, None, speed_limit_ms)
                for i in range(len(best.waypoints) - 1):
                    world.debug.draw_line(best.waypoints[i], best.waypoints[i + 1],
                                           thickness=0.2, color=carla.Color(0, 0, 255),
                                           life_time=REPLAN_INTERVAL * TICK_DELTA)
                executor.set_trajectory(best)

            control = executor.step(ego)
            ego.apply_control(control)
            top_down_follow(world, ego, height=35.0)
            logger.record(tick, t, best)

            if tick % 20 == 0:
                loc = ego.get_location()
                vel = ego.get_velocity()
                speed = math.hypot(vel.x, vel.y) * 3.6
                print(f"  t={t:5.1f}s | pos=({loc.x:6.1f},{loc.y:6.1f}) | "
                      f"speed={speed:5.1f} km/h | "
                      f"ρ_P0={best.rho_p0:+.2f} ρ_P1={best.rho_p1:+.2f} "
                      f"η_P2={best.eta_p2:.3f} | α={best.speed_factor:.1f}")

        logger.report()

    finally:
        print("\n[S4] Cleaning up...")
        for actor in actors.values():
            if isinstance(actor, list):
                continue
            if actor and hasattr(actor, 'is_alive') and actor.is_alive:
                actor.destroy()
        if traffic_ids:
            client.apply_batch([carla.command.DestroyActor(x) for x in traffic_ids])
        if args.record:
            stop_recording(client)
        settings = world.get_settings()
        settings.synchronous_mode = False
        world.apply_settings(settings)
        print("[S4] Done.")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--record', action='store_true',
                         help='record the run to output/scenario_4_highway_traffic/scenario_4.log')
    run(parser.parse_args())
