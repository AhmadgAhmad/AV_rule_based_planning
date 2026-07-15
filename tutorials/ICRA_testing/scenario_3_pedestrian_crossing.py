"""
scenario_3_pedestrian_crossing.py — Jaywalkers, Forced Safety Filtering
===========================================================================
Exercises Stage 1 (P0) of the hierarchical planner.

Setup:
    A handful of pedestrians spawn beside the road just ahead of the ego
    and are commanded to walk straight across it (same walker-spawn +
    AI-controller pattern as CARLA's generate_traffic.py, simplified down
    to a fixed number of walkers on a fixed crossing path instead of
    random navigation points). This puts moving obstacles directly on top
    of several of SamplingTrajectoryGenerator's candidate trajectories.

    TWTLEvaluator._evaluate_p0() measures distance from every waypoint to
    every pedestrian; any trajectory that comes within PED_SAFETY_MARGIN
    (2.5 m) scores rho_p0 < 0 and Stage 1 removes it — before P1 or P2 ever
    get a vote. The survivors are the ones that slow down, hang back, or
    shift laterally away from the crossing.

TWTL Formula for Scenario 3:
    φ₃ = H_T ¬π_ped_collision  ∧  [H₂ π_yield_zone][t_ped_visible, t_ped_clear]

    i.e. "never collide with a pedestrian, and while the crossing is
    occupied, stay in the yield zone (reduced speed / held position)."

What to watch in the console:
    [Planner] Stage 1 (P0): N → M safe trajectories
    should shrink sharply while the walkers are mid-crossing, then recover
    once they reach the far sidewalk.

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

TOWN            = 'Town05'
HOST            = 'localhost'
PORT            = 2000
TIMEOUT         = 20.0
TICK_DELTA      = 0.05
RUN_TICKS       = 350
REPLAN_INTERVAL = 10
OUTPUT_DIR      = 'output/scenario_3_pedestrian_crossing'
SPAWN_INDEX     = 12
NUM_WALKERS     = 3
CROSS_AHEAD_M   = 18.0     # how far ahead of the ego the crossing point is
CROSS_WIDTH_M   = 10.0     # lateral span of the crossing (perpendicular to road)
WALKER_SPEED    = 1.3      # m/s, brisk walking pace


def spawn_crossing_walkers(world, carla_map, ego, client):
    """Spawn NUM_WALKERS beside the road ahead of the ego and send them
    walking straight across it. Mirrors generate_traffic.py's two-step
    walker + AI-controller spawn, but with an explicit crossing target
    instead of a random navigation point."""
    bp_lib = world.get_blueprint_library()
    walker_bps = bp_lib.filter('walker.pedestrian.*')

    ego_wp = carla_map.get_waypoint(ego.get_location())
    fwd = ego_wp.transform.get_forward_vector()
    right = ego_wp.transform.get_right_vector()
    crossing_center = ego.get_location() + carla.Location(
        x=fwd.x * CROSS_AHEAD_M, y=fwd.y * CROSS_AHEAD_M, z=0.5)

    SpawnActor = carla.command.SpawnActor
    batch, walker_speed = [], []
    for i in range(NUM_WALKERS):
        offset = (i - (NUM_WALKERS - 1) / 2.0) * 2.0   # spread walkers along the crossing
        start_loc = crossing_center + carla.Location(
            x=-right.x * (CROSS_WIDTH_M / 2) + fwd.x * offset,
            y=-right.y * (CROSS_WIDTH_M / 2) + fwd.y * offset,
        )
        walker_bp = random.choice(walker_bps)
        if walker_bp.has_attribute('is_invincible'):
            walker_bp.set_attribute('is_invincible', 'false')
        batch.append(SpawnActor(walker_bp, carla.Transform(start_loc)))
        walker_speed.append(WALKER_SPEED)

    results = client.apply_batch_sync(batch, True)
    walkers = []
    for res, spd in zip(results, walker_speed):
        if res.error:
            print(f"[S3] Walker spawn error: {res.error}")
            continue
        walkers.append({'id': res.actor_id, 'speed': spd})

    # Spawn AI walker controllers
    controller_bp = bp_lib.find('controller.ai.walker')
    batch = [SpawnActor(controller_bp, carla.Transform(), w['id']) for w in walkers]
    results = client.apply_batch_sync(batch, True)
    for w, res in zip(walkers, results):
        if res.error:
            print(f"[S3] Controller spawn error: {res.error}")
            continue
        w['con'] = res.actor_id

    world.tick()

    all_ids = [i for w in walkers for i in (w.get('con'), w['id']) if i is not None]
    all_actors = world.get_actors(all_ids)
    actors_by_id = {a.id: a for a in all_actors}

    # Target = straight across the road from each walker's start point
    for w in walkers:
        if 'con' not in w:
            continue
        controller = actors_by_id[w['con']]
        walker_actor = actors_by_id[w['id']]
        start_loc = walker_actor.get_location()
        target_loc = crossing_center + carla.Location(
            x=right.x * (CROSS_WIDTH_M / 2), y=right.y * (CROSS_WIDTH_M / 2))
        controller.start()
        controller.go_to_location(target_loc)
        controller.set_max_speed(w['speed'])

    print(f"[S3] Spawned {len(walkers)} walkers crossing {CROSS_AHEAD_M:.0f}m ahead of ego")
    return walkers, all_ids


def setup(world, client, args):
    carla_map = world.get_map()
    actors = {}

    ego = spawn_ego(world, carla_map)
    actors['ego'] = ego
    print(f"[S3] Ego spawned at {ego.get_location()}")

    world.get_spectator().set_transform(carla.Transform(
        ego.get_location() + carla.Location(z=20), carla.Rotation(pitch=-90)))
    world.tick()

    world.debug.draw_string(ego.get_location() + carla.Location(z=3), "EGO CAR HERE",
                             draw_shadow=False, color=carla.Color(255, 0, 0),
                             life_time=20.0, persistent_lines=True)

    walkers, walker_ids = spawn_crossing_walkers(world, carla_map, ego, client)
    actors['walker_ids'] = walker_ids

    actors['camera'] = attach_overhead_camera(world, ego, OUTPUT_DIR)
    return actors


def run(args):
    client = carla.Client(HOST, PORT)
    client.set_timeout(TIMEOUT)

    print(f"[S3] Loading {TOWN}...")
    world = client.load_world(TOWN)
    world.set_weather(debug_weather())

    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = TICK_DELTA
    world.apply_settings(settings)

    carla_map = world.get_map()
    actors = {}
    walker_ids = []

    if args.record:
        os.makedirs(OUTPUT_DIR, exist_ok=True)
        start_recording(client, os.path.join(OUTPUT_DIR, 'scenario_3.log'))

    try:
        actors = setup(world, client, args)
        ego = actors['ego']
        walker_ids = actors.get('walker_ids', [])

        speed_limit_ms = ego.get_speed_limit() / 3.6

        planner = HierarchicalPlanner(carla_map)
        executor = PathExecutor(dt=TICK_DELTA)
        logger = RobustnessLogger("Scenario 3 — Pedestrian Crossing, Forced Safety Filtering")

        print(f"\n[S3] Starting planning loop "
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
            top_down_follow(world, ego)
            logger.record(tick, t, best)

            if tick % 20 == 0:
                loc = ego.get_location()
                vel = ego.get_velocity()
                speed = math.hypot(vel.x, vel.y) * 3.6
                print(f"  t={t:5.1f}s | pos=({loc.x:6.1f},{loc.y:6.1f}) | "
                      f"speed={speed:5.1f} km/h | "
                      f"ρ_P0={best.rho_p0:+.2f} ρ_P1={best.rho_p1:+.2f} "
                      f"η_P2={best.eta_p2:.3f}")

        logger.report()

    finally:
        print("\n[S3] Cleaning up...")
        for i in range(0, len(walker_ids), 2):
            con = world.get_actors().find(walker_ids[i]) if i < len(walker_ids) else None
            if con and con.is_alive:
                con.stop()
        for actor in actors.values():
            if isinstance(actor, list):
                continue
            if actor and hasattr(actor, 'is_alive') and actor.is_alive:
                actor.destroy()
        if walker_ids:
            client.apply_batch([carla.command.DestroyActor(x) for x in walker_ids])
        if args.record:
            stop_recording(client)
        settings = world.get_settings()
        settings.synchronous_mode = False
        world.apply_settings(settings)
        print("[S3] Done.")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--record', action='store_true',
                         help='record the run to output/scenario_3_pedestrian_crossing/scenario_3.log')
    run(parser.parse_args())
