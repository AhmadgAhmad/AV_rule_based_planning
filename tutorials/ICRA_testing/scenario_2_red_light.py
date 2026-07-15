"""
scenario_2_red_light.py — Red Light, Forced Legal Filtering
================================================================
Exercises Stage 2 (P1) of the hierarchical planner.

Setup:
    Ego approaches an intersection with the nearest light frozen RED.
    Several of the SamplingTrajectoryGenerator's candidate trajectories
    (the higher speed_factor ones) are geometrically fine and P0-safe —
    nothing is in the way — but they cross into the junction while the
    light is red, so TWTLEvaluator._evaluate_p1() scores them rho_p1 < 0.
    Stage 2 removes them regardless of how efficient or comfortable they
    looked under P2. Only the trajectories that stop short of the junction
    survive to be ranked.

TWTL Formula for Scenario 2:
    φ₂ = H_T ¬π_red_entry  ∧  H_T ¬π_collision  ∧  [H₂ π_stopped][t_arrive, T]

    i.e. "never enter the junction while red, never collide, and once you've
    arrived at the stop line stay stopped until T."

What to watch in the console:
    [Planner] Stage 2 (P1): N → M legal trajectories
    should show candidates being dropped as the ego nears the light. If
    every candidate looks legal, back SPAWN_INDEX further from the light
    (below) so at least one sampled trajectory reaches the junction.

Author: Ahmad Ahmad | For: Nidhi's Autonomous Driving Course
"""

import argparse
import math
import os
import sys

import carla

sys.path.insert(0, os.path.dirname(__file__))
from planner import HierarchicalPlanner, PathExecutor, RobustnessLogger
from scenario_utils import (debug_weather, attach_overhead_camera, top_down_follow,
                             start_recording, stop_recording, spawn_ego, nearest_light)

# ─── CONFIG ──────────────────────────────────────────────────────────────────

TOWN            = 'Town05'
HOST            = 'localhost'
PORT            = 2000
TIMEOUT         = 20.0
TICK_DELTA      = 0.05
RUN_TICKS       = 400          # 20 seconds — long enough to reach the light and hold
REPLAN_INTERVAL = 10
OUTPUT_DIR      = 'output/scenario_2_red_light'
SPAWN_INDEX     = 12           # same start point as scenario 1; light freezes RED instead


def setup(world, client, args):
    carla_map = world.get_map()
    actors = {}

    ego = spawn_ego(world, carla_map)
    actors['ego'] = ego
    print(f"[S2] Ego spawned at {ego.get_location()}")

    world.get_spectator().set_transform(carla.Transform(
        ego.get_location() + carla.Location(z=20), carla.Rotation(pitch=-90)))
    world.tick()

    world.debug.draw_string(ego.get_location() + carla.Location(z=3), "EGO CAR HERE",
                             draw_shadow=False, color=carla.Color(255, 0, 0),
                             life_time=20.0, persistent_lines=True)

    light = nearest_light(world, ego)
    if light:
        dist = light.get_location().distance(ego.get_location())
        print(f"[S2] Nearest light {light.id} is {dist:.1f}m away")
        light.set_state(carla.TrafficLightState.Red)
        light.freeze(True)
        actors['light'] = light
        print(f"[S2] Light {light.id} → RED (frozen)")
    else:
        print("[S2] WARNING: no traffic light found near ego — "
              "P1 red-light filtering won't be exercised. Try a different SPAWN_INDEX.")

    actors['camera'] = attach_overhead_camera(world, ego, OUTPUT_DIR)
    return actors


def run(args):
    client = carla.Client(HOST, PORT)
    client.set_timeout(TIMEOUT)

    print(f"[S2] Loading {TOWN}...")
    world = client.load_world(TOWN)
    world.set_weather(debug_weather())

    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = TICK_DELTA
    world.apply_settings(settings)

    carla_map = world.get_map()
    actors = {}

    if args.record:
        os.makedirs(OUTPUT_DIR, exist_ok=True)
        start_recording(client, os.path.join(OUTPUT_DIR, 'scenario_2.log'))

    try:
        actors = setup(world, client, args)
        ego = actors['ego']
        light = actors.get('light')

        speed_limit_ms = ego.get_speed_limit() / 3.6

        planner = HierarchicalPlanner(carla_map)
        executor = PathExecutor(dt=TICK_DELTA)
        logger = RobustnessLogger("Scenario 2 — Red Light, Forced Legal Filtering")

        print(f"\n[S2] Starting planning loop "
              f"(replan every {REPLAN_INTERVAL} ticks = {REPLAN_INTERVAL*TICK_DELTA:.2f}s)\n")

        best = None
        for tick in range(RUN_TICKS):
            world.tick()
            t = tick * TICK_DELTA

            if tick % REPLAN_INTERVAL == 0:
                best, candidates = planner.plan(ego, world, light, speed_limit_ms)
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
                wp = carla_map.get_waypoint(loc)
                junc = "⚡ IN JUNCTION" if wp.is_junction else ""
                print(f"  t={t:5.1f}s | pos=({loc.x:6.1f},{loc.y:6.1f}) | "
                      f"speed={speed:5.1f} km/h | "
                      f"ρ_P0={best.rho_p0:+.2f} ρ_P1={best.rho_p1:+.2f} "
                      f"η_P2={best.eta_p2:.3f} {junc}")

        logger.report()

    finally:
        print("\n[S2] Cleaning up...")
        for actor in actors.values():
            if actor and actor.is_alive:
                actor.destroy()
        if args.record:
            stop_recording(client)
        settings = world.get_settings()
        settings.synchronous_mode = False
        world.apply_settings(settings)
        print("[S2] Done.")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--record', action='store_true',
                         help='record the run to output/scenario_2_red_light/scenario_2.log '
                              '(replay with CARLA\'s start_replaying.py)')
    run(parser.parse_args())
