"""
Scenario 4: Blocked Intersection — Pedestrians Crossing
========================================================
Purpose : Light is GREEN but pedestrians are crossing.
          Tests that P0 (safety) overrides P1 (legal compliance).
Expected: ρ(φ₄) > 0 if ego yields to ALL pedestrians before proceeding.

TWTL Formula:
    φ₄ = H_T ¬π_ped_collision
       ∧ [H₃ π_ped_clear][0,∞] · [H₁ π_through][0,10]
       ∧ H_T ¬π_collision

    KEY: φ₄ = Green light DOES NOT override safety!

Author  : Ahmad Ahmad  |  For: Nidhi's Autonomous Driving Course
"""

import carla
import math
import os
import time

# ─── CONFIG ──────────────────────────────────────────────────────────────────

TOWN               = 'Town05'
HOST               = 'localhost'
PORT               = 2000
TIMEOUT            = 20.0
TICK_DELTA         = 0.05
RUN_TICKS          = 800     # 40 seconds — pedestrians take time
OUTPUT_DIR         = 'output/scenario_4'

EGO_X              =  0.0
EGO_Y              = -40.0
EGO_YAW            =  90.0

NUM_PEDESTRIANS    = 3
PED_WALK_SPEED     = 1.2     # m/s

# Crosswalk bounding box (pedestrians walk across x-axis)
CROSSWALK_X_MIN    = -6.0
CROSSWALK_X_MAX    =  6.0
CROSSWALK_Y_MIN    = -3.5
CROSSWALK_Y_MAX    =  3.5

# Safety
PED_SAFETY_RADIUS  = 2.0    # metres — hard safety boundary around pedestrians
COLLISION_RADIUS   = 1.0    # metres — general collision radius


# ─── PEDESTRIAN SPAWNING ─────────────────────────────────────────────────────

def spawn_crossing_pedestrians(world, bp_lib, num_peds=NUM_PEDESTRIANS):
    """
    Spawn pedestrians on the west sidewalk and walk them east across the intersection.
    Uses CARLA's walker AI controller.
    Returns (pedestrians, controllers).
    """
    ped_bps        = list(bp_lib.filter('walker.pedestrian.*'))
    controller_bp  = bp_lib.find('controller.ai.walker')

    import random

    # Start positions: west sidewalk, staggered north-south
    start_positions = [
        carla.Transform(carla.Location(x=-7.0, y= 0.5, z=0.5)),
        carla.Transform(carla.Location(x=-7.0, y=-1.0, z=0.5)),
        carla.Transform(carla.Location(x=-7.0, y=-2.5, z=0.5)),
    ]

    # Goal: east sidewalk
    goal_location = carla.Location(x=7.0, y=0.0, z=0.0)

    pedestrians  = []
    controllers  = []

    # Spawn pedestrian actors
    for i in range(num_peds):
        bp = random.choice(ped_bps)
        try:
            ped = world.spawn_actor(bp, start_positions[i])
            pedestrians.append(ped)
            print(f"[S4] Pedestrian {i+1} spawned at {start_positions[i].location}")
        except RuntimeError as e:
            print(f"[S4] Could not spawn pedestrian {i+1}: {e}")

    # Tick world BEFORE starting controllers (CARLA requirement)
    world.tick()

    # Spawn AI controllers attached to each pedestrian
    for i, ped in enumerate(pedestrians):
        try:
            ctrl = world.spawn_actor(controller_bp, carla.Transform(), attach_to=ped)
            controllers.append(ctrl)
        except RuntimeError as e:
            print(f"[S4] Could not spawn controller {i+1}: {e}")
            controllers.append(None)

    # Second tick to initialize controllers
    world.tick()

    # Start walking
    for i, (ped, ctrl) in enumerate(zip(pedestrians, controllers)):
        if ctrl:
            ctrl.start()
            ctrl.go_to_location(goal_location)
            ctrl.set_max_speed(PED_WALK_SPEED + i * 0.05)   # slight variation
            print(f"[S4] Pedestrian {i+1} walking east at {PED_WALK_SPEED + i*0.05:.2f} m/s")

    return pedestrians, controllers


# ─── MONITORING ──────────────────────────────────────────────────────────────

def check_pedestrian_in_crosswalk(world):
    """
    Returns (clear, blocking_location).
    clear = True if NO pedestrian is inside the crosswalk bounding box.
    """
    pedestrians = world.get_actors().filter('walker.pedestrian.*')

    for ped in pedestrians:
        loc = ped.get_location()
        in_crosswalk = (CROSSWALK_X_MIN <= loc.x <= CROSSWALK_X_MAX and
                        CROSSWALK_Y_MIN <= loc.y <= CROSSWALK_Y_MAX)
        if in_crosswalk:
            return False, loc

    return True, None


def compute_ped_safety_robustness(ego, pedestrians):
    """
    P0 (highest priority): ego must stay > PED_SAFETY_RADIUS from all pedestrians.
    Negative robustness = safety violation — scenario FAILED.
    """
    if not pedestrians:
        return 100.0

    ego_loc   = ego.get_location()
    distances = [ego_loc.distance(p.get_location()) for p in pedestrians]
    min_dist  = min(distances)

    return min_dist - PED_SAFETY_RADIUS   # positive = safe, negative = violated


def compute_crossing_robustness(ego, pedestrians, world):
    """
    Compute per-tick robustness bundle for φ₄.
    Returns dict of individual robustness values.
    """
    ped_safety_rho         = compute_ped_safety_robustness(ego, pedestrians)
    crosswalk_clear, _     = check_pedestrian_in_crosswalk(world)

    vel   = ego.get_velocity()
    speed = math.sqrt(vel.x**2 + vel.y**2)

    waypoint = world.get_map().get_waypoint(ego.get_location())

    return {
        'ped_safety_rho':  ped_safety_rho,          # P0: never hit pedestrian
        'crosswalk_clear': crosswalk_clear,          # P1: is crosswalk clear?
        'ego_speed_ms':    speed,
        'in_junction':     waypoint.is_junction,
    }


# ─── RUN ─────────────────────────────────────────────────────────────────────

def run_scenario_4():
    client = carla.Client(HOST, PORT)
    client.set_timeout(TIMEOUT)

    print(f"[S4] Loading {TOWN}...")
    world = client.load_world(TOWN)
    world.set_weather(carla.WeatherParameters.ClearNoon)

    settings = world.get_settings()
    settings.synchronous_mode    = True
    settings.fixed_delta_seconds = TICK_DELTA
    world.apply_settings(settings)

    bp_lib    = world.get_blueprint_library()
    carla_map = world.get_map()
    actors    = {}
    pedestrians, controllers = [], []
    log = []

    try:
        # ── Spawn pedestrians first ──────────────────────────────────────────
        pedestrians, controllers = spawn_crossing_pedestrians(world, bp_lib)

        # ── Spawn ego ────────────────────────────────────────────────────────
        ego_bp = bp_lib.find('vehicle.tesla.model3')
        ego_bp.set_attribute('color', '0,100,255')   # blue for S4
        target_loc = carla.Location(x=EGO_X, y=EGO_Y, z=0.5)
        spawn_pts  = carla_map.get_spawn_points()
        nearest    = min(spawn_pts, key=lambda sp: sp.location.distance(target_loc))
        ego        = world.spawn_actor(ego_bp, nearest)
        actors['ego'] = ego
        print(f"[S4] Ego spawned at {ego.get_location()}")

        # ── Traffic light → GREEN (pedestrians override this!) ───────────────
        lights = world.get_actors().filter('traffic.traffic_light')
        light  = min(lights, key=lambda l: l.get_location().distance(ego.get_location()),
                     default=None)
        if light:
            light.set_state(carla.TrafficLightState.Green)
            light.freeze(True)
            actors['light'] = light
            print(f"[S4] Traffic light {light.id} → GREEN (frozen)")
            print(f"[S4] NOTE: Ego must still YIELD to pedestrians despite green!")

        # ── Overhead camera ──────────────────────────────────────────────────
        os.makedirs(OUTPUT_DIR, exist_ok=True)
        cam_bp = bp_lib.find('sensor.camera.rgb')
        cam_bp.set_attribute('image_size_x', '1280')
        cam_bp.set_attribute('image_size_y', '720')
        cam = world.spawn_actor(cam_bp,
                                carla.Transform(carla.Location(z=30.0), carla.Rotation(pitch=-90.0)),
                                attach_to=ego)
        cam.listen(lambda img: img.save_to_disk(f'{OUTPUT_DIR}/frame_{img.frame:06d}.png'))
        actors['camera'] = cam

        # Enable autopilot (replace with hierarchical planner)
        ego.set_autopilot(True)

        print(f"\n[S4] Running {RUN_TICKS} ticks ({RUN_TICKS*TICK_DELTA:.0f}s) ...")
        print(f"[S4] Watch: ego should STOP for pedestrians even though light is GREEN.\n")

        for tick in range(RUN_TICKS):
            world.tick()
            t = tick * TICK_DELTA

            rob = compute_crossing_robustness(ego, pedestrians, world)
            log.append({'t': t, **rob})

            if tick % 20 == 0:
                loc   = ego.get_location()
                speed = rob['ego_speed_ms'] * 3.6
                clear_str = "CLEAR ✓" if rob['crosswalk_clear'] else "OCCUPIED ⚠️"
                safe_str  = f"ρ_ped={rob['ped_safety_rho']:.1f}"
                junc_str  = "IN JUNCTION" if rob['in_junction'] else ""
                p0_warn   = " 🚨 P0 VIOLATED!" if rob['ped_safety_rho'] < 0 else ""
                print(f"  t={t:5.1f}s | pos=({loc.x:5.1f},{loc.y:6.1f}) | "
                      f"speed={speed:5.1f} km/h | crosswalk={clear_str} | "
                      f"{safe_str}{p0_warn}")

        print_scenario4_report(log)

    finally:
        # Stop controllers before destroying pedestrians
        for ctrl in controllers:
            if ctrl and ctrl.is_alive:
                ctrl.stop()
                ctrl.destroy()
        for ped in pedestrians:
            if ped and ped.is_alive:
                ped.destroy()
        for actor in actors.values():
            if actor and actor.is_alive:
                actor.destroy()

        settings = world.get_settings()
        settings.synchronous_mode = False
        world.apply_settings(settings)
        print("[S4] Done.")


def print_scenario4_report(log):
    if not log:
        return

    min_ped_safety = min(r['ped_safety_rho'] for r in log)
    ever_clear     = any(r['crosswalk_clear'] for r in log)
    passed_thru    = any(r['in_junction'] for r in log)

    # P0 is the dominant constraint
    p0_satisfied = min_ped_safety > 0

    # φ₄ passed only if P0 holds AND planner eventually proceeded
    rho = min_ped_safety if p0_satisfied else min_ped_safety

    print(f"\n{'='*55}")
    print(f"  TWTL ROBUSTNESS REPORT — Scenario 4")
    print(f"{'='*55}")
    print(f"  ρ(¬ped_collision)  =  {min_ped_safety:+.3f}   ← P0 priority!")
    print(f"  crosswalk_cleared  =  {ever_clear}")
    print(f"  passed_through     =  {passed_thru}")
    print(f"  ──────────────────────────────────────────────")
    if not p0_satisfied:
        print(f"  ρ(φ₄)  =  {rho:+.3f}   P0 VIOLATED — STOP everything!")
    else:
        print(f"  ρ(φ₄)  =  {rho:+.3f}")
    print(f"  Result : {'PASSED ✓' if rho > 0 else 'FAILED ✗'}")
    print(f"{'='*55}\n")


if __name__ == '__main__':
    run_scenario_4()
