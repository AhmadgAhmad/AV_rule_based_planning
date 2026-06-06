"""
Scenario 3: Red Light — Waiting Traffic
========================================
Purpose : Ego must stop behind two queued NPC vehicles at a red light.
          Tests safe following distance + red-light compliance simultaneously.
Expected: ρ(φ₃) > 0 if ego stops safely and never enters intersection during RED.

TWTL Formula:
    φ₃ = H_T ¬π_collision  ∧  H_T ¬π_red_entry  ∧  [ H₅ π_stopped ][0,30]

Author  : Ahmad Ahmad  |  For: Nidhi's Autonomous Driving Course
"""

import carla
import math
import os

# ─── CONFIG ──────────────────────────────────────────────────────────────────

TOWN              = 'Town05'
HOST              = 'localhost'
PORT              = 2000
TIMEOUT           = 20.0
TICK_DELTA        = 0.05
RUN_TICKS         = 600     # 30 seconds
OUTPUT_DIR        = 'output/scenario_3'

# Actor positions (y-axis = direction of travel toward +y)
NPC1_Y            = -3.0    # stopped at stop line
NPC2_Y            = -11.0   # 8m behind NPC1
EGO_Y             = -45.0   # 45m back from intersection

# Safety rules
STOP_SPEED_THRESH = 0.5     # m/s — below this = "stopped"
SAFETY_BUFFER     = 2.0     # metres extra beyond 2-second rule
COLLISION_RADIUS  = 1.5     # metres — hard collision boundary


# ─── SPAWN HELPERS ───────────────────────────────────────────────────────────

def spawn_npc_vehicles(world, bp_lib):
    """
    Spawn two stationary NPC vehicles queued at red light.
    Physics disabled so they stay perfectly still.
    """
    configs = [
        {
            'bp':   'vehicle.audi.a2',
            'loc':  carla.Location(x=0.0, y=NPC1_Y, z=0.5),
            'yaw':  90.0,
            'label': 'NPC1 (at stop line)',
        },
        {
            'bp':   'vehicle.nissan.micra',
            'loc':  carla.Location(x=0.0, y=NPC2_Y, z=0.5),
            'yaw':  90.0,
            'label': 'NPC2 (8m behind NPC1)',
        },
    ]

    carla_map  = world.get_map()
    spawn_pts  = carla_map.get_spawn_points()
    npcs       = []

    for cfg in configs:
        bp = bp_lib.find(cfg['bp'])
        # Find nearest valid spawn point
        nearest = min(spawn_pts, key=lambda sp: sp.location.distance(cfg['loc']))

        try:
            npc = world.spawn_actor(bp, nearest)
            npc.set_simulate_physics(False)   # freeze in place
            npcs.append(npc)
            print(f"[S3] Spawned {cfg['label']} at {npc.get_location()}")
        except RuntimeError as e:
            print(f"[S3] Could not spawn {cfg['label']}: {e}")

    return npcs


# ─── MONITORING ──────────────────────────────────────────────────────────────

def compute_safe_distance_robustness(ego, npcs):
    """
    φ_safe: ego maintains 2-second following distance from nearest NPC.
    Returns robustness (positive = safe, negative = too close).
    """
    ego_loc   = ego.get_location()
    ego_vel   = ego.get_velocity()
    ego_speed = math.sqrt(ego_vel.x**2 + ego_vel.y**2)

    # Required distance = 2s × speed + constant buffer
    required_dist = 2.0 * ego_speed + SAFETY_BUFFER

    distances = [ego_loc.distance(npc.get_location()) for npc in npcs]
    min_dist  = min(distances) if distances else 100.0

    return min_dist - required_dist, min_dist, required_dist


def compute_red_entry_robustness(ego_location, light_state):
    """
    φ_red_entry: ego must not be inside the intersection while light is RED.
    Returns +1 (safe) or -1 (violation).
    """
    in_intersection = abs(ego_location.y) < 10.0   # within 10m of center

    if light_state == carla.TrafficLightState.Red and in_intersection:
        return -1.0
    return 1.0


def compute_stopped_robustness(ego):
    """
    φ_stopped: ego's speed is below STOP_SPEED_THRESH.
    Returns signed margin (positive = stopped, negative = still moving).
    """
    vel   = ego.get_velocity()
    speed = math.sqrt(vel.x**2 + vel.y**2)
    return STOP_SPEED_THRESH - speed


# ─── RUN ─────────────────────────────────────────────────────────────────────

def run_scenario_3():
    client = carla.Client(HOST, PORT)
    client.set_timeout(TIMEOUT)

    print(f"[S3] Loading {TOWN}...")
    world = client.load_world(TOWN)
    world.set_weather(carla.WeatherParameters.ClearNoon)

    settings = world.get_settings()
    settings.synchronous_mode    = True
    settings.fixed_delta_seconds = TICK_DELTA
    world.apply_settings(settings)

    bp_lib    = world.get_blueprint_library()
    carla_map = world.get_map()
    actors    = {}
    npcs      = []
    log       = []

    try:
        # ── Spawn NPCs (stopped at red) ──────────────────────────────────────
        npcs = spawn_npc_vehicles(world, bp_lib)
        actors['npcs'] = npcs

        # ── Spawn ego ────────────────────────────────────────────────────────
        ego_bp = bp_lib.find('vehicle.tesla.model3')
        ego_bp.set_attribute('color', '255,0,0')    # red for this scenario
        target_loc = carla.Location(x=0.0, y=EGO_Y, z=0.5)
        spawn_pts  = carla_map.get_spawn_points()
        nearest    = min(spawn_pts, key=lambda sp: sp.location.distance(target_loc))
        ego        = world.spawn_actor(ego_bp, nearest)
        actors['ego'] = ego
        print(f"[S3] Ego spawned at {ego.get_location()}")

        # ── Traffic light → RED (frozen) ─────────────────────────────────────
        lights = world.get_actors().filter('traffic.traffic_light')
        light  = min(lights, key=lambda l: l.get_location().distance(ego.get_location()),
                     default=None)
        if light:
            light.set_state(carla.TrafficLightState.Red)
            light.freeze(True)
            actors['light'] = light
            print(f"[S3] Traffic light {light.id} → RED (frozen for 30s)")

        # ── Overhead camera ──────────────────────────────────────────────────
        os.makedirs(OUTPUT_DIR, exist_ok=True)
        cam_bp = bp_lib.find('sensor.camera.rgb')
        cam_bp.set_attribute('image_size_x', '1280')
        cam_bp.set_attribute('image_size_y', '720')
        cam = world.spawn_actor(cam_bp,
                                carla.Transform(carla.Location(z=25.0), carla.Rotation(pitch=-90.0)),
                                attach_to=ego)
        cam.listen(lambda img: img.save_to_disk(f'{OUTPUT_DIR}/frame_{img.frame:06d}.png'))
        actors['camera'] = cam

        # Enable autopilot (replace with hierarchical planner)
        ego.set_autopilot(True)

        print(f"\n[S3] Running {RUN_TICKS} ticks ({RUN_TICKS*TICK_DELTA:.0f}s) ...")
        print("[S3] Ego should approach and stop behind NPC queue.\n")

        for tick in range(RUN_TICKS):
            world.tick()
            t = tick * TICK_DELTA

            loc   = ego.get_location()
            state = light.get_state() if light else None

            # Compute per-tick robustness values
            safe_rho, min_dist, req_dist = compute_safe_distance_robustness(ego, npcs)
            red_rho                      = compute_red_entry_robustness(loc, state)
            stop_rho                     = compute_stopped_robustness(ego)

            log.append({
                't':         t,
                'safe_rho':  safe_rho,
                'red_rho':   red_rho,
                'stop_rho':  stop_rho,
                'min_dist':  min_dist,
                'location':  loc,
            })

            if tick % 20 == 0:
                vel   = ego.get_velocity()
                speed = math.sqrt(vel.x**2 + vel.y**2) * 3.6
                warn  = " ⚠️  RED ENTRY!" if red_rho < 0 else ""
                warn += " ⚠️  TOO CLOSE!" if safe_rho < 0 else ""
                print(f"  t={t:5.1f}s | pos=({loc.x:5.1f},{loc.y:6.1f}) | "
                      f"speed={speed:5.1f} km/h | min_dist={min_dist:.1f}m{warn}")

        print_scenario3_report(log)

    finally:
        for actor in actors.values():
            if isinstance(actor, list):
                for a in actor:
                    if a and a.is_alive:
                        a.destroy()
            elif actor and actor.is_alive:
                actor.destroy()
        settings = world.get_settings()
        settings.synchronous_mode = False
        world.apply_settings(settings)
        print("[S3] Done.")


def print_scenario3_report(log):
    if not log:
        return

    min_safe    = min(r['safe_rho'] for r in log)
    min_red     = min(r['red_rho']  for r in log)
    stopped_rho = max(r['stop_rho'] for r in log)   # best stop achieved

    # ρ(φ₃) = min of all three constraints
    rho = min(min_safe, min_red, stopped_rho)

    print(f"\n{'='*50}")
    print(f"  TWTL ROBUSTNESS REPORT — Scenario 3")
    print(f"{'='*50}")
    print(f"  ρ(¬collision)   = {min_safe:+.3f}")
    print(f"  ρ(¬red_entry)   = {min_red:+.3f}")
    print(f"  ρ(stopped)      = {stopped_rho:+.3f}")
    print(f"  ─────────────────────────────────────")
    print(f"  ρ(φ₃)  =  {rho:+.3f}")
    print(f"  Result : {'PASSED ✓' if rho > 0 else 'FAILED ✗'}")
    print(f"{'='*50}\n")


if __name__ == '__main__':
    run_scenario_3()
