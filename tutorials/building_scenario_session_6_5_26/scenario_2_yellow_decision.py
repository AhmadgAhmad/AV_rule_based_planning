"""
Scenario 2: Yellow Light — Decision Point
==========================================
Purpose : Test stop-or-go logic. At t=0 the light turns yellow.
          Planner must decide to stop or proceed based on speed and distance.
Expected: ρ(φ₂) > 0 if correct decision made; red light never violated.

TWTL Formula:
    φ₂ = (d > d_stop → [ H₃ π_stopped ][0,5])
       ∧ (d ≤ d_stop → [ H₁ π_through ][0,4])
       ∧ H_T ¬π_red_violation

Author  : Ahmad Ahmad  |  For: Nidhi's Autonomous Driving Course
"""

import carla
import math
import os

# ─── CONFIG ──────────────────────────────────────────────────────────────────

TOWN         = 'Town05'
HOST         = 'localhost'
PORT         = 2000
TIMEOUT      = 20.0
TICK_DELTA   = 0.05
RUN_TICKS    = 400       # 20 seconds total
OUTPUT_DIR   = 'output/scenario_2'

EGO_X        =  0.0
EGO_Y        = -25.0     # 25m before stop line — inside dilemma zone
EGO_YAW      =  90.0
EGO_SPEED_KMH = 30.0    # initial speed to set

STOP_LINE_Y  = -3.0      # y coordinate of the stop line
DECELERATION = 3.0       # comfortable braking m/s²

YELLOW_DURATION = 3.0    # seconds of yellow before red


# ─── DECISION LOGIC ──────────────────────────────────────────────────────────

def compute_stop_or_go(ego, stop_line_y=STOP_LINE_Y, decel=DECELERATION):
    """
    Decide: should ego stop or proceed through yellow?

    Returns:
        decision        : 'STOP' or 'GO'
        stopping_dist   : distance needed to stop at current speed
        dist_to_line    : current distance to stop line
    """
    velocity = ego.get_velocity()
    speed_ms = math.sqrt(velocity.x**2 + velocity.y**2)
    location = ego.get_location()

    dist_to_line   = abs(stop_line_y - location.y)
    stopping_dist  = (speed_ms**2) / (2 * decel)

    # If we CANNOT stop before the line → safest action is to GO through
    decision = 'GO' if stopping_dist > dist_to_line else 'STOP'

    print(f"[S2] speed={speed_ms*3.6:.1f} km/h | "
          f"dist_to_line={dist_to_line:.1f}m | "
          f"stopping_dist={stopping_dist:.1f}m | "
          f"→ {decision}")

    return decision, stopping_dist, dist_to_line


# ─── TWTL MONITORING ─────────────────────────────────────────────────────────

def compute_yellow_robustness(trajectory):
    """
    Evaluate φ₂ over the recorded trajectory.

    trajectory : list of dicts with keys:
                   location, speed_ms, light_state ('green'/'yellow'/'red')

    Returns ρ(φ₂) — positive = passed, negative = violated.
    """
    red_violation_values = []

    for state in trajectory:
        loc         = state['location']
        light_state = state['light_state']
        in_intersec = abs(loc.y) < 10.0    # within 10m of intersection center

        # Rule: never enter intersection while RED
        if light_state == 'red' and in_intersec:
            red_violation_values.append(-1.0)
        else:
            red_violation_values.append(1.0)

    rho = min(red_violation_values) if red_violation_values else 0.0

    print(f"\n{'='*50}")
    print(f"  TWTL ROBUSTNESS REPORT — Scenario 2")
    print(f"{'='*50}")
    print(f"  ρ(¬red_violation) = {rho:+.3f}")
    print(f"  Result            : {'PASSED ✓' if rho > 0 else 'FAILED ✗'}")
    print(f"{'='*50}\n")

    return rho


# ─── SETUP & RUN ─────────────────────────────────────────────────────────────

def run_scenario_2():
    client = carla.Client(HOST, PORT)
    client.set_timeout(TIMEOUT)

    print(f"[S2] Loading {TOWN}...")
    world  = client.load_world(TOWN)
    world.set_weather(carla.WeatherParameters.ClearNoon)

    settings = world.get_settings()
    settings.synchronous_mode    = True
    settings.fixed_delta_seconds = TICK_DELTA
    world.apply_settings(settings)

    bp_lib     = world.get_blueprint_library()
    carla_map  = world.get_map()
    actors     = {}
    trajectory = []

    try:
        # Spawn ego
        ego_bp = bp_lib.find('vehicle.tesla.model3')
        ego_bp.set_attribute('color', '255,165,0')   # orange for yellow scenario
        target_loc    = carla.Location(x=EGO_X, y=EGO_Y, z=0.5)
        spawn_points  = carla_map.get_spawn_points()
        nearest_spawn = min(spawn_points, key=lambda sp: sp.location.distance(target_loc))
        ego           = world.spawn_actor(ego_bp, nearest_spawn)
        actors['ego'] = ego
        print(f"[S2] Ego spawned at {ego.get_location()}")

        # Find traffic light
        lights = world.get_actors().filter('traffic.traffic_light')
        light  = min(lights, key=lambda l: l.get_location().distance(ego.get_location()),
                     default=None)
        if light:
            # Start with GREEN, will switch to YELLOW at run start
            light.set_state(carla.TrafficLightState.Green)
            light.freeze(True)
            actors['light'] = light
            print(f"[S2] Light {light.id} ready (will turn YELLOW at t=0)")

        # Attach camera
        os.makedirs(OUTPUT_DIR, exist_ok=True)
        cam_bp = bp_lib.find('sensor.camera.rgb')
        cam_bp.set_attribute('image_size_x', '1280')
        cam_bp.set_attribute('image_size_y', '720')
        cam_transform = carla.Transform(carla.Location(z=25.0), carla.Rotation(pitch=-90.0))
        camera = world.spawn_actor(cam_bp, cam_transform, attach_to=ego)
        camera.listen(lambda img: img.save_to_disk(f'{OUTPUT_DIR}/frame_{img.frame:06d}.png'))
        actors['camera'] = camera

        # Enable autopilot (replace with your planner)
        ego.set_autopilot(True)

        # ── Main loop ───────────────────────────────────────────────────────
        yellow_triggered = False
        red_triggered    = False

        for tick in range(RUN_TICKS):
            world.tick()
            t = tick * TICK_DELTA

            loc = ego.get_location()
            vel = ego.get_velocity()
            speed_ms = math.sqrt(vel.x**2 + vel.y**2)

            # t=0: switch to YELLOW and log decision
            if not yellow_triggered and t >= 0.1:
                if light:
                    light.set_state(carla.TrafficLightState.Yellow)
                    light.freeze(True)
                yellow_triggered = True
                decision, sd, dl = compute_stop_or_go(ego)
                print(f"[S2] t={t:.1f}s → YELLOW! Decision: {decision}")

            # t=3s: switch to RED
            if not red_triggered and t >= YELLOW_DURATION:
                if light:
                    light.set_state(carla.TrafficLightState.Red)
                    light.freeze(True)
                red_triggered = True
                print(f"[S2] t={t:.1f}s → RED")

            # Determine current light state string
            if light:
                state_enum = light.get_state()
                light_str  = {
                    carla.TrafficLightState.Green:  'green',
                    carla.TrafficLightState.Yellow: 'yellow',
                    carla.TrafficLightState.Red:    'red',
                }.get(state_enum, 'unknown')
            else:
                light_str = 'unknown'

            trajectory.append({
                'time':        t,
                'location':    loc,
                'speed_ms':    speed_ms,
                'light_state': light_str,
            })

            if tick % 20 == 0:
                print(f"  t={t:5.1f}s | pos=({loc.x:6.1f},{loc.y:6.1f}) | "
                      f"speed={speed_ms*3.6:5.1f} km/h | light={light_str}")

        compute_yellow_robustness(trajectory)

    finally:
        for actor in actors.values():
            if actor and actor.is_alive:
                actor.destroy()
        settings = world.get_settings()
        settings.synchronous_mode = False
        world.apply_settings(settings)
        print("[S2] Done.")


if __name__ == '__main__':
    run_scenario_2()
