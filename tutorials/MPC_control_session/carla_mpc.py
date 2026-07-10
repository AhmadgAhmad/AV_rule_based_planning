"""
carla_mpc.py — MPC + CARLA (v4: single-point frame conversion)
==============================================================
THE fix: CARLA yaw is clockwise-positive (left-handed).
Bicycle model assumes counter-clockwise-positive (right-handed).

Conversion at ONE place only:
  get_state()   → negate psi before giving to MPC
  to_carla_cmd()→ negate delta before giving to CARLA
  build_ref()   → negate psi in reference arrays

Everything else is untouched.
"""

import carla
import numpy as np
import time
import math
import sys

sys.path.insert(0, '/home/ahmad/Desktop/RuleBookDriving/tutorials/MPC_control_session')
from mpc_osqp import build_qp, get_ref_window

DELTA_MAX = 0.45
A_MAX     =  3.0
A_MIN     = -5.0
DT        =  0.1
V_REF     =  3.0    # NOTE: must match V_REF in mpc_osqp.py

# ── Connect ──────────────────────────────────────────────────────
print("[1] Connecting...")
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world  = client.get_world()
bp_lib = world.get_blueprint_library()
print(f"    Map: {world.get_map().name}")

settings = world.get_settings()
settings.synchronous_mode    = True
settings.fixed_delta_seconds = DT
world.apply_settings(settings)

# ── Spawn ────────────────────────────────────────────────────────
print("[2] Spawning...")
bp       = bp_lib.find('vehicle.tesla.model3')
spawn_pt = world.get_map().get_spawn_points()[0]
vehicle  = world.spawn_actor(bp, spawn_pt)
vehicle.set_autopilot(False)
for _ in range(3):
    world.tick()

tf = vehicle.get_transform()
print(f"    ID={vehicle.id}  "
      f"pos=({tf.location.x:.1f}, {tf.location.y:.1f})  "
      f"CARLA yaw={tf.rotation.yaw:.1f}°")

# ── Spectator ────────────────────────────────────────────────────
spectator = world.get_spectator()
def update_spectator(v, h=28, d=12):
    tf  = v.get_transform()
    yaw = tf.rotation.yaw                         # CARLA degrees
    spectator.set_transform(carla.Transform(
        carla.Location(
            x=tf.location.x - d*math.cos(math.radians(yaw)),
            y=tf.location.y - d*math.sin(math.radians(yaw)),
            z=tf.location.z + h),
        carla.Rotation(pitch=-65, yaw=yaw)))
update_spectator(vehicle)

# ── Reference path ───────────────────────────────────────────────
def build_ref(world, vehicle, n_pts=200, spacing=2.0):
    """
    Build reference path from vehicle's current waypoint.
    Follows the lane FORWARD (same direction vehicle is facing).
    Draws the path as green spheres in CARLA so you can see it.
    """
    cmap   = world.get_map()
    debug  = world.debug
    veh_tf = vehicle.get_transform()

    wp = cmap.get_waypoint(
        veh_tf.location,
        project_to_road=True,
        lane_type=carla.LaneType.Driving)

    # ── Check we're going FORWARD not backward ────────────────
    # Dot product of vehicle heading vs waypoint heading:
    # if negative, waypoints go opposite direction → use previous direction
    veh_yaw = math.radians(veh_tf.rotation.yaw)
    wp_yaw  = math.radians(wp.transform.rotation.yaw)
    dot = math.cos(veh_yaw)*math.cos(wp_yaw) + math.sin(veh_yaw)*math.sin(wp_yaw)
    if dot < 0:
        print("    [REF] Waypoint direction opposite to vehicle — flipping")
        # Try the other lane direction
        alt = wp.get_left_lane() or wp.get_right_lane()
        if alt:
            wp = alt

    xs, ys, psis = [], [], []
    prev_wp = None
    for _ in range(n_pts):
        loc = wp.transform.location
        xs.append(loc.x)
        ys.append(loc.y)
        psis.append(math.radians(wp.transform.rotation.yaw))   # CARLA frame, as-is

        # Draw green sphere at every 5th waypoint
        if len(xs) % 5 == 1:
            debug.draw_point(
                carla.Location(x=loc.x, y=loc.y, z=loc.z + 0.5),
                size=0.08,
                color=carla.Color(0, 255, 0),   # green
                life_time=30.0,
                persistent_lines=True
            )

        # Draw direction arrow at every 20th waypoint
        if len(xs) % 20 == 1:
            wp_yaw_rad = math.radians(wp.transform.rotation.yaw)
            end = carla.Location(
                x=loc.x + 2.0*math.cos(wp_yaw_rad),
                y=loc.y + 2.0*math.sin(wp_yaw_rad),
                z=loc.z + 0.5
            )
            debug.draw_arrow(
                carla.Location(x=loc.x, y=loc.y, z=loc.z+0.5),
                end,
                thickness=0.1,
                arrow_size=0.3,
                color=carla.Color(0, 200, 255),  # cyan
                life_time=30.0
            )

        nxt = wp.next(spacing)
        if not nxt:
            break
        prev_wp = wp
        wp = nxt[0]

    RX   = np.array(xs)
    RY   = np.array(ys)
    RPSI = np.array(psis)
    RV   = np.full(len(xs), V_REF)

    vpos = veh_tf.location
    d0   = np.hypot(vpos.x - RX[0], vpos.y - RY[0])
    veh_yaw_deg = veh_tf.rotation.yaw
    ref_yaw_deg = math.degrees(RPSI[0])   # CARLA degrees, no negation

    print(f"[3] Ref: {len(xs)} pts  d0={d0:.2f}m")
    print(f"    Vehicle heading : {veh_yaw_deg:.1f}°")
    print(f"    Ref[0]  heading : {ref_yaw_deg:.1f}°  (dot={dot:.2f})")
    print(f"    Ref start: ({RX[0]:.1f}, {RY[0]:.1f})")
    print(f"    Ref end:   ({RX[-1]:.1f}, {RY[-1]:.1f})")
    print(f"    → GREEN dots + CYAN arrows drawn in CARLA viewport")

    # Mark start with a big red sphere
    debug.draw_point(
        carla.Location(x=float(RX[0]), y=float(RY[0]), z=vpos.z+1.0),
        size=0.25,
        color=carla.Color(255, 0, 0),   # red = start
        life_time=30.0
    )
    # Mark end with a big blue sphere
    debug.draw_point(
        carla.Location(x=float(RX[-1]), y=float(RY[-1]), z=vpos.z+1.0),
        size=0.25,
        color=carla.Color(0, 0, 255),   # blue = end
        life_time=30.0
    )

    world.tick()   # flush debug draws to viewport
    return RX, RY, RPSI, RV

RX, RY, RPSI, RV = build_ref(world, vehicle)


# ── State: negate psi for MPC ────────────────────────────────────
def get_state(vehicle) -> np.ndarray:
    """CARLA → MPC state [X, Y, psi, v] — all in CARLA's native frame."""
    tf  = vehicle.get_transform()
    vel = vehicle.get_velocity()
    psi = math.radians(tf.rotation.yaw)   # as-is, no negation
    v   = math.sqrt(vel.x**2 + vel.y**2)
    return np.array([tf.location.x, tf.location.y, psi, v])


# ── Control: negate delta back to CARLA ──────────────────────────
def to_carla_cmd(u0: np.ndarray) -> carla.VehicleControl:
    """
    MPC output → CARLA command. No sign flip: model is now in CARLA frame.
    Positive delta = clockwise yaw = right turn = positive CARLA steer. Consistent.
    """
    delta, a = u0
    cmd = carla.VehicleControl()
    cmd.steer = float(np.clip(delta / DELTA_MAX, -1.0, 1.0))
    if a >= 0:
        cmd.throttle = float(np.clip(a / A_MAX, 0.0, 1.0))
        cmd.brake    = 0.0
    else:
        cmd.throttle = 0.0
        cmd.brake    = float(np.clip(-a / abs(A_MIN), 0.0, 1.0))
    cmd.hand_brake        = False
    cmd.manual_gear_shift = False
    return cmd


# ── Kick-start ───────────────────────────────────────────────────
print("[4] Kick-starting (gentle)...")
# Use a very small throttle — just enough to overcome static friction
# High throttle here puts us at 5+ m/s before MPC takes over
for _ in range(2):
    vehicle.apply_control(carla.VehicleControl(throttle=0.25, steer=0.0))
    world.tick()

# ── Main loop ────────────────────────────────────────────────────
print("\n── MPC loop running (Ctrl-C to stop) ──\n")

u_prev   = np.zeros(2)
step     = 0
err_hist = []

try:
    while True:
        world.tick()
        step += 1

        x = get_state(vehicle)
        ref_x, ref_y, ref_psi, ref_v = get_ref_window(x, RX, RY, RPSI, RV)

        t0 = time.perf_counter()
        u0, _, _ = build_qp(x, ref_x, ref_y, ref_psi, ref_v, u_prev)
        dt_ms = (time.perf_counter() - t0) * 1000

        cmd = to_carla_cmd(u0)
        vehicle.apply_control(cmd)
        u_prev = u0.copy()

        update_spectator(vehicle)

        err = np.hypot(x[0] - ref_x[0], x[1] - ref_y[0])
        err_hist.append(err)

        # ── Live debug drawing in CARLA viewport ──────────────
        debug = world.debug
        veh_loc = vehicle.get_transform().location

        # 1. MPC predicted horizon: yellow dots
        #    X_pred not available here — draw ref window instead
        for i in range(0, len(ref_x), 2):
            debug.draw_point(
                carla.Location(x=float(ref_x[i]),
                               y=float(ref_y[i]),
                               z=veh_loc.z + 0.3),
                size=0.06,
                color=carla.Color(255, 220, 0),   # yellow
                life_time=0.15   # short: refreshes each step
            )

        # 2. Immediate target (closest ref point): magenta sphere
        debug.draw_point(
            carla.Location(x=float(ref_x[0]),
                           y=float(ref_y[0]),
                           z=veh_loc.z + 0.8),
            size=0.18,
            color=carla.Color(255, 0, 200),   # magenta
            life_time=0.15
        )

        # 3. Steering arrow from vehicle showing applied delta
        yaw_carla = vehicle.get_transform().rotation.yaw
        delta_carla_deg = yaw_carla + math.degrees(-u0[0])  # arrow direction
        arrow_len = 4.0 + abs(u0[0]) * 8.0   # longer = more steer
        arrow_end = carla.Location(
            x=veh_loc.x + arrow_len * math.cos(math.radians(delta_carla_deg)),
            y=veh_loc.y + arrow_len * math.sin(math.radians(delta_carla_deg)),
            z=veh_loc.z + 1.2
        )
        # Color: green if small steer, red if saturated
        sat_ratio = abs(u0[0]) / DELTA_MAX
        arrow_color = carla.Color(
            int(255 * sat_ratio),        # R: high when saturated
            int(255 * (1-sat_ratio)),    # G: high when small
            0
        )
        debug.draw_arrow(
            carla.Location(x=veh_loc.x, y=veh_loc.y, z=veh_loc.z+1.2),
            arrow_end,
            thickness=0.12,
            arrow_size=0.4,
            color=arrow_color,
            life_time=0.15
        )

        # 4. HUD text above vehicle
        psi_err = x[2] - ref_psi[0]
        psi_err = (psi_err + np.pi) % (2*np.pi) - np.pi
        psi_err_deg = math.degrees(psi_err)
        hud_text = (f"step:{step} err:{err:.2f}m v:{x[3]:.1f}m/s "
                    f"d:{u0[0]:+.3f} a:{u0[1]:+.2f} yD:{psi_err_deg:+.1f}")
        debug.draw_string(
            carla.Location(x=veh_loc.x - 2, y=veh_loc.y, z=veh_loc.z + 5),
            hud_text,
            color=carla.Color(255, 255, 255),
            life_time=0.15
        )

        sat = abs(u0[0]) >= DELTA_MAX * 0.98
        psi_err = x[2] - ref_psi[0]
        psi_err = (psi_err + np.pi) % (2*np.pi) - np.pi
        print(f"step {step:4d} | "
              f"pos=({x[0]:6.1f},{x[1]:6.1f}) | "
              f"v={x[3]:.2f} | "
              f"d={u0[0]:+.3f}{'SAT' if sat else '   '} "
              f"a={u0[1]:+.2f} | "
              f"err={err:.2f}m "
              f"yD={math.degrees(psi_err):+.1f}° | "
              f"{dt_ms:.1f}ms")

finally:
    print(f"\n── Stopped at step {step} ──")
    if err_hist:
        print(f"   Mean err={np.mean(err_hist):.2f}m  "
              f"Max err={np.max(err_hist):.2f}m")
    settings.synchronous_mode = False
    world.apply_settings(settings)
    vehicle.destroy()
    print("   Cleaned up.")
