"""
carla_mpc.py — MPC + CARLA (v3: proper frame conversion)
=========================================================
Root cause: CARLA is left-handed (Y right, yaw clockwise).
MPC/math is right-handed (Y left, yaw counter-clockwise).

Fix: convert EVERYTHING into one consistent frame before MPC,
convert back after. We choose to work in CARLA's frame inside
MPC by negating Y and psi when building the reference, so the
linearization is consistent with the state we feed in.

Approach: stay in CARLA frame throughout.
  - State fed to MPC:  [X, Y, psi, v]  as-is from CARLA
  - Reference built:   from CARLA waypoints, psi as-is
  - After MPC solves:  negate delta  (left-handed steer flip)
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
V_REF     =  5.0

# ═══════════════════════════════════════════════════
# CONNECT + SYNC
# ═══════════════════════════════════════════════════
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

# ═══════════════════════════════════════════════════
# SPAWN
# ═══════════════════════════════════════════════════
print("[2] Spawning...")
bp       = bp_lib.find('vehicle.tesla.model3')
spawn_pt = world.get_map().get_spawn_points()[0]
vehicle  = world.spawn_actor(bp, spawn_pt)
vehicle.set_autopilot(False)
for _ in range(3):
    world.tick()

tf  = vehicle.get_transform()
print(f"    pos=({tf.location.x:.1f}, {tf.location.y:.1f}, {tf.location.z:.1f})"
      f"  yaw={tf.rotation.yaw:.1f}°")

# ═══════════════════════════════════════════════════
# SPECTATOR
# ═══════════════════════════════════════════════════
spectator = world.get_spectator()
def update_spectator(v, h=28, d=12):
    tf  = v.get_transform()
    yaw = tf.rotation.yaw
    spectator.set_transform(carla.Transform(
        carla.Location(
            x=tf.location.x - d*math.cos(math.radians(yaw)),
            y=tf.location.y - d*math.sin(math.radians(yaw)),
            z=tf.location.z + h),
        carla.Rotation(pitch=-65, yaw=yaw)))
update_spectator(vehicle)

# ═══════════════════════════════════════════════════
# REFERENCE PATH
# ═══════════════════════════════════════════════════
def build_ref(world, vehicle, n_pts=300, spacing=2.0):
    cmap = world.get_map()
    wp   = cmap.get_waypoint(
        vehicle.get_transform().location,
        project_to_road=True,
        lane_type=carla.LaneType.Driving
    )
    xs, ys, psis = [], [], []
    for _ in range(n_pts):
        loc = wp.transform.location
        xs.append(loc.x)
        ys.append(loc.y)
        # Keep yaw in CARLA degrees → convert to radians AS-IS
        # (same frame as get_state, so MPC sees consistent geometry)
        psis.append(math.radians(wp.transform.rotation.yaw))
        nxt = wp.next(spacing)
        if not nxt: break
        wp = nxt[0]

    RX   = np.array(xs)
    RY   = np.array(ys)
    RPSI = np.array(psis)
    RV   = np.full(len(xs), V_REF)

    vpos = vehicle.get_transform().location
    d0 = np.hypot(vpos.x - RX[0], vpos.y - RY[0])
    print(f"[3] Ref: {len(xs)} pts  "
          f"start=({RX[0]:.1f},{RY[0]:.1f})  "
          f"veh=({vpos.x:.1f},{vpos.y:.1f})  "
          f"d0={d0:.2f}m")
    return RX, RY, RPSI, RV

RX, RY, RPSI, RV = build_ref(world, vehicle)

# ── Diagnostic: print first 3 ref points and vehicle heading ──
veh_psi = math.radians(vehicle.get_transform().rotation.yaw)
print(f"\n    Vehicle psi = {math.degrees(veh_psi):.1f}°  "
      f"({veh_psi:.3f} rad)")
print(f"    Ref psi[0]  = {math.degrees(RPSI[0]):.1f}°  "
      f"({RPSI[0]:.3f} rad)")
print(f"    Ref psi[1]  = {math.degrees(RPSI[1]):.1f}°")
print(f"    Ref psi[2]  = {math.degrees(RPSI[2]):.1f}°\n")

# ═══════════════════════════════════════════════════
# STATE + CONTROL CONVERSION
# ═══════════════════════════════════════════════════
def get_state(vehicle) -> np.ndarray:
    """CARLA → MPC state.
    We stay in CARLA's left-handed frame (no conversion).
    MPC geometry will be consistent because ref is also in CARLA frame.
    """
    tf  = vehicle.get_transform()
    vel = vehicle.get_velocity()
    psi = math.radians(tf.rotation.yaw)   # CARLA frame, as-is
    v   = math.sqrt(vel.x**2 + vel.y**2)
    return np.array([tf.location.x, tf.location.y, psi, v])


def to_carla_cmd(delta: float, a: float) -> carla.VehicleControl:
    """
    MPC delta is in CARLA frame (left-handed).
    CARLA steer convention: positive = turn right.
    Bicycle model: positive delta = turn left (standard).
    In left-handed frame these are the same sign, so NO flip needed.
    We just normalize by DELTA_MAX.
    """
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


# ═══════════════════════════════════════════════════
# KICK-START
# ═══════════════════════════════════════════════════
print("[4] Kick-starting (3 ticks)...")
for _ in range(3):
    vehicle.apply_control(carla.VehicleControl(throttle=0.4, steer=0.0))
    world.tick()

# ═══════════════════════════════════════════════════
# MAIN LOOP
# ═══════════════════════════════════════════════════
print("\n── MPC loop (Ctrl-C to stop) ──\n")

u_prev   = np.zeros(2)
step     = 0
err_hist = []

# Saturation counter: if steering maxes out 20 steps in a row,
# auto-flip and report it — helps diagnose the sign empirically
sat_count  = 0
did_flip   = False
steer_sign = +1.0   # multiply delta by this before sending to CARLA

try:
    while True:
        world.tick()
        step += 1

        x = get_state(vehicle)
        ref_x, ref_y, ref_psi, ref_v = get_ref_window(x, RX, RY, RPSI, RV)

        t0 = time.perf_counter()
        u0, _, _ = build_qp(x, ref_x, ref_y, ref_psi, ref_v, u_prev)
        dt_ms = (time.perf_counter() - t0) * 1000

        delta = steer_sign * u0[0]
        a     = u0[1]
        cmd   = to_carla_cmd(delta, a)
        vehicle.apply_control(cmd)
        u_prev = u0.copy()

        update_spectator(vehicle)

        err = np.hypot(x[0] - ref_x[0], x[1] - ref_y[0])
        err_hist.append(err)

        sat = abs(u0[0]) >= DELTA_MAX * 0.98
        sat_count = sat_count + 1 if sat else 0
        sat_str = " SAT" if sat else "    "

        print(f"step {step:4d} | "
              f"pos=({x[0]:7.1f},{x[1]:7.1f}) | "
              f"v={x[3]:.2f} | "
              f"δ={u0[0]:+.3f}{sat_str} a={a:+.2f} | "
              f"steer={cmd.steer:+.3f} | "
              f"err={err:.2f}m | {dt_ms:.1f}ms")

        # ── Auto-flip if saturated for 20 steps and err growing ──
        if sat_count >= 20 and not did_flip:
            recent_err = err_hist[-20:]
            if recent_err[-1] > recent_err[0]:   # error is growing
                steer_sign *= -1.0
                did_flip = True
                sat_count = 0
                print(f"\n  *** AUTO-FLIP: steer_sign → {steer_sign:+.0f} "
                      f"(err was growing: {recent_err[0]:.1f}→{recent_err[-1]:.1f}m) ***\n")

finally:
    print(f"\n── Stopped at step {step} ──")
    if err_hist:
        print(f"   Mean err={np.mean(err_hist):.2f}m  "
              f"Max err={np.max(err_hist):.2f}m")
    settings.synchronous_mode = False
    world.apply_settings(settings)
    vehicle.destroy()
    print("   Cleaned up.")