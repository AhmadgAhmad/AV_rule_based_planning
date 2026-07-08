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
V_REF     =  5.0

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
def build_ref(world, vehicle, n_pts=300, spacing=2.0):
    cmap = world.get_map()
    wp   = cmap.get_waypoint(
        vehicle.get_transform().location,
        project_to_road=True,
        lane_type=carla.LaneType.Driving)

    xs, ys, psis = [], [], []
    for _ in range(n_pts):
        loc = wp.transform.location
        xs.append(loc.x)
        ys.append(loc.y)
        # ★ Negate yaw: CARLA clockwise → MPC counter-clockwise
        psis.append(-math.radians(wp.transform.rotation.yaw))
        nxt = wp.next(spacing)
        if not nxt: break
        wp = nxt[0]

    RX   = np.array(xs)
    RY   = np.array(ys)
    RPSI = np.array(psis)
    RV   = np.full(len(xs), V_REF)

    vpos = vehicle.get_transform().location
    d0   = np.hypot(vpos.x - RX[0], vpos.y - RY[0])
    print(f"[3] Ref: {len(xs)} pts  d0={d0:.2f}m  "
          f"psi[0]={math.degrees(RPSI[0]):.1f}°(MPC)  "
          f"psi[1]={math.degrees(RPSI[1]):.1f}°")
    return RX, RY, RPSI, RV

RX, RY, RPSI, RV = build_ref(world, vehicle)


# ── State: negate psi for MPC ────────────────────────────────────
def get_state(vehicle) -> np.ndarray:
    """
    CARLA → MPC state.
    X, Y stay as-is (same axes).
    psi negated: CARLA clockwise → MPC counter-clockwise.
    v is speed magnitude, always positive.
    """
    tf  = vehicle.get_transform()
    vel = vehicle.get_velocity()
    psi_carla = math.radians(tf.rotation.yaw)
    psi_mpc   = -psi_carla               # ★ THE FIX
    v         = math.sqrt(vel.x**2 + vel.y**2)
    return np.array([tf.location.x, tf.location.y, psi_mpc, v])


# ── Control: negate delta back to CARLA ──────────────────────────
def to_carla_cmd(u0: np.ndarray) -> carla.VehicleControl:
    """
    MPC output delta is in right-handed frame.
    Negate to convert back to CARLA left-handed steer.
    """
    delta_mpc, a = u0
    delta_carla  = -delta_mpc            # ★ THE FIX (inverse of get_state)
    cmd = carla.VehicleControl()
    cmd.steer = float(np.clip(delta_carla / DELTA_MAX, -1.0, 1.0))
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
print("[4] Kick-starting...")
for _ in range(3):
    vehicle.apply_control(carla.VehicleControl(throttle=0.4, steer=0.0))
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

        sat = abs(u0[0]) >= DELTA_MAX * 0.98
        print(f"step {step:4d} | "
              f"pos=({x[0]:7.1f},{x[1]:7.1f}) | "
              f"v={x[3]:.2f} | "
              f"δ={u0[0]:+.3f}{'SAT' if sat else '   '} "
              f"a={u0[1]:+.2f} | "
              f"steer={cmd.steer:+.3f} | "
              f"err={err:.2f}m | {dt_ms:.1f}ms")

finally:
    print(f"\n── Stopped at step {step} ──")
    if err_hist:
        print(f"   Mean err={np.mean(err_hist):.2f}m  "
              f"Max err={np.max(err_hist):.2f}m")
    settings.synchronous_mode = False
    world.apply_settings(settings)
    vehicle.destroy()
    print("   Cleaned up.")