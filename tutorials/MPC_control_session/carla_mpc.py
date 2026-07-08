"""
carla_mpc.py  —  MPC controller in CARLA
==========================================
Fixed version. Key changes from original:
  - Imports from mpc_osqp (not mpc_simulation)
  - Spectator camera follows vehicle every tick
  - One warm-up tick after spawn before entering loop
  - Verbose spawn/position prints for debugging
  - try/finally cleans up reliably

Run from the same folder as mpc_osqp.py:
    python carla_mpc.py
"""

import carla
import numpy as np
import time
import math
import sys

# ── Make sure mpc_osqp.py is importable ──────────────────────────
# Change this path if mpc_osqp.py is somewhere else
# sys.path.insert(0, '/home/ahmad/Desktop/RuleBookDriving/tutorials/MPC_control_session')

from mpc_osqp import build_qp, get_ref_window   # ← correct file now

# ═══════════════════════════════════════════════════
# CONSTANTS  (must match mpc_osqp.py)
# ═══════════════════════════════════════════════════
DELTA_MAX = 0.45    # [rad]
A_MAX     =  3.0    # [m/s²]
A_MIN     = -5.0    # [m/s²]
DT        =  0.1    # [s]  — must match mpc_osqp DT


# ═══════════════════════════════════════════════════
# 1.  CONNECT TO CARLA
# ═══════════════════════════════════════════════════
print("[1/5] Connecting to CARLA...")
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world  = client.get_world()
bp_lib = world.get_blueprint_library()
print(f"      Connected. Map: {world.get_map().name}")


# ═══════════════════════════════════════════════════
# 2.  SYNCHRONOUS MODE
# ═══════════════════════════════════════════════════
settings = world.get_settings()
settings.synchronous_mode    = True
settings.fixed_delta_seconds = DT
world.apply_settings(settings)
print(f"[2/5] Sync mode ON  (dt={DT}s)")


# ═══════════════════════════════════════════════════
# 3.  SPAWN VEHICLE
# ═══════════════════════════════════════════════════
print("[3/5] Spawning vehicle...")
bp       = bp_lib.find('vehicle.tesla.model3')
spawn_pts = world.get_map().get_spawn_points()
spawn_pt  = spawn_pts[0]

vehicle = world.spawn_actor(bp, spawn_pt)
vehicle.set_autopilot(False)

# ── Warm-up tick: lets physics initialize before we read state ──
world.tick()

loc = vehicle.get_transform().location
print(f"      Spawned ID={vehicle.id}  "
      f"pos=({loc.x:.1f}, {loc.y:.1f}, {loc.z:.1f})")

if loc.z < 0.1:
    print("      [WARN] z < 0.1 — vehicle may be underground. "
          "Try spawn_pts[1] or spawn_pts[2].")


# ═══════════════════════════════════════════════════
# 4.  SPECTATOR CAMERA  (so you can see the vehicle)
# ═══════════════════════════════════════════════════
spectator = world.get_spectator()

def update_spectator(vehicle, height=30.0, dist=0.0):
    """
    Bird's-eye spectator view following the vehicle.
    height : metres above vehicle
    dist   : metres behind vehicle (0 = directly above)
    """
    tf  = vehicle.get_transform()
    yaw = tf.rotation.yaw               # degrees
    # Position: above and slightly behind
    dx  = -dist * math.cos(math.radians(yaw))
    dy  = -dist * math.sin(math.radians(yaw))
    cam_loc = carla.Location(
        x = tf.location.x + dx,
        y = tf.location.y + dy,
        z = tf.location.z + height
    )
    cam_rot = carla.Rotation(pitch=-70, yaw=yaw, roll=0)
    spectator.set_transform(carla.Transform(cam_loc, cam_rot))

# Move spectator to vehicle immediately so you see it on first tick
update_spectator(vehicle)
print("[4/5] Spectator camera positioned above vehicle")


# ═══════════════════════════════════════════════════
# 5.  REFERENCE PATH FROM MAP
# ═══════════════════════════════════════════════════
def build_ref_from_map(world, start_transform, n_pts=300, spacing=2.0, v_ref=7.0):
    """Follow CARLA lane waypoints → (RX, RY, RPSI, RV) arrays."""
    cmap = world.get_map()
    wp   = cmap.get_waypoint(start_transform.location)

    xs, ys, psis = [], [], []
    for _ in range(n_pts):
        loc = wp.transform.location
        xs.append(loc.x)
        ys.append(loc.y)
        psis.append(math.radians(wp.transform.rotation.yaw))
        nxt = wp.next(spacing)
        if not nxt:
            break
        wp = nxt[0]

    RX   = np.array(xs)
    RY   = np.array(ys)
    RPSI = np.array(psis)
    RV   = np.full(len(xs), v_ref)
    print(f"[5/5] Reference path: {len(xs)} waypoints, "
          f"start=({xs[0]:.1f},{ys[0]:.1f}), "
          f"end=({xs[-1]:.1f},{ys[-1]:.1f})")
    return RX, RY, RPSI, RV

RX, RY, RPSI, RV = build_ref_from_map(world, spawn_pt)


# ═══════════════════════════════════════════════════
# HELPER FUNCTIONS
# ═══════════════════════════════════════════════════

def get_state(vehicle) -> np.ndarray:
    """CARLA vehicle → MPC state [X, Y, ψ, v]"""
    tf  = vehicle.get_transform()
    vel = vehicle.get_velocity()
    psi = math.radians(tf.rotation.yaw)
    v   = math.sqrt(vel.x**2 + vel.y**2)
    return np.array([tf.location.x, tf.location.y, psi, v])


def to_carla_cmd(u0: np.ndarray) -> carla.VehicleControl:
    """MPC [δ, a] → CARLA VehicleControl (normalized)"""
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


# ═══════════════════════════════════════════════════
# MAIN LOOP
# ═══════════════════════════════════════════════════
print("\n── Starting MPC loop (Ctrl-C to stop) ──\n")

u_prev    = np.zeros(2)
step      = 0
err_hist  = []

try:
    while True:
        # ① Advance CARLA physics
        world.tick()
        step += 1

        # ② Read state
        x = get_state(vehicle)

        # ③ Reference window
        ref_x, ref_y, ref_psi, ref_v = get_ref_window(x, RX, RY, RPSI, RV)

        # ④ Solve MPC
        t0 = time.perf_counter()
        u0, X_pred, _ = build_qp(x, ref_x, ref_y, ref_psi, ref_v, u_prev)
        dt_ms = (time.perf_counter() - t0) * 1000

        # ⑤ Apply control
        vehicle.apply_control(to_carla_cmd(u0))
        u_prev = u0.copy()

        # ⑥ Move spectator to follow
        update_spectator(vehicle, height=25, dist=10)

        # ⑦ Log every step
        err = np.hypot(x[0] - ref_x[0], x[1] - ref_y[0])
        err_hist.append(err)
        print(f"step {step:4d} | "
              f"pos=({x[0]:7.2f},{x[1]:7.2f}) | "
              f"v={x[3]:.2f}m/s | "
              f"δ={u0[0]:+.3f} a={u0[1]:+.2f} | "
              f"err={err:.2f}m | "
              f"solve={dt_ms:.1f}ms")

        # ── Steer sign debug (first 5 steps only) ────────────
        if step <= 5:
            cmd = to_carla_cmd(u0)
            print(f"       → CARLA cmd: steer={cmd.steer:+.3f}  "
                  f"throttle={cmd.throttle:.2f}  brake={cmd.brake:.2f}")

        # ── Safety: stop if vehicle goes underground ──────────
        if x[2] < -5.0:       # psi check is heading, use z separately
            tf = vehicle.get_transform()
            if tf.location.z < -2.0:
                print("[WARN] Vehicle underground — stopping loop")
                break

finally:
    # ── Always restore and clean up ───────────────────────────
    print(f"\n── Loop ended at step {step} ──")
    if err_hist:
        print(f"   Mean tracking error : {np.mean(err_hist):.3f} m")
        print(f"   Max  tracking error : {np.max(err_hist):.3f} m")

    settings.synchronous_mode = False
    world.apply_settings(settings)
    vehicle.destroy()
    print("   Cleaned up. Exiting.")