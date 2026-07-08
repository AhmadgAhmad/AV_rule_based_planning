"""
carla_mpc.py  —  MPC + CARLA  (fixed coordinate frame + ref alignment)
=======================================================================
Fixes from v1:
  1. Steering sign flipped  (CARLA left-handed Y → negate delta)
  2. Reference path starts AT the vehicle, not at waypoint[0]
  3. Vehicle gets a throttle kick to overcome static friction
  4. Tighter steering weight to stop saturation
  5. Spectator follows vehicle
"""

import carla
import numpy as np
import time
import math
import sys

sys.path.insert(0, '/home/ahmad/Desktop/RuleBookDriving/tutorials/MPC_control_session')
from mpc_osqp import build_qp, get_ref_window

# ═══════════════════════════════════════════════════
# CONSTANTS
# ═══════════════════════════════════════════════════
DELTA_MAX = 0.45
A_MAX     =  3.0
A_MIN     = -5.0
DT        =  0.1
V_REF     =  5.0    # start slower (7 was too fast for the first corner)

# ── Coordinate frame flag ─────────────────────────
# CARLA uses left-handed coords: Y points RIGHT.
# Our MPC assumes standard right-handed (Y points LEFT).
# Effect: steering sign is flipped.  Set True to correct it.
FLIP_STEER = True


# ═══════════════════════════════════════════════════
# CONNECT + SYNC MODE
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
print(f"[2] Sync mode ON (dt={DT}s)")


# ═══════════════════════════════════════════════════
# SPAWN VEHICLE
# ═══════════════════════════════════════════════════
print("[3] Spawning vehicle...")
bp        = bp_lib.find('vehicle.tesla.model3')
spawn_pts = world.get_map().get_spawn_points()
spawn_pt  = spawn_pts[0]

vehicle = world.spawn_actor(bp, spawn_pt)
vehicle.set_autopilot(False)

# Warm-up: 3 ticks so physics settle before we read state
for _ in range(3):
    world.tick()

loc = vehicle.get_transform().location
print(f"    ID={vehicle.id}  pos=({loc.x:.1f}, {loc.y:.1f}, {loc.z:.1f})")


# ═══════════════════════════════════════════════════
# SPECTATOR
# ═══════════════════════════════════════════════════
spectator = world.get_spectator()

def update_spectator(v, height=28, dist=12):
    tf  = v.get_transform()
    yaw = tf.rotation.yaw
    dx  = -dist * math.cos(math.radians(yaw))
    dy  = -dist * math.sin(math.radians(yaw))
    spectator.set_transform(carla.Transform(
        carla.Location(x=tf.location.x+dx,
                       y=tf.location.y+dy,
                       z=tf.location.z+height),
        carla.Rotation(pitch=-65, yaw=yaw, roll=0)
    ))

update_spectator(vehicle)
print("[4] Spectator set")


# ═══════════════════════════════════════════════════
# REFERENCE PATH — starts FROM vehicle position
# ═══════════════════════════════════════════════════
def build_ref_from_vehicle(world, vehicle, n_pts=300, spacing=2.0, v_ref=V_REF):
    """
    Build reference path starting from the vehicle's CURRENT waypoint.
    This guarantees err≈0 at step 1 instead of 13m.
    """
    cmap = world.get_map()
    # Get the lane waypoint closest to where the vehicle actually IS
    wp = cmap.get_waypoint(
        vehicle.get_transform().location,
        project_to_road=True,
        lane_type=carla.LaneType.Driving
    )

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

    # Initial error check
    vx = vehicle.get_transform().location.x
    vy = vehicle.get_transform().location.y
    d0 = np.hypot(vx - RX[0], vy - RY[0])
    print(f"[5] Reference: {len(xs)} pts  "
          f"start=({xs[0]:.1f},{ys[0]:.1f})  "
          f"initial offset={d0:.2f}m")
    if d0 > 3.0:
        print(f"    [WARN] Initial offset {d0:.1f}m > 3m — "
              f"vehicle may not be on the road center")
    return RX, RY, RPSI, RV

RX, RY, RPSI, RV = build_ref_from_vehicle(world, vehicle)


# ═══════════════════════════════════════════════════
# HELPERS
# ═══════════════════════════════════════════════════
def get_state(vehicle) -> np.ndarray:
    tf  = vehicle.get_transform()
    vel = vehicle.get_velocity()
    psi = math.radians(tf.rotation.yaw)
    v   = math.sqrt(vel.x**2 + vel.y**2)
    return np.array([tf.location.x, tf.location.y, psi, v])


def to_carla_cmd(u0: np.ndarray) -> carla.VehicleControl:
    delta, a = u0
    cmd = carla.VehicleControl()

    # ── Coordinate frame fix ──────────────────────────────────
    # MPC solves in standard right-handed frame.
    # CARLA's Y axis is flipped (left-handed), so positive delta
    # (left turn in math) becomes a right turn in CARLA.
    # Negating steer corrects this.
    steer = delta / DELTA_MAX
    if FLIP_STEER:
        steer = -steer
    cmd.steer = float(np.clip(steer, -1.0, 1.0))

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
# KICK-START: give the car a small push so it's moving
# before MPC takes over (avoids static-friction stall)
# ═══════════════════════════════════════════════════
print("[6] Kick-starting vehicle (3 ticks throttle)...")
kickstart = carla.VehicleControl(throttle=0.4, steer=0.0,
                                  brake=0.0, hand_brake=False)
for _ in range(3):
    vehicle.apply_control(kickstart)
    world.tick()


# ═══════════════════════════════════════════════════
# MAIN LOOP
# ═══════════════════════════════════════════════════
print("\n── MPC loop running (Ctrl-C to stop) ──\n"
      f"    FLIP_STEER={FLIP_STEER}  V_REF={V_REF}m/s\n")

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
        u0, X_pred, _ = build_qp(x, ref_x, ref_y, ref_psi, ref_v, u_prev)
        dt_ms = (time.perf_counter() - t0) * 1000

        cmd = to_carla_cmd(u0)
        vehicle.apply_control(cmd)
        u_prev = u0.copy()

        update_spectator(vehicle)

        err = np.hypot(x[0] - ref_x[0], x[1] - ref_y[0])
        err_hist.append(err)

        # ── Steering saturation warning ───────────────────────
        sat = abs(u0[0]) >= DELTA_MAX * 0.98
        sat_flag = " ← SATURATED" if sat else ""

        print(f"step {step:4d} | "
              f"pos=({x[0]:7.1f},{x[1]:7.1f}) | "
              f"v={x[3]:.2f}m/s | "
              f"δ={u0[0]:+.3f}{sat_flag} "
              f"a={u0[1]:+.2f} | "
              f"steer={cmd.steer:+.3f} | "
              f"err={err:.2f}m | "
              f"{dt_ms:.1f}ms")

        # ── If saturated for 10+ steps, flip steer sign ───────
        if step == 10 and all(abs(e) > 5.0 for e in err_hist):
            global FLIP_STEER
            FLIP_STEER = not FLIP_STEER
            print(f"\n    [AUTO-FIX] Error still high after 10 steps — "
                  f"flipping steer sign. FLIP_STEER={FLIP_STEER}\n")

finally:
    print(f"\n── Stopped at step {step} ──")
    if err_hist:
        print(f"   Mean err: {np.mean(err_hist):.3f}m  "
              f"Max err: {np.max(err_hist):.3f}m")
    settings.synchronous_mode = False
    world.apply_settings(settings)
    vehicle.destroy()
    print("   Cleaned up.")