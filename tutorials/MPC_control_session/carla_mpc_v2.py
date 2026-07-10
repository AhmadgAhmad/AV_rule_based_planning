"""
carla_mpc_v2.py — MPC controller in CARLA
==========================================
Replaces the PID from carla_pid.py with our OSQP MPC.
Loop structure identical to carla_pid.py — only the control
step changes. mpc_osqp.py is imported cleanly (no side effects).

Run:
    python carla_mpc_v2.py
"""

import carla
import math
import time
import sys
import numpy as np

# ── CARLA path ────────────────────────────────────────────────────
CARLA_ROOT = '/opt/carla-simulator/CARLA_0.9.13_RSS'
sys.path.insert(0, f'{CARLA_ROOT}/PythonAPI/carla')
sys.path.insert(0, f'{CARLA_ROOT}/PythonAPI/carla/agents')

# ── MPC imports — safe now that mpc_osqp has __main__ guard ──────
sys.path.insert(0, '/home/ahmad/Desktop/RuleBookDriving/tutorials/MPC_control_session')
from mpc_osqp import build_qp, get_ref_window

print("✓ mpc_osqp imported cleanly (no simulation ran)")


# ═══════════════════════════════════════════════════════════
# CONFIG
# ═══════════════════════════════════════════════════════════
V_REF          =  3.0   # m/s  — target speed (must match mpc_osqp.V_REF)
WAYPOINT_SPACE =  2.0   # m between reference waypoints
N_WAYPOINTS    =  300
DT             =  0.1   # s  — sync tick, must match mpc_osqp.DT
DELTA_MAX      =  0.45  # rad
A_MAX          =  3.0   # m/s²
A_MIN          = -5.0   # m/s²


# ═══════════════════════════════════════════════════════════
# CONNECT + SYNC
# ═══════════════════════════════════════════════════════════
print("[1] Connecting to CARLA...")
client  = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world   = client.get_world()
bp_lib  = world.get_blueprint_library()
cmap    = world.get_map()
debug   = world.debug
print(f"    Map: {cmap.name}")

settings = world.get_settings()
settings.synchronous_mode    = True
settings.fixed_delta_seconds = DT
world.apply_settings(settings)
print(f"[2] Sync mode ON  (dt={DT}s)")


# ═══════════════════════════════════════════════════════════
# SPAWN
# ═══════════════════════════════════════════════════════════
print("[3] Spawning vehicle...")
bp       = bp_lib.find('vehicle.tesla.model3')
spawn_pt = cmap.get_spawn_points()[0]
vehicle  = world.spawn_actor(bp, spawn_pt)
vehicle.set_autopilot(False)
for _ in range(5):
    world.tick()

tf = vehicle.get_transform()
print(f"    ID={vehicle.id}  pos=({tf.location.x:.1f},{tf.location.y:.1f})  "
      f"yaw={tf.rotation.yaw:.1f}°")


# ═══════════════════════════════════════════════════════════
# SPECTATOR
# ═══════════════════════════════════════════════════════════
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


# ═══════════════════════════════════════════════════════════
# REFERENCE PATH  (CARLA waypoints → numpy arrays for MPC)
# ═══════════════════════════════════════════════════════════
print("[4] Building reference path...")

def build_ref_from_carla(world, vehicle, n=N_WAYPOINTS, spacing=WAYPOINT_SPACE):
    wp = world.get_map().get_waypoint(
        vehicle.get_transform().location,
        project_to_road=True,
        lane_type=carla.LaneType.Driving)
    xs, ys, psis, wps = [], [], [], []
    for _ in range(n):
        loc = wp.transform.location
        xs.append(loc.x)
        ys.append(loc.y)
        psis.append(math.radians(wp.transform.rotation.yaw))
        wps.append(wp)
        nxt = wp.next(spacing)
        if not nxt: break
        wp = nxt[0]
    RX   = np.array(xs)
    RY   = np.array(ys)
    RPSI = np.array(psis)
    RV   = np.full(len(xs), V_REF)
    print(f"    {len(xs)} pts  start=({xs[0]:.1f},{ys[0]:.1f})  "
          f"end=({xs[-1]:.1f},{ys[-1]:.1f})")
    return RX, RY, RPSI, RV, wps

RX, RY, RPSI, RV, wp_list = build_ref_from_carla(world, vehicle)

# Draw reference in CARLA viewport
print("[5] Drawing reference path...")
for i, wp in enumerate(wp_list):
    loc = wp.transform.location
    debug.draw_point(
        carla.Location(x=loc.x, y=loc.y, z=loc.z+0.5),
        size=0.07, color=carla.Color(0,255,0), life_time=120.0)
    if i % 20 == 0:
        yaw_r = math.radians(wp.transform.rotation.yaw)
        debug.draw_arrow(
            carla.Location(x=loc.x, y=loc.y, z=loc.z+0.5),
            carla.Location(x=loc.x+3*math.cos(yaw_r),
                           y=loc.y+3*math.sin(yaw_r), z=loc.z+0.5),
            thickness=0.1, arrow_size=0.3,
            color=carla.Color(0,200,255), life_time=120.0)

debug.draw_point(carla.Location(x=float(RX[0]),  y=float(RY[0]),  z=2.0),
                 size=0.25, color=carla.Color(255,0,0),   life_time=120.0)
debug.draw_point(carla.Location(x=float(RX[-1]), y=float(RY[-1]), z=2.0),
                 size=0.25, color=carla.Color(0,0,255),   life_time=120.0)
world.tick()


# ═══════════════════════════════════════════════════════════
# STATE + CONTROL CONVERTERS
# ═══════════════════════════════════════════════════════════
def get_state(vehicle) -> np.ndarray:
    """CARLA vehicle → MPC state [X, Y, psi, v]."""
    tf  = vehicle.get_transform()
    vel = vehicle.get_velocity()
    psi = math.radians(tf.rotation.yaw)   # CARLA frame as-is
    v   = math.sqrt(vel.x**2 + vel.y**2)
    return np.array([tf.location.x, tf.location.y, psi, v])


def to_carla_cmd(u0: np.ndarray) -> carla.VehicleControl:
    """MPC [delta, a] → CARLA VehicleControl."""
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


# ═══════════════════════════════════════════════════════════
# MAIN LOOP
# ═══════════════════════════════════════════════════════════
print(f"\n── MPC loop running at V_REF={V_REF}m/s (Ctrl-C to stop) ──\n")

u_prev    = np.zeros(2)
step      = 0
err_hist  = []
spd_hist  = []
solve_hist= []
loop_start = time.time()

try:
    while True:
        world.tick()
        step += 1

        # ① Read state
        x = get_state(vehicle)

        # ② Reference window for this step
        ref_x, ref_y, ref_psi, ref_v = get_ref_window(x, RX, RY, RPSI, RV)

        # ③ Solve MPC
        t0 = time.perf_counter()
        u0, X_pred, _ = build_qp(x, ref_x, ref_y, ref_psi, ref_v, u_prev)
        dt_ms = (time.perf_counter() - t0) * 1000
        solve_hist.append(dt_ms)

        # ④ Apply control
        cmd = to_carla_cmd(u0)
        vehicle.apply_control(cmd)
        u_prev = u0.copy()

        update_spectator(vehicle)

        # ⑤ Metrics
        err = np.hypot(x[0] - ref_x[0], x[1] - ref_y[0])
        spd = x[3]
        err_hist.append(err)
        spd_hist.append(spd)

        # ⑥ Live debug overlays
        debug.draw_point(
            carla.Location(x=float(ref_x[0]), y=float(ref_y[0]),
                           z=vehicle.get_transform().location.z + 1.0),
            size=0.15, color=carla.Color(255,0,200), life_time=DT*2)

        sat = abs(u0[0]) >= DELTA_MAX * 0.98
        veh_loc = vehicle.get_transform().location
        debug.draw_string(
            carla.Location(x=veh_loc.x-2, y=veh_loc.y, z=veh_loc.z+5),
            f"step:{step} err:{err:.2f}m v:{spd:.1f}m/s "
            f"d:{u0[0]:+.3f}{'!' if sat else ' '} solve:{dt_ms:.1f}ms",
            color=carla.Color(255,255,255), life_time=DT*2)

        # ⑦ Log
        if step % 10 == 0:
            print(f"step {step:4d} | "
                  f"pos=({x[0]:6.1f},{x[1]:6.1f}) | "
                  f"v={spd:.2f}m/s | "
                  f"d={u0[0]:+.3f}{'SAT' if sat else '   '} "
                  f"a={u0[1]:+.2f} | "
                  f"err={err:.2f}m | "
                  f"{dt_ms:.1f}ms")

finally:
    elapsed = time.time() - loop_start
    print(f"\n── Summary ──────────────────────────────")
    print(f"   Steps:           {step}")
    print(f"   Time:            {elapsed:.1f}s")
    if err_hist:
        print(f"   Mean track err:  {np.mean(err_hist):.3f} m")
        print(f"   Max  track err:  {np.max(err_hist):.3f} m")
    if solve_hist:
        print(f"   Mean solve time: {np.mean(solve_hist):.2f} ms")
        print(f"   Max  solve time: {np.max(solve_hist):.2f} ms")

    settings.synchronous_mode = False
    world.apply_settings(settings)
    vehicle.destroy()
    print("   Cleaned up.")
