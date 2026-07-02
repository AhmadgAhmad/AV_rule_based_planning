import carla, numpy as np, time, math
import sys
sys.path.append('/opt/carla-simulator/PythonAPI/carla')

from mpc_simulation import build_qp, bicycle_step, get_ref_window  # our MPC

# ── Connect ──────────────────────────────────────────────
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world  = client.get_world()
bp_lib = world.get_blueprint_library()

# ── Synchronous mode: we control the tick ────────────────
settings = world.get_settings()
settings.synchronous_mode = True
settings.fixed_delta_seconds = 0.1   # must match MPC dt!
world.apply_settings(settings)

# ── Spawn vehicle ─────────────────────────────────────────
bp       = bp_lib.find('vehicle.tesla.model3')
spawn_pt = world.get_map().get_spawn_points()[0]
vehicle  = world.spawn_actor(bp, spawn_pt)
vehicle.set_autopilot(False)
print(f"Spawned: {vehicle.id} at {spawn_pt.location}")

def get_state(vehicle) -> np.ndarray:
    """Read CARLA vehicle → MPC state [X, Y, ψ, v]"""
    tf  = vehicle.get_transform()
    vel = vehicle.get_velocity()

    # CARLA yaw: degrees, clockwise-positive (left-handed)
    # Convert to radians, counter-clockwise (standard math)
    psi = math.radians(tf.rotation.yaw)

    # World-frame speed magnitude
    v   = math.sqrt(vel.x**2 + vel.y**2)

    return np.array([
        tf.location.x,   # X [m]
        tf.location.y,   # Y [m]
        psi,             # ψ [rad]
        v,               # speed [m/s]
    ])