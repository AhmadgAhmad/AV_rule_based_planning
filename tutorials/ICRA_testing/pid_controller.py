"""
pid_controller.py — PID Trajectory Tracking for the Hierarchical Planner
==========================================================================
Standard CARLA-style split controller:

    PIDLongitudinalController  →  target speed error        → throttle/brake
    PIDLateralController       →  heading error to waypoint  → steer
    VehiclePIDController       →  combines both, one run_step() per tick

This is what PathExecutor (in planner.py) now calls internally. Previously
PathExecutor used bare proportional gains (KP_STEER, KP_THROTTLE) with no
memory of past error — fine for a demo, but it leaves steady-state error
uncorrected and can't damp oscillation. Adding the I and D terms here is
literally the difference between "P controller" and "PID controller":

    P  → reacts to how far off you are right now
    I  → reacts to how long you've been off (kills steady-state error,
         e.g. never-quite-reaching the target speed on a slope)
    D  → reacts to how fast the error is changing (damps overshoot /
         steering oscillation around the reference path)

Author: Ahmad Ahmad | For: Nidhi's Autonomous Driving Course
"""

import math

import carla


class PIDLongitudinalController:
    """PID controller for speed tracking. Output in [-1, 1]: positive
    is interpreted as throttle, negative as brake."""

    def __init__(self, K_P=1.0, K_I=0.05, K_D=0.05, dt=0.05, integral_limit=10.0):
        self._k_p = K_P
        self._k_i = K_I
        self._k_d = K_D
        self._dt = dt
        self._integral = 0.0
        self._integral_limit = integral_limit   # anti-windup clamp
        self._prev_error = None

    def run_step(self, target_speed_ms, current_speed_ms):
        error = target_speed_ms - current_speed_ms

        self._integral += error * self._dt
        self._integral = max(-self._integral_limit, min(self._integral_limit, self._integral))

        de = 0.0 if self._prev_error is None else (error - self._prev_error) / self._dt
        self._prev_error = error

        output = (self._k_p * error) + (self._k_d * de) + (self._k_i * self._integral)
        return max(-1.0, min(1.0, output))

    def reset(self):
        self._integral = 0.0
        self._prev_error = None


class PIDLateralController:
    """PID controller for heading error → steer, in [-1, 1].

    Error is the signed angle between the vehicle's forward vector and the
    vector from the vehicle to the target waypoint (positive = target is to
    the right, so steer right).
    """

    def __init__(self, K_P=1.2, K_I=0.02, K_D=0.15, dt=0.05, integral_limit=2.0):
        self._k_p = K_P
        self._k_i = K_I
        self._k_d = K_D
        self._dt = dt
        self._integral = 0.0
        self._integral_limit = integral_limit
        self._prev_error = None

    def run_step(self, target_location, vehicle_transform):
        v_begin = vehicle_transform.location
        yaw_rad = math.radians(vehicle_transform.rotation.yaw)
        v_end = carla.Location(x=v_begin.x + math.cos(yaw_rad),
                                y=v_begin.y + math.sin(yaw_rad))

        v_vec = (v_end.x - v_begin.x, v_end.y - v_begin.y)
        w_vec = (target_location.x - v_begin.x, target_location.y - v_begin.y)

        mag_v = math.hypot(*v_vec)
        mag_w = math.hypot(*w_vec)
        if mag_v * mag_w < 1e-6:
            return 0.0

        dot = v_vec[0] * w_vec[0] + v_vec[1] * w_vec[1]
        cos_theta = max(-1.0, min(1.0, dot / (mag_v * mag_w)))
        error = math.acos(cos_theta)

        # sign via 2D cross product: which side of the heading is the target on
        cross = v_vec[0] * w_vec[1] - v_vec[1] * w_vec[0]
        if cross < 0:
            error *= -1.0

        self._integral += error * self._dt
        self._integral = max(-self._integral_limit, min(self._integral_limit, self._integral))

        de = 0.0 if self._prev_error is None else (error - self._prev_error) / self._dt
        self._prev_error = error

        output = (self._k_p * error) + (self._k_d * de) + (self._k_i * self._integral)
        return max(-1.0, min(1.0, output))

    def reset(self):
        self._integral = 0.0
        self._prev_error = None


class VehiclePIDController:
    """Combines longitudinal + lateral PID into a single carla.VehicleControl
    per tick. This is the drop-in replacement for the old P-only logic in
    PathExecutor.step()."""

    def __init__(self, dt=0.05, args_longitudinal=None, args_lateral=None,
                 max_throttle=0.8, max_brake=1.0, max_steer=0.8):
        args_longitudinal = args_longitudinal or {}
        args_lateral = args_lateral or {}
        self._max_throttle = max_throttle
        self._max_brake = max_brake
        self._max_steer = max_steer
        self._lon_controller = PIDLongitudinalController(dt=dt, **args_longitudinal)
        self._lat_controller = PIDLateralController(dt=dt, **args_lateral)

    def reset(self):
        self._lon_controller.reset()
        self._lat_controller.reset()

    def run_step(self, target_speed_ms, target_location, ego):
        transform = ego.get_transform()
        vel = ego.get_velocity()
        current_speed_ms = math.hypot(vel.x, vel.y)

        accel = self._lon_controller.run_step(target_speed_ms, current_speed_ms)
        steer = self._lat_controller.run_step(target_location, transform)

        control = carla.VehicleControl()
        if accel >= 0.0:
            control.throttle = min(accel, self._max_throttle)
            control.brake = 0.0
        else:
            control.throttle = 0.0
            control.brake = min(-accel, self._max_brake)

        control.steer = max(-self._max_steer, min(self._max_steer, steer))
        control.hand_brake = False
        control.manual_gear_shift = False
        return control
