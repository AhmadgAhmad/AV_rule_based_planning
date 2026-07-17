"""
mock_carla.py — Minimal fake `carla` module for offline unit testing
========================================================================
This is NOT a CARLA replacement — it implements only the handful of
classes/methods that pid_controller.py and planner.py actually touch
(Location, Rotation, Transform, VehicleControl, TrafficLightState, plus
enough of Actor/World/Map to drive the planner and PID logic). It lets you
sanity-check the math and the P0/P1/P2 filtering logic on a laptop with no
GPU and no simulator running.

It does NOT model real vehicle physics, road geometry, or CARLA's actual
API surface — scenario_1_integrated.py through scenario_4_highway_traffic.py
still need a real CARLA server; this only covers the algorithmic core
(pid_controller.py + planner.py).

Usage (see test_offline.py):
    import sys
    sys.modules['carla'] = __import__('mock_carla')
    from planner import HierarchicalPlanner, ...   # now resolves against the fake

Author: Ahmad Ahmad | For: Nidhi's Autonomous Driving Course
"""

import math


# ═══════════════════════════════════════════════════════════════════════════
# Geometry primitives
# ═══════════════════════════════════════════════════════════════════════════

class Vector3D:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = x, y, z


class Location:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = x, y, z

    def distance(self, other):
        return math.sqrt((self.x - other.x) ** 2 +
                          (self.y - other.y) ** 2 +
                          (self.z - other.z) ** 2)

    def __add__(self, other):
        return Location(self.x + other.x, self.y + other.y, self.z + other.z)

    def __repr__(self):
        return f"Location({self.x:.2f}, {self.y:.2f}, {self.z:.2f})"


class Rotation:
    def __init__(self, pitch=0.0, yaw=0.0, roll=0.0):
        self.pitch, self.yaw, self.roll = pitch, yaw, roll


class Transform:
    def __init__(self, location=None, rotation=None):
        self.location = location if location is not None else Location()
        self.rotation = rotation if rotation is not None else Rotation()

    def get_forward_vector(self):
        yaw = math.radians(self.rotation.yaw)
        return Vector3D(math.cos(yaw), math.sin(yaw), 0.0)

    def get_right_vector(self):
        yaw = math.radians(self.rotation.yaw) - math.pi / 2
        return Vector3D(math.cos(yaw), math.sin(yaw), 0.0)


class VehicleControl:
    def __init__(self, throttle=0.0, steer=0.0, brake=0.0,
                 hand_brake=False, manual_gear_shift=False):
        self.throttle = throttle
        self.steer = steer
        self.brake = brake
        self.hand_brake = hand_brake
        self.manual_gear_shift = manual_gear_shift

    def __repr__(self):
        return (f"VehicleControl(throttle={self.throttle:.2f}, "
                f"steer={self.steer:.2f}, brake={self.brake:.2f})")


class TrafficLightState:
    Red = 'Red'
    Yellow = 'Yellow'
    Green = 'Green'
    Off = 'Off'
    Unknown = 'Unknown'


class LaneType:
    Driving = 'Driving'
    Parking = 'Parking'
    Shoulder = 'Shoulder'
    Bidirectional = 'Bidirectional'


class WeatherParameters:
    def __init__(self, **kwargs):
        for k, v in kwargs.items():
            setattr(self, k, v)


class Color:
    def __init__(self, r=0, g=0, b=0, a=255):
        self.r, self.g, self.b, self.a = r, g, b, a


class DebugHelper:
    """No-op stand-in for world.debug.draw_line / draw_string."""
    def draw_line(self, *args, **kwargs):
        pass

    def draw_string(self, *args, **kwargs):
        pass


# ═══════════════════════════════════════════════════════════════════════════
# Road / map fixtures
# ═══════════════════════════════════════════════════════════════════════════

class Waypoint:
    """A point + heading on a straight road. `next(step)` advances along
    the current heading. `is_junction` can be forced True to simulate a
    junction zone."""

    def __init__(self, x, y, yaw=0.0, is_junction=False, z=0.0,
                 lane_width=3.5, lane_type=LaneType.Driving):
        self.transform = Transform(Location(x, y, z), Rotation(yaw=yaw))
        self.is_junction = is_junction
        self.lane_width = lane_width
        self.lane_type = lane_type

    def next(self, step):
        yaw = math.radians(self.transform.rotation.yaw)
        nx = self.transform.location.x + step * math.cos(yaw)
        ny = self.transform.location.y + step * math.sin(yaw)
        # by default, waypoints become junction once past JUNCTION_START_X
        # if the caller set it on the originating Map (see Map.get_waypoint)
        return [Waypoint(nx, ny, yaw=self.transform.rotation.yaw,
                          is_junction=self.is_junction,
                          lane_width=self.lane_width, lane_type=self.lane_type)]


class Map:
    """Fake carla_map: a straight road along +x. Optionally flags a stretch
    of x as a junction (for red-light P1 tests)."""

    def __init__(self, junction_x_range=None):
        self.junction_x_range = junction_x_range  # (xmin, xmax) or None

    def get_waypoint(self, location):
        is_junc = False
        if self.junction_x_range:
            xmin, xmax = self.junction_x_range
            is_junc = xmin <= location.x <= xmax
        return Waypoint(location.x, location.y, yaw=0.0, is_junction=is_junc)

    def get_spawn_points(self):
        return [Transform(Location(0.0, 0.0, 0.0), Rotation(yaw=0.0))]


# ═══════════════════════════════════════════════════════════════════════════
# Actors / world
# ═══════════════════════════════════════════════════════════════════════════

class ActorList(list):
    def filter(self, pattern):
        prefix = pattern.replace('*', '')
        return ActorList([a for a in self if getattr(a, 'type_id', '').startswith(prefix)])

    def find(self, actor_id):
        for a in self:
            if a.id == actor_id:
                return a
        return None


class Actor:
    """Fake dynamic actor (ego, obstacle vehicle, or pedestrian). Point-mass
    kinematics only — enough to check that PID control drives position/speed
    error to zero, NOT a substitute for CARLA's vehicle physics."""

    _next_id = 0

    def __init__(self, x=0.0, y=0.0, yaw=0.0, speed_limit=50.0,
                 type_id='vehicle.test.car'):
        Actor._next_id += 1
        self.id = Actor._next_id
        self.type_id = type_id
        self.is_alive = True
        self._loc = Location(x, y, 0.0)
        self._yaw = yaw
        self._speed = 0.0
        self._speed_limit = speed_limit

    def get_location(self):
        return self._loc

    def get_transform(self):
        return Transform(self._loc, Rotation(yaw=self._yaw))

    def get_velocity(self):
        yaw = math.radians(self._yaw)
        return Vector3D(self._speed * math.cos(yaw), self._speed * math.sin(yaw), 0.0)

    def get_speed_limit(self):
        return self._speed_limit

    def get_state(self):
        """Only meaningful if this Actor is standing in for a traffic light
        (type_id='traffic.traffic_light') — set via set_light_state()."""
        return getattr(self, '_light_state', None)

    def set_light_state(self, state):
        self._light_state = state

    def apply_control(self, control, dt=0.05, max_accel=3.0, max_yaw_rate_deg=90.0,
                       drag_coeff=0.0):
        """Toy point-mass + heading integrator. `drag_coeff` lets tests
        introduce a steady-state disturbance (e.g. rolling resistance) to
        demonstrate why the PID's integral term matters."""
        accel_cmd = control.throttle * max_accel - control.brake * max_accel
        drag = drag_coeff * self._speed
        self._speed = max(0.0, self._speed + (accel_cmd - drag) * dt)
        self._yaw += control.steer * max_yaw_rate_deg * dt
        yaw_rad = math.radians(self._yaw)
        self._loc = Location(
            self._loc.x + self._speed * math.cos(yaw_rad) * dt,
            self._loc.y + self._speed * math.sin(yaw_rad) * dt,
            self._loc.z,
        )

    def destroy(self):
        self.is_alive = False


class World:
    def __init__(self, actors=None):
        self._actors = ActorList(actors or [])
        self.debug = DebugHelper()

    def get_actors(self):
        return self._actors

    def get_spectator(self):
        return _NullSpectator()

    def tick(self):
        pass


class _NullSpectator:
    def set_transform(self, *args, **kwargs):
        pass
