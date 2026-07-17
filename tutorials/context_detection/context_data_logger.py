"""
context_data_logger.py — CARLA Feature/Label Logger for Context Detector Training
======================================================================================
Attach one ContextDataLogger to any scenario's tick loop (scenario 1-4, or a
dedicated data-collection run) to write one CSV row per tick: the feature
vector context_model.py expects, plus a weakly-supervised label.

IMPORTANT — where labels come from:
    There's no ground-truth "context" annotation in CARLA. The label here
    is derived programmatically from privileged map/traffic state (lane
    type tags, junction proximity, traffic light proximity, speed limit) —
    richer signal than the current rule-based ContextDetector uses (which
    only looks at speed limit + junction), but still a heuristic, not a
    human annotation. Training on it teaches the network to fuse more cues
    and generalize/smooth across a time window — it is not free lunch;
    it will not correct a labeling mistake baked into derive_label(). If
    you want stronger ground truth later, the cleanest upgrade is bucketing
    by known map zone (e.g. Town04's highway loop road IDs) instead of a
    per-tick rule — see the note at the bottom of this file.

Usage inside a scenario loop:
    logger = ContextDataLogger('output/context_data/run_001.csv')
    ...
    for tick in range(RUN_TICKS):
        world.tick()
        logger.log(world, carla_map, ego, tick, tick * TICK_DELTA, light=light)
    ...
    logger.close()

Author: Ahmad Ahmad | For: Nidhi's Autonomous Driving Course
"""

import csv
import math

import carla

from context_model import FEATURE_NAMES, CONTEXT_CLASSES

# ─── Encodings — keep in sync with context_model.py's comments ─────────────

_LANE_TYPE_MAP = {
    carla.LaneType.Driving: 0,
    carla.LaneType.Parking: 1,
    carla.LaneType.Shoulder: 2,
    carla.LaneType.Bidirectional: 3,
}

_LIGHT_STATE_MAP = {
    None: 0,
    carla.TrafficLightState.Red: 1,
    carla.TrafficLightState.Yellow: 2,
    carla.TrafficLightState.Green: 3,
}

# ─── Label heuristic thresholds ─────────────────────────────────────────────

JUNCTION_DIST_THRESH_M = 15.0
LIGHT_DIST_THRESH_M    = 25.0
HIGHWAY_SPEED_THRESH   = 50.0   # km/h
PARKING_SPEED_THRESH   = 20.0   # km/h

# ─── Sensing ranges ──────────────────────────────────────────────────────────

JUNCTION_LOOKAHEAD_M = 60.0
JUNCTION_STEP_M      = 3.0
CURVATURE_LOOKAHEAD_M = 15.0
LIGHT_SEARCH_RADIUS_M = 50.0
NEARBY_ACTOR_RADIUS_M  = 30.0
NO_JUNCTION_SENTINEL   = JUNCTION_LOOKAHEAD_M
NO_LIGHT_SENTINEL      = LIGHT_SEARCH_RADIUS_M


def derive_label(lane_type_code: int, is_junction: bool, dist_to_junction: float,
                  dist_to_light: float, speed_limit_kmh: float) -> str:
    """Weakly-supervised label from privileged map/traffic state. See the
    module docstring for the honesty caveat on what this is and isn't."""
    if lane_type_code == 1:  # Parking-tagged lane in the map
        return 'PARKING'
    if is_junction or dist_to_junction < JUNCTION_DIST_THRESH_M or dist_to_light < LIGHT_DIST_THRESH_M:
        return 'INTERSECTION'
    if speed_limit_kmh > HIGHWAY_SPEED_THRESH:
        return 'HIGHWAY'
    if speed_limit_kmh < PARKING_SPEED_THRESH:
        return 'PARKING'
    return 'CITY'


class ContextDataLogger:

    def __init__(self, filepath: str, route_id: str = None):
        """route_id: an identifier for this drive (e.g. 'town04_highway_01').
        Written into every row so train_context_model.py can split train/
        val/test BY ROUTE rather than by row — random row-splitting on a
        continuous drive leaks near-duplicate adjacent frames across splits."""
        self.filepath = filepath
        self.route_id = route_id or filepath
        self._file = open(filepath, 'w', newline='')
        self._writer = csv.writer(self._file)
        self._writer.writerow(FEATURE_NAMES + ['label', 'tick', 't', 'route_id'])
        self._prev_speed = None
        self._prev_yaw = None
        self._prev_t = None

    def log(self, world, carla_map, ego, tick: int, t: float, light=None):
        loc = ego.get_location()
        wp = carla_map.get_waypoint(loc)
        vel = ego.get_velocity()
        speed = math.hypot(vel.x, vel.y)
        yaw = ego.get_transform().rotation.yaw

        accel_long, yaw_rate = self._finite_difference(speed, yaw, t)

        speed_limit_kmh = ego.get_speed_limit()
        lane_width = getattr(wp, 'lane_width', 3.5)
        lane_type_code = _LANE_TYPE_MAP.get(getattr(wp, 'lane_type', None), -1)
        is_junction = 1.0 if wp.is_junction else 0.0
        dist_to_junction = self._dist_to_junction(wp)
        curvature = self._road_curvature(wp)

        dist_to_light, light_state_code = self._light_info(world, ego, light)
        num_vehicles, num_peds, avg_rel_speed = self._nearby_actors(world, ego, speed)

        features = [
            speed, accel_long, yaw_rate,
            speed_limit_kmh, lane_width, lane_type_code,
            is_junction, dist_to_junction, curvature,
            dist_to_light, light_state_code,
            num_vehicles, num_peds, avg_rel_speed,
        ]
        assert len(features) == len(FEATURE_NAMES), \
            "feature list drifted out of sync with FEATURE_NAMES in context_model.py"

        label = derive_label(lane_type_code, wp.is_junction, dist_to_junction,
                              dist_to_light, speed_limit_kmh)
        assert label in CONTEXT_CLASSES

        self._writer.writerow(features + [label, tick, t, self.route_id])

    def close(self):
        self._file.close()

    # ─── Feature helpers ────────────────────────────────────────────────────

    def _finite_difference(self, speed, yaw, t):
        if self._prev_t is None:
            self._prev_speed, self._prev_yaw, self._prev_t = speed, yaw, t
            return 0.0, 0.0
        dt = t - self._prev_t
        if dt <= 1e-6:
            return 0.0, 0.0
        accel_long = (speed - self._prev_speed) / dt
        dyaw = self._normalize_angle_deg(yaw - self._prev_yaw)
        yaw_rate = dyaw / dt
        self._prev_speed, self._prev_yaw, self._prev_t = speed, yaw, t
        return accel_long, yaw_rate

    @staticmethod
    def _normalize_angle_deg(angle_deg):
        while angle_deg > 180.0:
            angle_deg -= 360.0
        while angle_deg < -180.0:
            angle_deg += 360.0
        return angle_deg

    def _dist_to_junction(self, waypoint):
        """Walk forward along the road until a junction is hit, or give up
        at JUNCTION_LOOKAHEAD_M (returned as the sentinel 'far away')."""
        if waypoint.is_junction:
            return 0.0
        travelled = 0.0
        current = waypoint
        while travelled < JUNCTION_LOOKAHEAD_M:
            next_wps = current.next(JUNCTION_STEP_M)
            if not next_wps:
                break
            current = next_wps[0]
            travelled += JUNCTION_STEP_M
            if current.is_junction:
                return travelled
        return NO_JUNCTION_SENTINEL

    def _road_curvature(self, waypoint):
        """Heading change (degrees) between here and a point
        CURVATURE_LOOKAHEAD_M ahead — cheap proxy for road curvature."""
        ahead = waypoint.next(CURVATURE_LOOKAHEAD_M)
        if not ahead:
            return 0.0
        return self._normalize_angle_deg(
            ahead[0].transform.rotation.yaw - waypoint.transform.rotation.yaw)

    def _light_info(self, world, ego, light_hint):
        """If the scenario already knows the relevant light (light_hint),
        use it directly; otherwise search nearby lights, same pattern as
        ContextDetector.lightAhead() in planner.py."""
        ego_loc = ego.get_location()
        if light_hint is not None:
            dist = light_hint.get_location().distance(ego_loc)
            if dist <= LIGHT_SEARCH_RADIUS_M:
                return dist, _LIGHT_STATE_MAP.get(light_hint.get_state(), 0)

        lights = world.get_actors().filter('traffic.traffic_light')
        nearest, nearest_dist = None, float('inf')
        for l in lights:
            d = l.get_location().distance(ego_loc)
            if d < nearest_dist:
                nearest, nearest_dist = l, d
        if nearest is not None and nearest_dist <= LIGHT_SEARCH_RADIUS_M:
            return nearest_dist, _LIGHT_STATE_MAP.get(nearest.get_state(), 0)
        return NO_LIGHT_SENTINEL, 0

    def _nearby_actors(self, world, ego, ego_speed):
        ego_loc = ego.get_location()
        ego_id = ego.id

        vehicles = [a for a in world.get_actors().filter('vehicle.*') if a.id != ego_id]
        peds = list(world.get_actors().filter('walker.pedestrian.*'))

        nearby_vehicles = [v for v in vehicles if v.get_location().distance(ego_loc) <= NEARBY_ACTOR_RADIUS_M]
        nearby_peds = [p for p in peds if p.get_location().distance(ego_loc) <= NEARBY_ACTOR_RADIUS_M]

        if nearby_vehicles:
            rel_speeds = []
            for v in nearby_vehicles:
                vel = v.get_velocity()
                v_speed = math.hypot(vel.x, vel.y)
                rel_speeds.append(v_speed - ego_speed)
            avg_rel_speed = sum(rel_speeds) / len(rel_speeds)
        else:
            avg_rel_speed = 0.0

        return len(nearby_vehicles), len(nearby_peds), avg_rel_speed


# ─── Note: stronger ground truth via map-zone bucketing ─────────────────────
#
# If per-tick heuristic labels feel too noisy once you look at the data,
# the cleanest upgrade is NOT a fancier rule — it's bucketing by known map
# geography instead of inferring context from live state at all:
#
#   TOWN04_HIGHWAY_ROAD_IDS = {12, 13, 14, ...}   # from town04.xodr
#   TOWN05_INTERSECTION_JUNCTION_IDS = {3, 7, 9}  # from town05.xodr
#
#   def label_from_zone(wp, town_name):
#       road_id = wp.road_id
#       if town_name == 'Town04' and road_id in TOWN04_HIGHWAY_ROAD_IDS:
#           return 'HIGHWAY'
#       ...
#
# This trades generality (works only for towns you've hand-mapped) for
# near-perfect label quality, which can be worth it for a clean ICRA
# result — you can even use it as a second, cross-checking label source
# and report inter-labeler agreement between the two heuristics.
