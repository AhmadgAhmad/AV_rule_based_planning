"""
planner.py — Hierarchical Planner for CARLA Intersection Scenarios
===================================================================

Pipeline:
    SamplingTrajectoryGenerator
        ↓  N candidate trajectories (parameterized by speed, lateral offset)
    TWTLEvaluator
        ↓  computes ρ_P0, ρ_P1, η_P2 for each trajectory
    ContextDetector
        ↓  determines driving context (INTERSECTION, HIGHWAY, CITY, PARKING)
    HierarchicalPlanner
        ↓  Stage 1: filter on P0 (safety)
        ↓  Stage 2: filter on P1 (legal)
        ↓  Stage 3: rank by P2 (soft, context-aware AGM robustness)
    PathExecutor
        ↓  converts best trajectory → VehicleControl commands (throttle/steer/brake)

Author: Ahmad Ahmad | For: Nidhi's Autonomous Driving Course
"""

import carla
import math
import numpy as np
from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional, Tuple

from pid_controller import VehiclePIDController


# ═══════════════════════════════════════════════════════════════════════════════
# DATA STRUCTURES
# ═══════════════════════════════════════════════════════════════════════════════

class Context(Enum):
    INTERSECTION = "intersection"
    HIGHWAY      = "highway"
    CITY         = "city"
    PARKING      = "parking"


@dataclass
class Trajectory:
    """A single candidate trajectory produced by the generator."""
    waypoints:      List[carla.Location]   # geometric path on the road
    target_speeds:  List[float]            # target speed (m/s) at each waypoint
    speed_factor:   float                  # α — fraction of speed limit used
    lateral_offset: float                  # β — meters from lane center

    # Set by TWTLEvaluator
    rho_p0: float = -999.0    # safety robustness (P0)
    rho_p1: float = -999.0    # legal robustness  (P1)
    eta_p2: float = -999.0    # soft AGM robustness (P2, context-aware)

    # Diagnostics
    p0_reason: str = ""
    p1_reason: str = ""

    @property
    def total_length(self) -> float:
        total = 0.0
        for i in range(1, len(self.waypoints)):
            total += self.waypoints[i].distance(self.waypoints[i-1])
        return total


# ═══════════════════════════════════════════════════════════════════════════════
# SAMPLING TRAJECTORY GENERATOR
# ═══════════════════════════════════════════════════════════════════════════════

class SamplingTrajectoryGenerator:
    """
    Generates N candidate trajectories by varying speed and lateral offset.

    Each trajectory follows CARLA's road network (via waypoint.next()),
    so all paths are guaranteed to stay on valid roads.

    Parameterization:
        speed_factor α ∈ SPEED_FACTORS  — fraction of speed limit
        lateral_offset β ∈ LATERAL_OFFSETS — meters from lane center
    """

    SPEED_FACTORS    = [0.0, 0.3, 0.5, 0.7, 0.9, 1.0]   # 0.0 = full stop
    LATERAL_OFFSETS  = [-0.4, -0.2, 0.0, 0.2, 0.4]       # meters from center
    WAYPOINT_STEP    = 2.0     # meters between waypoints
    HORIZON          = 30      # waypoints per trajectory

    def __init__(self, carla_map):
        self.carla_map = carla_map

    def generate(self, ego: carla.Actor, speed_limit_ms: float) -> List[Trajectory]:
        """
        Generate all candidate trajectories from current ego position.
        Returns list of Trajectory objects (waypoints + target speeds).
        """
        trajectories = []
        start_wp = self.carla_map.get_waypoint(ego.get_location())

        for alpha in self.SPEED_FACTORS:
            for beta in self.LATERAL_OFFSETS:
                traj = self._build_trajectory(start_wp, alpha, beta, speed_limit_ms)
                if traj is not None:
                    trajectories.append(traj)

        print(f"[Generator] Generated {len(trajectories)} candidate trajectories "
              f"({len(self.SPEED_FACTORS)} speeds × {len(self.LATERAL_OFFSETS)} offsets)")
        return trajectories

    def _build_trajectory(self,
                          start_wp: carla.Waypoint,
                          speed_factor: float,
                          lateral_offset: float,
                          speed_limit_ms: float) -> Optional[Trajectory]:
        """
        Build one trajectory by following road waypoints with a speed profile
        and a constant lateral offset from the lane center.
        """
        waypoints     = []
        target_speeds = []
        current_wp    = start_wp

        target_speed = speed_factor * speed_limit_ms   # m/s

        for _ in range(self.HORIZON):
            # Apply lateral offset: shift location perpendicular to lane direction
            location = self._apply_lateral_offset(current_wp, lateral_offset)
            waypoints.append(location)
            target_speeds.append(target_speed)

            # Advance along road
            next_wps = current_wp.next(self.WAYPOINT_STEP)
            if not next_wps:
                break
            current_wp = next_wps[0]   # take straight-ahead continuation

        if len(waypoints) < 3:
            return None   # trajectory too short to be useful

        return Trajectory(
            waypoints      = waypoints,
            target_speeds  = target_speeds,
            speed_factor   = speed_factor,
            lateral_offset = lateral_offset,
        )

    def _apply_lateral_offset(self,
                               waypoint: carla.Waypoint,
                               offset: float) -> carla.Location:
        """
        Shift waypoint location perpendicular to the lane direction by `offset` meters.
        Positive offset = right of lane center, negative = left.
        """
        yaw_rad = math.radians(waypoint.transform.rotation.yaw)

        # Perpendicular direction (rotate lane forward vector by 90°)
        perp_x = -math.sin(yaw_rad)
        perp_y =  math.cos(yaw_rad)

        loc = waypoint.transform.location
        return carla.Location(
            x = loc.x + offset * perp_x,
            y = loc.y + offset * perp_y,
            z = loc.z
        )


# ═══════════════════════════════════════════════════════════════════════════════
# CONTEXT DETECTOR
# ═══════════════════════════════════════════════════════════════════════════════

class ContextDetector:
    """
    Detects the current driving context from world state.

    Context determines Stage 3 soft-preference weights:
        INTERSECTION  → comfort > efficiency (careful, smooth)
        HIGHWAY       → efficiency > comfort (fast, steady)
        CITY          → balanced
        PARKING       → ultra-slow, precision lateral
    """
    # TODO [Nidhi]: this is a very basic implementation. In a real system, context detection
    # would be more sophisticated and use richer information (e.g. traffic light state, nearby actors, lane markings, etc.) rather than just speed limit and junction presence.
    
    JUNCTION_LOOKAHEAD   = 25.0   # metres — how far ahead to look for junctions
    HIGHWAY_SPEED_LIMIT  = 50.0   # km/h — threshold for highway classification
    PARKING_SPEED_LIMIT  = 20.0   # km/h

    # If there is a light detected, also treat it as an intersection scenario
    def lightAhead(self, ego, world, radius=30):
        lights = world.get_actors().filter('traffic.traffic_light')
        egoLoc = ego.get_location()
        for light in lights:
            if light.get_location().distance(egoLoc) < radius:
                return True
        return False


    def detect(self,
               ego: carla.Actor,
               carla_map,
               world: carla.World) -> Context:
        """Detect context from current ego state and surrounding world."""

        location     = ego.get_location()
        waypoint     = carla_map.get_waypoint(location)
        speed_limit  = self._get_speed_limit(ego, carla_map)

        # Priority order: most specific first
        if self._is_at_or_approaching_intersection(waypoint) or self.lightAhead(ego, world):
            ctx = Context.INTERSECTION
        elif speed_limit > self.HIGHWAY_SPEED_LIMIT:
            ctx = Context.HIGHWAY
        elif speed_limit < self.PARKING_SPEED_LIMIT:
            ctx = Context.PARKING
        else:
            ctx = Context.CITY

        print(f"[Context] Detected: {ctx.value} "
              f"(junction={waypoint.is_junction}, speed_limit={speed_limit:.0f} km/h)")
        return ctx

    def _is_at_or_approaching_intersection(self, waypoint: carla.Waypoint) -> bool:
        """True if currently in a junction OR a junction is within lookahead distance."""
        if waypoint.is_junction:
            return True

        # Look ahead along road
        lookahead = self.JUNCTION_LOOKAHEAD
        step      = 3.0
        current   = waypoint

        while lookahead > 0:
            next_wps = current.next(step)
            if not next_wps:
                break
            current   = next_wps[0]
            lookahead -= step
            if current.is_junction:
                return True

        return False

    def _get_speed_limit(self, ego: carla.Actor, carla_map) -> float:
        """Get current road speed limit in km/h."""
        waypoint     = carla_map.get_waypoint(ego.get_location())
        # CARLA exposes speed_limit on the actor directly
        speed_limit  = ego.get_speed_limit()   # km/h
        return speed_limit if speed_limit > 0 else 30.0


# ═══════════════════════════════════════════════════════════════════════════════
# TWTL EVALUATOR  (P0 / P1 / P2)
# ═══════════════════════════════════════════════════════════════════════════════

class TWTLEvaluator:
    """
    Evaluates each trajectory against the three TWTL priority levels.

    P0 (Safety)   — hard filter, traditional ρ
        H_T ¬π_collision  ∧  H_T ¬π_ped_collision

    P1 (Legal)    — hard filter, traditional ρ
        H_T π_speed_ok  ∧  H_T ¬π_red_entry

    P2 (Soft)     — ranking, AGM robustness η
        Context-weighted combination of comfort, efficiency, safety margin
    """
    # TODO [Nidhi]: this is a good place to learn about TWTL i practice -- recall our last session on Temporal Logics.
    # P0 thresholds
    OBSTACLE_SAFETY_MARGIN  = 1.5    # metres clearance required
    PED_SAFETY_MARGIN       = 2.5    # metres clearance from pedestrians

    # P1 thresholds
    SPEED_LIMIT_TOLERANCE   = 1.5    # m/s over limit allowed (small buffer)

    def evaluate_all(self,
                     trajectories: List[Trajectory],
                     world: carla.World,
                     carla_map,
                     light: Optional[carla.Actor],
                     speed_limit_ms: float,
                     context: Context,
                     ego_id: int = -1) -> List[Trajectory]:
        """
        Compute rho_p0, rho_p1, eta_p2 for every trajectory.
        Modifies trajectory objects in-place. Returns same list.

        ego_id: the actor ID of the ego vehicle — excluded from obstacle check.
                Without this, the ego counts as its own obstacle and P0 rejects
                every trajectory immediately (distance = 0 < safety margin).
        """
        # Pre-fetch world actors once (expensive CARLA call)
        # IMPORTANT: filter out ego from obstacles — it is not an obstacle to itself
        obstacles    = [a for a in world.get_actors().filter('vehicle.*')
                        if a.id != ego_id]
        pedestrians  = list(world.get_actors().filter('walker.pedestrian.*'))
        light_state  = light.get_state() if light else None

        for traj in trajectories:
            traj.rho_p0, traj.p0_reason = self._evaluate_p0(
                traj, obstacles, pedestrians)
            traj.rho_p1, traj.p1_reason = self._evaluate_p1(
                traj, light_state, speed_limit_ms, carla_map)
            traj.eta_p2 = self._evaluate_p2(
                traj, context, speed_limit_ms)

        return trajectories

    # ─── P0: Safety ──────────────────────────────────────────────────────────

    def _evaluate_p0(self,
                     traj: Trajectory,
                     obstacles: list,
                     pedestrians: list) -> Tuple[float, str]:
        """
        ρ_P0 = min over all waypoints of (distance to nearest obstacle - margin).
        Negative → safety violation.
        """
        min_obstacle_dist = float('inf')
        min_ped_dist      = float('inf')

        for wp_loc in traj.waypoints:
            # Check vehicles
            for obs in obstacles:
                d = wp_loc.distance(obs.get_location())
                if d < min_obstacle_dist:
                    min_obstacle_dist = d

            # Check pedestrians (tighter margin)
            for ped in pedestrians:
                d = wp_loc.distance(ped.get_location())
                if d < min_ped_dist:
                    min_ped_dist = d

        rho_obstacle = min_obstacle_dist - self.OBSTACLE_SAFETY_MARGIN
        rho_ped      = min_ped_dist      - self.PED_SAFETY_MARGIN

        # Conjunction: worst of the two
        rho = min(rho_obstacle, rho_ped)

        if rho < 0:
            reason = ("too close to pedestrian" if rho_ped < rho_obstacle
                      else "too close to obstacle")
        else:
            reason = "safe"

        return rho, reason

    # ─── P1: Legal Compliance ────────────────────────────────────────────────

    def _evaluate_p1(self,
                     traj: Trajectory,
                     light_state,
                     speed_limit_ms: float,
                     carla_map) -> Tuple[float, str]:
        """
        ρ_P1 = min of speed compliance and red-light compliance.
        """
        # Speed check: max target speed must not exceed limit + tolerance
        max_speed = max(traj.target_speeds)
        rho_speed = (speed_limit_ms + self.SPEED_LIMIT_TOLERANCE) - max_speed

        # Red-light check: if light is RED, trajectory must not enter junction
        rho_light = 1.0   # default: compliant
        if light_state == carla.TrafficLightState.Red:
            for wp_loc in traj.waypoints:
                wp = carla_map.get_waypoint(wp_loc)
                if wp.is_junction:
                    rho_light = -1.0
                    break

        rho = min(rho_speed, rho_light)

        if rho < 0:
            reason = ("runs red light" if rho_light < 0 else "exceeds speed limit")
        else:
            reason = "legal"

        return rho, reason

    # ─── P2: Soft Preferences (AGM Robustness η) ─────────────────────────────

    def _evaluate_p2(self,
                     traj: Trajectory,
                     context: Context,
                     speed_limit_ms: float) -> float:
        """
        η_P2 = AGM-weighted combination of comfort, efficiency, safety_margin.

        Context-specific weights (sum to 1.0):
            INTERSECTION : comfort=0.5, safety_margin=0.3, efficiency=0.2
            HIGHWAY      : efficiency=0.5, safety_margin=0.3, comfort=0.2
            CITY         : comfort=0.4, safety_margin=0.3, efficiency=0.3
            PARKING      : safety_margin=0.5, comfort=0.4, efficiency=0.1
        """
        WEIGHTS = {
            Context.INTERSECTION: dict(comfort=0.5, safety_margin=0.3, efficiency=0.2),
            Context.HIGHWAY:      dict(efficiency=0.5, safety_margin=0.3, comfort=0.2),
            Context.CITY:         dict(comfort=0.4, safety_margin=0.3, efficiency=0.3),
            Context.PARKING:      dict(safety_margin=0.5, comfort=0.4, efficiency=0.1),
        }

        w = WEIGHTS[context]

        # Comfort: penalise large lateral offset (smoother is better)
        max_offset  = max(SamplingTrajectoryGenerator.LATERAL_OFFSETS)
        comfort     = 1.0 - abs(traj.lateral_offset) / max_offset   # ∈ [0, 1]

        # Efficiency: reward higher speed relative to limit
        avg_speed   = np.mean(traj.target_speeds)
        efficiency  = avg_speed / max(speed_limit_ms, 1e-6)         # ∈ [0, 1]

        # Safety margin: normalise rho_p0 to [-1, 1]
        safety_margin = max(-1.0, min(1.0, traj.rho_p0 / 10.0))

        raw_scores = {
            'comfort':        comfort,
            'efficiency':     efficiency,
            'safety_margin':  safety_margin,
        }

        # Weighted arithmetic mean (all values in [0,1])
        eta = sum(w[k] * raw_scores[k] for k in w)
        return eta


# ═══════════════════════════════════════════════════════════════════════════════
# HIERARCHICAL PLANNER  (3-stage filter)
# ═══════════════════════════════════════════════════════════════════════════════

class HierarchicalPlanner:
    """
    Three-stage lexicographic filter:

        Stage 1  →  Remove trajectories with ρ_P0 < 0  (safety violation)
        Stage 2  →  Remove trajectories with ρ_P1 < 0  (legal violation)
        Stage 3  →  Select trajectory with highest η_P2 (soft, context-aware)

    Fallback policy:
        If Stage 1 removes everything  →  emergency brake (zero-speed trajectory)
        If Stage 2 removes everything  →  best P0-safe trajectory (least legal harm)
    """

    def __init__(self, carla_map):
        self.generator  = SamplingTrajectoryGenerator(carla_map)
        self.detector   = ContextDetector()
        self.evaluator  = TWTLEvaluator()
        self.carla_map  = carla_map

    def plan(self,
             ego:           carla.Actor,
             world:         carla.World,
             light:         Optional[carla.Actor],
             speed_limit_ms: float) -> Trajectory:
        """
        Full planning cycle. Call once per control step (or at configurable Hz).
        Returns the best trajectory to execute.
        """

        # ── 0. Detect context ────────────────────────────────────────────────
        context = self.detector.detect(ego, self.carla_map, world)

        # ── 1. Sample candidate trajectories ────────────────────────────────
        candidates = self.generator.generate(ego, speed_limit_ms)

        # ── 2. Evaluate all trajectories (P0, P1, P2) ───────────────────────
        candidates = self.evaluator.evaluate_all(
            candidates, world, self.carla_map, light, speed_limit_ms, context,
            ego_id=ego.id   # exclude ego from its own obstacle check
        )

        # ── Stage 1: P0 filter ───────────────────────────────────────────────
        p0_safe = [t for t in candidates if t.rho_p0 >= 0]

        if not p0_safe:
            print("[Planner] Stage 1: ALL trajectories unsafe — EMERGENCY BRAKE")
            return self._emergency_brake_trajectory(ego), candidates

        print(f"[Planner] Stage 1 (P0): {len(candidates)} → {len(p0_safe)} safe trajectories")

        # ── Stage 2: P1 filter ───────────────────────────────────────────────
        p1_legal = [t for t in p0_safe if t.rho_p1 >= 0]

        if not p1_legal:
            # Fallback: choose the P0-safe trajectory that least violates P1
            print(f"[Planner] Stage 2 (P1): All remaining violate legal — using least-harmful")
            p1_legal = [max(p0_safe, key=lambda t: t.rho_p1)]
        else:
            print(f"[Planner] Stage 2 (P1): {len(p0_safe)} → {len(p1_legal)} legal trajectories")

        # ── Stage 3: P2 ranking (context-aware soft preferences) ────────────
        best = max(p1_legal, key=lambda t: t.eta_p2)

        print(f"[Planner] Stage 3 (P2, ctx={context.value}): "
              f"selected α={best.speed_factor:.1f}, β={best.lateral_offset:+.2f}m | "
              f"ρ_P0={best.rho_p0:.2f} ρ_P1={best.rho_p1:.2f} η_P2={best.eta_p2:.3f}")

        return best, candidates

    def _emergency_brake_trajectory(self, ego: carla.Actor) -> Trajectory:
        """Return a single-waypoint stop-in-place trajectory."""
        loc = ego.get_location()
        return Trajectory(
            waypoints      = [loc],
            target_speeds  = [0.0],
            speed_factor   = 0.0,
            lateral_offset = 0.0,
            rho_p0         = 0.0,
            rho_p1         = 0.0,
            eta_p2         = 0.0,
            p0_reason      = "emergency brake",
        )


# ═══════════════════════════════════════════════════════════════════════════════
# PATH EXECUTOR
# ═══════════════════════════════════════════════════════════════════════════════

class PathExecutor:
    """
    Converts a Trajectory into VehicleControl commands using a proper PID
    controller (longitudinal speed PID + lateral heading-error PID), instead
    of the earlier proportional-only law. See pid_controller.py.

    Call step() once per simulation tick, advancing one waypoint at a time.
    """

    WAYPOINT_REACH_RADIUS = 3.0    # metres — when to advance to next waypoint

    # PID gains — tuned for 20 Hz (dt=0.05s) fixed timestep. If REPLAN_INTERVAL
    # or TICK_DELTA changes in a scenario, pass a matching `dt` through.
    LONGITUDINAL_PID = dict(K_P=1.0, K_I=0.05, K_D=0.05)
    LATERAL_PID      = dict(K_P=1.2, K_I=0.02, K_D=0.15)

    def __init__(self, dt: float = 0.05):
        self.current_wp_index = 0
        self.current_traj     = None
        self.controller = VehiclePIDController(
            dt=dt,
            args_longitudinal=self.LONGITUDINAL_PID,
            args_lateral=self.LATERAL_PID,
        )

    def set_trajectory(self, traj: Trajectory):
        """Load a new trajectory, reset waypoint index, and clear PID error
        history — a new trajectory means a new tracking problem, so stale
        integral/derivative terms from the last one should not carry over."""
        self.current_traj     = traj
        self.current_wp_index = 0
        self.controller.reset()
        print(f"[Executor] New trajectory loaded: {len(traj.waypoints)} waypoints, "
              f"speeds {min(traj.target_speeds):.1f}–{max(traj.target_speeds):.1f} m/s")

    def step(self, ego: carla.Actor) -> carla.VehicleControl:
        """
        Compute VehicleControl for this tick.
        Advances to next waypoint when within WAYPOINT_REACH_RADIUS.
        """
        if self.current_traj is None or self.current_wp_index >= len(self.current_traj.waypoints):
            return self._brake_control()

        traj       = self.current_traj
        target_loc = traj.waypoints[self.current_wp_index]
        target_spd = traj.target_speeds[self.current_wp_index]

        ego_loc = ego.get_location()

        # ── Advance waypoint ─────────────────────────────────────────────────
        dist_to_wp = ego_loc.distance(target_loc)
        if dist_to_wp < self.WAYPOINT_REACH_RADIUS:
            self.current_wp_index += 1
            if self.current_wp_index >= len(traj.waypoints):
                print(f"[Executor] Trajectory complete.")
                return self._brake_control()
            target_loc = traj.waypoints[self.current_wp_index]
            target_spd = traj.target_speeds[self.current_wp_index]

        return self.controller.run_step(target_spd, target_loc, ego)

    @staticmethod
    def _brake_control() -> carla.VehicleControl:
        return carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0)


# ═══════════════════════════════════════════════════════════════════════════════
# ROBUSTNESS LOGGER
# ═══════════════════════════════════════════════════════════════════════════════

class RobustnessLogger:
    """Accumulates per-tick TWTL robustness values and prints a final report."""

    def __init__(self, scenario_name: str):
        self.scenario_name = scenario_name
        self.log = []

    def record(self, tick: int, t: float, best_traj: Trajectory):
        self.log.append({
            't':     t,
            'tick':  tick,
            'rho_p0': best_traj.rho_p0,
            'rho_p1': best_traj.rho_p1,
            'eta_p2': best_traj.eta_p2,
            'speed_factor':   best_traj.speed_factor,
            'lateral_offset': best_traj.lateral_offset,
        })

    def report(self):
        if not self.log:
            return

        min_p0 = min(r['rho_p0'] for r in self.log)
        min_p1 = min(r['rho_p1'] for r in self.log)
        avg_p2 = sum(r['eta_p2'] for r in self.log) / len(self.log)

        # Overall TWTL robustness = min of hard constraints
        rho_overall = min(min_p0, min_p1)

        print(f"\n{'═'*55}")
        print(f"  TWTL ROBUSTNESS REPORT — {self.scenario_name}")
        print(f"{'═'*55}")
        print(f"  min ρ_P0 (safety)   =  {min_p0:+.3f}")
        print(f"  min ρ_P1 (legal)    =  {min_p1:+.3f}")
        print(f"  avg η_P2 (soft)     =  {avg_p2:+.3f}")
        print(f"  ─────────────────────────────────────────────")
        print(f"  ρ_overall           =  {rho_overall:+.3f}")
        print(f"  Result : {'PASSED ✓' if rho_overall > 0 else 'FAILED ✗'}")
        print(f"{'═'*55}\n")
