"""
test_offline.py — Offline Sanity Tests (no CARLA server required)
=======================================================================
Run with:
    python3 test_offline.py -v

Covers the two files that don't need a live simulator to be meaningfully
tested — pid_controller.py and planner.py's evaluation/filtering logic.
The scenario_*.py files spawn real actors and drive a real world, so they
still need CARLA running; this suite is for catching math/logic bugs
*before* you spend time booting the simulator.

What's covered:
    1. PIDLongitudinalController — converges to target speed; the integral
       term specifically is shown to remove steady-state error that a
       P-only controller leaves behind under a disturbance (drag).
    2. PIDLateralController       — steer sign matches which side the
       target is on, and goes to ~0 when the target is dead ahead.
    3. VehiclePIDController + toy kinematics — the ego actually converges
       toward a target waypoint over simulated time.
    4. TWTLEvaluator P0 (safety)  — rho_p0 goes negative near an obstacle,
       stays non-negative when clear.
    5. TWTLEvaluator P1 (legal)   — rho_p1 goes negative on red-light entry
       into a junction and on speeding, stays non-negative otherwise.
    6. TWTLEvaluator P2 (soft)    — INTERSECTION context ranks the smoother/
       slower trajectory higher; HIGHWAY context flips that preference
       toward the faster one, matching the docstring claims in
       scenario_4_highway_traffic.py.
    7. HierarchicalPlanner stage filtering — emergency brake when all P0
       fail, least-harmful fallback when all P1 fail, correct max-eta pick
       in the normal case. Generator/detector/evaluator are monkeypatched
       with controlled fixtures so this tests only the filter logic itself.
    8. PathExecutor — full trajectory tracking loop against the toy Actor,
       confirms the executor advances through waypoints and reaches the end.

Author: Ahmad Ahmad | For: Nidhi's Autonomous Driving Course
"""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(__file__))

import mock_carla
sys.modules['carla'] = mock_carla   # must happen before importing planner/pid_controller
import carla  # noqa: E402  (now resolves to mock_carla)

from pid_controller import (PIDLongitudinalController, PIDLateralController,
                             VehiclePIDController)  # noqa: E402
from planner import (Trajectory, TWTLEvaluator, Context, HierarchicalPlanner,
                      PathExecutor, SamplingTrajectoryGenerator)  # noqa: E402


# ═══════════════════════════════════════════════════════════════════════════
# 1–3. PID CONTROLLERS
# ═══════════════════════════════════════════════════════════════════════════

class TestPIDLongitudinal(unittest.TestCase):

    def _simulate(self, controller, target_speed, drag_coeff, steps=400, dt=0.05):
        speed = 0.0
        for _ in range(steps):
            out = controller.run_step(target_speed, speed)
            accel_cmd = max(out, 0.0) * 3.0 - max(-out, 0.0) * 3.0
            speed = max(0.0, speed + (accel_cmd - drag_coeff * speed) * dt)
        return speed

    def test_reaches_target_speed_no_disturbance(self):
        pid = PIDLongitudinalController(K_P=1.0, K_I=0.05, K_D=0.05, dt=0.05)
        final_speed = self._simulate(pid, target_speed=10.0, drag_coeff=0.0)
        self.assertAlmostEqual(final_speed, 10.0, delta=0.3)

    def test_integral_term_removes_steady_state_error_under_drag(self):
        # P-only leaves a steady-state gap under a constant disturbance
        # (drag proportional to speed) — this is exactly why PID > P.
        p_only = PIDLongitudinalController(K_P=1.0, K_I=0.0, K_D=0.0, dt=0.05)
        pid = PIDLongitudinalController(K_P=1.0, K_I=0.15, K_D=0.05, dt=0.05)

        p_only_speed = self._simulate(p_only, target_speed=10.0, drag_coeff=0.3, steps=600)
        pid_speed = self._simulate(pid, target_speed=10.0, drag_coeff=0.3, steps=600)

        p_only_error = abs(10.0 - p_only_speed)
        pid_error = abs(10.0 - pid_speed)

        self.assertGreater(p_only_error, 0.5,
                            "expected P-only to leave a visible steady-state gap under drag")
        self.assertLess(pid_error, 0.3,
                         "expected the integral term to close most of that gap")
        self.assertLess(pid_error, p_only_error)


class TestPIDLateral(unittest.TestCase):

    def test_target_dead_ahead_gives_near_zero_steer(self):
        pid = PIDLateralController(K_P=1.2, K_I=0.0, K_D=0.0, dt=0.05)
        transform = carla.Transform(carla.Location(0, 0, 0), carla.Rotation(yaw=0))
        target = carla.Location(10, 0, 0)
        steer = pid.run_step(target, transform)
        self.assertAlmostEqual(steer, 0.0, delta=0.05)

    def test_target_to_the_left_and_right_have_opposite_sign(self):
        pid_left = PIDLateralController(K_P=1.2, K_I=0.0, K_D=0.0, dt=0.05)
        pid_right = PIDLateralController(K_P=1.2, K_I=0.0, K_D=0.0, dt=0.05)
        transform = carla.Transform(carla.Location(0, 0, 0), carla.Rotation(yaw=0))

        steer_left = pid_left.run_step(carla.Location(10, 5, 0), transform)   # target ahead-left
        steer_right = pid_right.run_step(carla.Location(10, -5, 0), transform)  # ahead-right

        self.assertGreater(steer_left * steer_right, -100)  # sanity: both finite
        self.assertNotEqual(math.copysign(1, steer_left), math.copysign(1, steer_right))


class TestVehiclePIDControllerConvergence(unittest.TestCase):

    def test_ego_converges_toward_target_waypoint(self):
        ego = carla.Actor(x=0.0, y=0.0, yaw=0.0)
        controller = VehiclePIDController(dt=0.05)
        target_loc = carla.Location(30.0, 5.0, 0.0)
        target_speed = 8.0

        initial_dist = ego.get_location().distance(target_loc)
        for _ in range(300):
            control = controller.run_step(target_speed, target_loc, ego)
            ego.apply_control(control, dt=0.05)

        final_dist = ego.get_location().distance(target_loc)
        self.assertLess(final_dist, initial_dist * 0.15,
                         "expected the PID-driven ego to close most of the distance "
                         "to the target waypoint")


# ═══════════════════════════════════════════════════════════════════════════
# 4–6. TWTL EVALUATOR (P0 / P1 / P2)
# ═══════════════════════════════════════════════════════════════════════════

def straight_trajectory(speed_factor=0.5, lateral_offset=0.0, n=10, step=2.0,
                         speed_limit_ms=13.9):
    waypoints = [carla.Location(i * step, lateral_offset, 0.0) for i in range(n)]
    target_speed = speed_factor * speed_limit_ms
    return Trajectory(
        waypoints=waypoints,
        target_speeds=[target_speed] * n,
        speed_factor=speed_factor,
        lateral_offset=lateral_offset,
    )


class TestTWTLEvaluatorP0(unittest.TestCase):

    def setUp(self):
        self.evaluator = TWTLEvaluator()

    def test_rejects_trajectory_too_close_to_obstacle(self):
        traj = straight_trajectory()
        obstacle = carla.Actor(x=4.0, y=0.0)  # sitting right on the path
        rho, reason = self.evaluator._evaluate_p0(traj, obstacles=[obstacle], pedestrians=[])
        self.assertLess(rho, 0.0)
        self.assertIn("obstacle", reason)

    def test_accepts_trajectory_clear_of_obstacles(self):
        traj = straight_trajectory()
        obstacle = carla.Actor(x=200.0, y=200.0)  # far away
        rho, reason = self.evaluator._evaluate_p0(traj, obstacles=[obstacle], pedestrians=[])
        self.assertGreaterEqual(rho, 0.0)
        self.assertEqual(reason, "safe")

    def test_pedestrian_margin_is_tighter_than_vehicle_margin(self):
        traj = straight_trajectory()
        # place a pedestrian just inside the 2.5m ped margin but outside the
        # 1.5m vehicle margin — should still trip P0.
        ped = carla.Actor(x=4.0, y=2.0, type_id='walker.pedestrian.0001')
        rho, reason = self.evaluator._evaluate_p0(traj, obstacles=[], pedestrians=[ped])
        self.assertLess(rho, 0.0)
        self.assertIn("pedestrian", reason)


class TestTWTLEvaluatorP1(unittest.TestCase):

    def setUp(self):
        self.evaluator = TWTLEvaluator()
        self.carla_map = mock_carla.Map(junction_x_range=(10.0, 30.0))

    def test_rejects_entering_junction_on_red(self):
        traj = straight_trajectory(speed_factor=0.5)  # waypoints reach x=18, inside junction
        rho, reason = self.evaluator._evaluate_p1(
            traj, light_state=carla.TrafficLightState.Red,
            speed_limit_ms=13.9, carla_map=self.carla_map)
        self.assertLess(rho, 0.0)
        self.assertEqual(reason, "runs red light")

    def test_allows_junction_entry_on_green(self):
        traj = straight_trajectory(speed_factor=0.5)
        rho, reason = self.evaluator._evaluate_p1(
            traj, light_state=carla.TrafficLightState.Green,
            speed_limit_ms=13.9, carla_map=self.carla_map)
        self.assertGreaterEqual(rho, 0.0)

    def test_rejects_exceeding_speed_limit(self):
        traj = straight_trajectory(speed_factor=1.0, speed_limit_ms=13.9)
        traj.target_speeds = [30.0] * len(traj.target_speeds)  # way over
        rho, reason = self.evaluator._evaluate_p1(
            traj, light_state=None, speed_limit_ms=13.9, carla_map=self.carla_map)
        self.assertLess(rho, 0.0)
        self.assertEqual(reason, "exceeds speed limit")


class TestTWTLEvaluatorP2(unittest.TestCase):

    def setUp(self):
        self.evaluator = TWTLEvaluator()
        # Comfort in this evaluator is driven purely by |lateral_offset|
        # (see TWTLEvaluator._evaluate_p2), independent of speed. So the two
        # fixtures vary offset (comfort) and speed_factor (efficiency)
        # together, in opposite directions, to make the context weighting
        # visible:
        max_offset = max(SamplingTrajectoryGenerator.LATERAL_OFFSETS)
        self.efficient_traj = straight_trajectory(speed_factor=1.0, lateral_offset=max_offset)
        self.comfortable_traj = straight_trajectory(speed_factor=0.3, lateral_offset=0.0)
        for t in (self.efficient_traj, self.comfortable_traj):
            t.rho_p0 = 5.0  # equal, generous safety margin for both

    def test_intersection_context_prefers_comfort_over_efficiency(self):
        eta_efficient = self.evaluator._evaluate_p2(self.efficient_traj, Context.INTERSECTION, 13.9)
        eta_comfortable = self.evaluator._evaluate_p2(self.comfortable_traj, Context.INTERSECTION, 13.9)
        self.assertGreater(eta_comfortable, eta_efficient,
                            "INTERSECTION weights comfort=0.5 over efficiency=0.2 — "
                            "the centered, slower trajectory should score higher")

    def test_highway_context_prefers_efficiency_over_comfort(self):
        eta_efficient = self.evaluator._evaluate_p2(self.efficient_traj, Context.HIGHWAY, 13.9)
        eta_comfortable = self.evaluator._evaluate_p2(self.comfortable_traj, Context.HIGHWAY, 13.9)
        self.assertGreater(eta_efficient, eta_comfortable,
                            "HIGHWAY weights efficiency=0.5 over comfort=0.2 — this should "
                            "flip the ranking relative to INTERSECTION context")


# ═══════════════════════════════════════════════════════════════════════════
# 7. HIERARCHICAL PLANNER — STAGE FILTERING
# ═══════════════════════════════════════════════════════════════════════════

class TestHierarchicalPlannerStages(unittest.TestCase):

    def setUp(self):
        self.carla_map = mock_carla.Map()
        self.planner = HierarchicalPlanner(self.carla_map)
        self.ego = carla.Actor(x=0.0, y=0.0)
        self.world = carla.World()

    def _make_candidates(self, specs):
        """specs: list of (rho_p0, rho_p1, eta_p2) tuples."""
        out = []
        for rho_p0, rho_p1, eta_p2 in specs:
            t = straight_trajectory()
            t.rho_p0, t.rho_p1, t.eta_p2 = rho_p0, rho_p1, eta_p2
            out.append(t)
        return out

    def _patch(self, candidates, context=Context.CITY):
        self.planner.generator.generate = lambda ego, speed_limit_ms: candidates
        self.planner.evaluator.evaluate_all = lambda *a, **k: candidates
        self.planner.detector.detect = lambda *a, **k: context

    def test_all_unsafe_triggers_emergency_brake(self):
        candidates = self._make_candidates([(-1.0, 1.0, 0.5), (-2.0, 1.0, 0.9)])
        self._patch(candidates)
        best, _ = self.planner.plan(self.ego, self.world, None, 13.9)
        self.assertEqual(best.p0_reason, "emergency brake")
        self.assertEqual(best.target_speeds, [0.0])

    def test_all_p1_illegal_falls_back_to_least_harmful(self):
        # all P0-safe, all P1-illegal — should pick the one with the highest (least negative) rho_p1
        candidates = self._make_candidates([(1.0, -0.5, 0.9), (1.0, -0.1, 0.2), (1.0, -2.0, 0.5)])
        self._patch(candidates)
        best, _ = self.planner.plan(self.ego, self.world, None, 13.9)
        self.assertAlmostEqual(best.rho_p1, -0.1)

    def test_normal_case_picks_highest_eta_among_legal(self):
        candidates = self._make_candidates([
            (1.0, 1.0, 0.3),
            (1.0, 1.0, 0.9),   # should win: safe, legal, highest eta_p2
            (1.0, -0.5, 0.99),  # would win on eta alone, but illegal — must be excluded
        ])
        self._patch(candidates)
        best, _ = self.planner.plan(self.ego, self.world, None, 13.9)
        self.assertAlmostEqual(best.eta_p2, 0.9)


# ═══════════════════════════════════════════════════════════════════════════
# 8. PATH EXECUTOR — FULL TRACKING LOOP
# ═══════════════════════════════════════════════════════════════════════════

class TestPathExecutor(unittest.TestCase):

    def test_executor_advances_through_waypoints_and_reaches_end(self):
        traj = straight_trajectory(speed_factor=0.6, n=15, step=3.0)
        ego = carla.Actor(x=0.0, y=0.0, yaw=0.0)
        executor = PathExecutor(dt=0.05)
        executor.set_trajectory(traj)

        final_wp = traj.waypoints[-1]
        for _ in range(800):
            control = executor.step(ego)
            ego.apply_control(control, dt=0.05)
            if executor.current_wp_index >= len(traj.waypoints):
                break

        dist_to_end = ego.get_location().distance(final_wp)
        self.assertLess(dist_to_end, 5.0,
                         "expected the executor to drive the ego to within a few "
                         "metres of the final waypoint")


if __name__ == '__main__':
    unittest.main(verbosity=2)
