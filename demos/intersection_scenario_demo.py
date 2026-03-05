"""
Intersection Scenario - Rule-Based Planning Demo
=================================================

This script demonstrates:
1. Four different intersection scenarios
2. Multiple maneuver types (stop, straight, left, right)
3. Rule-based trajectory selection (rulebook)
4. Lexicographic ordering for decision making
5. Visualization in CARLA

Scenarios:
    A: Green light, clear → Should go straight
    B: Yellow light → Should stop or go (depends on distance)
    C: Red light → Should turn right (only legal option)
    D: Green light, blocked → Should NOT block intersection

Author: Ahmad Ahmad
For: Nidhi (Curious Cardinals Mentorship)
Date: January 2026
"""

import carla
import numpy as np
from typing import List, Tuple, Optional, Dict
import time
import math


# ============================================================================
# QUINTIC POLYNOMIAL
# ============================================================================

class QuinticPolynomial:
    """Generates quintic polynomial trajectories."""
    
    def __init__(self, x0: float, v0: float, a0: float,
                 xf: float, vf: float, af: float, T: float):
        self.x0, self.v0, self.a0 = x0, v0, a0
        self.xf, self.vf, self.af = xf, vf, af
        self.T = T
        self.coeffs = self._compute_coefficients()
    
    def _compute_coefficients(self) -> np.ndarray:
        T = self.T
        A = np.array([
            [0,     0,     0,      0,       0,      1],
            [T**5,  T**4,  T**3,   T**2,    T,      1],
            [0,     0,     0,      0,       1,      0],
            [5*T**4, 4*T**3, 3*T**2, 2*T,   1,      0],
            [0,     0,     0,      2,       0,      0],
            [20*T**3, 12*T**2, 6*T, 2,      0,      0]
        ])
        b = np.array([self.x0, self.xf, self.v0, self.vf, self.a0, self.af])
        return np.linalg.solve(A, b)
    
    def calc_point(self, t: float) -> float:
        t = np.clip(t, 0, self.T)
        return np.polyval(self.coeffs[::-1], t)
    
    def calc_first_derivative(self, t: float) -> float:
        t = np.clip(t, 0, self.T)
        d_coeffs = np.array([5, 4, 3, 2, 1, 0]) * self.coeffs
        return np.polyval(d_coeffs[::-1], t)


# ============================================================================
# TRAJECTORY GENERATORS
# ============================================================================

def generate_stop_trajectory(s0: float, v0: float, stop_distance: float) -> dict:
    """
    Generate trajectory to stop before intersection.
    
    Args:
        s0: Initial position
        v0: Initial velocity
        stop_distance: Distance to stop line
        
    Returns:
        Trajectory dictionary
    """
    # Comfortable deceleration
    T = max(v0 / 2.0, 1.0)  # At least 1 second
    
    s_poly = QuinticPolynomial(s0, v0, 0.0, s0 + stop_distance, 0.0, 0.0, T)
    d_poly = QuinticPolynomial(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, T)
    
    waypoints = []
    dt = 0.1
    times = np.arange(0, T + dt, dt)
    
    for t in times:
        s = s_poly.calc_point(t)
        d = d_poly.calc_point(t)
        v = s_poly.calc_first_derivative(t)
        
        waypoints.append({'s': s, 'd': d, 'v': v, 't': t})
    
    return {
        'waypoints': waypoints,
        'duration': T,
        'action': 'STOP',
        'final_s': s0 + stop_distance,
        'final_v': 0.0,
        'color': (255, 0, 0)  # Red
    }


def generate_straight_trajectory(s0: float, v0: float, 
                                 intersection_dist: float, 
                                 v_target: float) -> dict:
    """
    Generate trajectory to go straight through intersection.
    
    Args:
        s0: Initial position
        v0: Initial velocity
        intersection_dist: Distance to intersection center
        v_target: Target velocity through intersection
        
    Returns:
        Trajectory dictionary
    """
    # Distance through intersection
    total_distance = intersection_dist + 15.0  # 15m past center
    
    # Time calculation
    avg_v = (v0 + v_target) / 2.0
    T = total_distance / avg_v
    
    s_poly = QuinticPolynomial(s0, v0, 0.0, s0 + total_distance, v_target, 0.0, T)
    d_poly = QuinticPolynomial(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, T)
    
    waypoints = []
    dt = 0.1
    times = np.arange(0, T + dt, dt)
    
    for t in times:
        s = s_poly.calc_point(t)
        d = d_poly.calc_point(t)
        v = s_poly.calc_first_derivative(t)
        
        waypoints.append({'s': s, 'd': d, 'v': v, 't': t})
    
    return {
        'waypoints': waypoints,
        'duration': T,
        'action': 'STRAIGHT',
        'final_s': s0 + total_distance,
        'final_v': v_target,
        'color': (0, 255, 0)  # Green
    }


def generate_right_turn_trajectory(s0: float, v0: float,
                                   intersection_dist: float) -> dict:
    """
    Generate trajectory to turn right at intersection.
    
    Args:
        s0: Initial position
        v0: Initial velocity
        intersection_dist: Distance to intersection center
        
    Returns:
        Trajectory dictionary
    """
    # Slow down for turn
    v_turn = 4.0  # Slow speed for turn
    
    # Approach + turn
    T = (intersection_dist / ((v0 + v_turn) / 2.0)) + 3.0
    
    # Move to right (negative d)
    s_poly = QuinticPolynomial(s0, v0, 0.0, 
                               s0 + intersection_dist + 10.0, 
                               v_turn, 0.0, T)
    d_poly = QuinticPolynomial(0.0, 0.0, 0.0, -3.5, 0.0, 0.0, T)
    
    waypoints = []
    dt = 0.1
    times = np.arange(0, T + dt, dt)
    
    for t in times:
        s = s_poly.calc_point(t)
        d = d_poly.calc_point(t)
        v = s_poly.calc_first_derivative(t)
        
        waypoints.append({'s': s, 'd': d, 'v': v, 't': t})
    
    return {
        'waypoints': waypoints,
        'duration': T,
        'action': 'RIGHT',
        'final_s': s0 + intersection_dist + 10.0,
        'final_v': v_turn,
        'color': (0, 0, 255)  # Blue
    }


def generate_left_turn_trajectory(s0: float, v0: float,
                                  intersection_dist: float) -> dict:
    """
    Generate trajectory to turn left at intersection.
    
    Args:
        s0: Initial position
        v0: Initial velocity
        intersection_dist: Distance to intersection center
        
    Returns:
        Trajectory dictionary
    """
    # Slow down for turn
    v_turn = 3.5  # Slower for left turn
    
    T = (intersection_dist / ((v0 + v_turn) / 2.0)) + 4.0
    
    # Move to left (positive d)
    s_poly = QuinticPolynomial(s0, v0, 0.0,
                               s0 + intersection_dist + 12.0,
                               v_turn, 0.0, T)
    d_poly = QuinticPolynomial(0.0, 0.0, 0.0, 3.5, 0.0, 0.0, T)
    
    waypoints = []
    dt = 0.1
    times = np.arange(0, T + dt, dt)
    
    for t in times:
        s = s_poly.calc_point(t)
        d = d_poly.calc_point(t)
        v = s_poly.calc_first_derivative(t)
        
        waypoints.append({'s': s, 'd': d, 'v': v, 't': t})
    
    return {
        'waypoints': waypoints,
        'duration': T,
        'action': 'LEFT',
        'final_s': s0 + intersection_dist + 12.0,
        'final_v': v_turn,
        'color': (255, 255, 0)  # Yellow
    }


# ============================================================================
# RULEBOOK EVALUATION
# ============================================================================

def evaluate_trajectory(trajectory: dict, context: dict) -> List[float]:
    """
    Evaluate trajectory using rulebook hierarchy.
    
    Priority 0: Collision
    Priority 1: Legal (traffic law compliance)
    Priority 2: Courtesy (don't block intersection)
    Priority 3: Progress (distance traveled)
    Priority 4: Efficiency (time)
    
    Args:
        trajectory: Trajectory to evaluate
        context: Dictionary with traffic_light, obstacles, etc.
        
    Returns:
        List of violation scores [r0, r1, r2, r3, r4]
    """
    
    # Priority 0: Collision (0 = safe, 1 = collision)
    r0 = check_collision(trajectory, context.get('obstacles', []))
    
    # Priority 1: Legal compliance (0 = legal, 1+ = violations)
    r1 = check_traffic_law(trajectory, context['traffic_light'], 
                          context['distance_to_intersection'])
    
    # Priority 2: Courtesy - blocking (0 = OK, 1 = blocking)
    r2 = check_blocking(trajectory, context)
    
    # Priority 3: Progress (negative distance, want to maximize)
    r3 = -trajectory['final_s']
    
    # Priority 4: Efficiency (time)
    r4 = trajectory['duration']
    
    return [r0, r1, r2, r3, r4]


def check_collision(trajectory: dict, obstacles: List) -> float:
    """Check if trajectory collides with obstacles."""
    # Simplified: check if we hit blocking vehicles
    for waypoint in trajectory['waypoints']:
        for obs in obstacles:
            # Simple distance check
            dist = math.sqrt((waypoint['s'] - obs['s'])**2 + 
                           (waypoint['d'] - obs['d'])**2)
            if dist < 3.0:  # Safety margin
                return 1.0  # Collision!
    return 0.0  # Safe


def check_traffic_law(trajectory: dict, traffic_light: str, 
                     distance_to_intersection: float) -> float:
    """
    Check if trajectory violates traffic laws.
    
    Returns:
        0.0 if legal, 1.0+ for violations
    """
    violations = 0.0
    action = trajectory['action']
    
    if traffic_light == 'red':
        # Can't go straight or left on red
        if action in ['STRAIGHT', 'LEFT']:
            violations += 1.0
        # Right on red is usually OK (0 violations)
    
    elif traffic_light == 'yellow':
        # Yellow light logic: can proceed if can't safely stop
        stopping_distance = trajectory['waypoints'][0]['v']**2 / (2 * 2.0)  # ~2 m/s² decel
        
        if distance_to_intersection > stopping_distance:
            # Can safely stop, but proceeding is also legal
            violations += 0.0  # Either choice is legal
        else:
            # Can't safely stop, must proceed
            violations += 0.0
    
    # Green light: all actions legal
    # (violations stays 0.0)
    
    return violations


def check_blocking(trajectory: dict, context: dict) -> float:
    """
    Check if trajectory blocks intersection.
    
    Returns:
        0.0 if not blocking, 1.0 if blocking
    """
    intersection_range = (context['distance_to_intersection'] - 5.0,
                         context['distance_to_intersection'] + 5.0)
    
    final_waypoint = trajectory['waypoints'][-1]
    
    # Check if we end up stopped inside intersection
    if intersection_range[0] <= final_waypoint['s'] <= intersection_range[1]:
        if final_waypoint['v'] < 0.5:  # Nearly stopped
            # Check if there are obstacles ahead (traffic jam)
            if context.get('blocked_ahead', False):
                return 1.0  # Blocking!
    
    return 0.0  # Not blocking


def select_best_trajectory(trajectories: List[dict], context: dict) -> dict:
    """
    Select best trajectory using lexicographic ordering.
    
    Args:
        trajectories: List of candidate trajectories
        context: Scene context (traffic light, obstacles, etc.)
        
    Returns:
        Best trajectory
    """
    # Evaluate all trajectories
    evaluated = []
    for traj in trajectories:
        violations = evaluate_trajectory(traj, context)
        evaluated.append((traj, violations))
    
    # Sort by lexicographic order (Python compares lists element-by-element)
    evaluated.sort(key=lambda x: x[1])
    
    # Best is first after sorting
    best_traj, best_violations = evaluated[0]
    
    # Print decision
    print("\n" + "="*70)
    print("RULEBOOK DECISION")
    print("="*70)
    print(f"Traffic Light: {context['traffic_light'].upper()}")
    print(f"Distance to Intersection: {context['distance_to_intersection']:.1f}m")
    print(f"Blocked Ahead: {context.get('blocked_ahead', False)}")
    print()
    print(f"Selected Action: {best_traj['action']}")
    print(f"Duration: {best_traj['duration']:.1f}s")
    print()
    print("Rulebook Evaluation:")
    print(f"  P0 (Collision):  {best_violations[0]:.1f} {'✓ Safe' if best_violations[0] == 0 else '✗ UNSAFE'}")
    print(f"  P1 (Legal):      {best_violations[1]:.1f} {'✓ Legal' if best_violations[1] == 0 else '✗ ILLEGAL'}")
    print(f"  P2 (Blocking):   {best_violations[2]:.1f} {'✓ Clear' if best_violations[2] == 0 else '✗ BLOCKING'}")
    print(f"  P3 (Progress):   {-best_violations[3]:.1f}m")
    print(f"  P4 (Efficiency): {best_violations[4]:.1f}s")
    
    # Show why others were rejected
    print("\nOther Options:")
    for traj, violations in evaluated[1:4]:  # Show top 3 alternatives
        print(f"  {traj['action']:8s}: P0={violations[0]:.0f} P1={violations[1]:.0f} "
              f"P2={violations[2]:.0f} P3={-violations[3]:.0f}m P4={violations[4]:.0f}s "
              f"{'❌' if violations > best_violations else ''}")
    
    print("="*70)
    
    return best_traj


# ============================================================================
# VISUALIZATION
# ============================================================================

def visualize_trajectories(world: carla.World, trajectories: List[dict],
                          selected: dict, reference_transform: carla.Transform):
    """
    Visualize all trajectories and highlight selected one.
    
    Args:
        world: CARLA world
        trajectories: All candidate trajectories
        selected: Selected trajectory
        reference_transform: Vehicle's reference transform
    """
    debug = world.debug
    
    # Draw all candidates as small dots
    for traj in trajectories:
        color = carla.Color(*traj['color'], 150)
        
        for i in range(0, len(traj['waypoints']), 3):
            wp = traj['waypoints'][i]
            
            # Convert Frenet to global (simplified: assume straight road)
            x = reference_transform.location.x + wp['s']
            y = reference_transform.location.y + wp['d']
            
            debug.draw_point(
                carla.Location(x=x, y=y, z=2.0),
                size=0.08,
                color=color,
                life_time=30.0
            )
    
    # Draw selected trajectory with larger dots
    color_selected = carla.Color(*selected['color'], 255)
    for i in range(0, len(selected['waypoints']), 2):
        wp = selected['waypoints'][i]
        
        x = reference_transform.location.x + wp['s']
        y = reference_transform.location.y + wp['d']
        
        debug.draw_point(
            carla.Location(x=x, y=y, z=2.5),
            size=0.15,
            color=color_selected,
            life_time=30.0
        )
    
    # Draw arrow at selected trajectory start
    start_wp = selected['waypoints'][0]
    x_start = reference_transform.location.x + start_wp['s']
    y_start = reference_transform.location.y + start_wp['d']
    
    # debug.draw_arrow(
    #     carla.Location(x=x_start, y=y_start, z=3.0),
    #     carla.Location(x=x_start, y=y_start, z=5.0),
    #     thickness=0.3,
    #     arrow_size=0.5,
    #     color=carla.Color(255, 255, 255, 255),
    #     life_time=30.0
    # )


def create_traffic_light_marker(world: carla.World, location: carla.Location,
                                state: str):
    """
    Draw a visual marker for traffic light state.
    
    Args:
        world: CARLA world
        location: Where to draw
        state: 'green', 'yellow', or 'red'
    """
    debug = world.debug
    
    colors = {
        'green': carla.Color(0, 255, 0, 255),
        'yellow': carla.Color(255, 255, 0, 255),
        'red': carla.Color(255, 0, 0, 255)
    }
    
    color = colors.get(state, carla.Color(255, 255, 255, 255))
    
    # Draw large sphere for traffic light
    debug.draw_point(
        location,
        size=0.5,
        color=color,
        life_time=30.0
    )
    
    # Draw post
    debug.draw_line(
        carla.Location(location.x, location.y, location.z - 5.0),
        location,
        thickness=0.1,
        color=carla.Color(100, 100, 100, 255),
        life_time=30.0
    )


# ============================================================================
# SCENARIO SETUP
# ============================================================================

def get_scenario_context(scenario: str, vehicle_location: carla.Location) -> dict:
    """
    Get context for specific scenario.
    
    Args:
        scenario: 'A', 'B', 'C', or 'D'
        vehicle_location: Current vehicle location
        
    Returns:
        Context dictionary
    """
    # Distance to intersection (ahead of vehicle)
    distance_to_intersection = 25.0
    
    contexts = {
        'A': {  # Green light, clear
            'traffic_light': 'green',
            'distance_to_intersection': distance_to_intersection,
            'obstacles': [],
            'blocked_ahead': False,
            'description': 'Green light, clear intersection'
        },
        'B': {  # Yellow light
            'traffic_light': 'yellow',
            'distance_to_intersection': distance_to_intersection,
            'obstacles': [],
            'blocked_ahead': False,
            'description': 'Yellow light, must decide'
        },
        'C': {  # Red light
            'traffic_light': 'red',
            'distance_to_intersection': distance_to_intersection,
            'obstacles': [],
            'blocked_ahead': False,
            'description': 'Red light, limited options'
        },
        'D': {  # Green but blocked
            'traffic_light': 'green',
            'distance_to_intersection': distance_to_intersection,
            'obstacles': [
                {'s': distance_to_intersection + 2, 'd': 0.0},  # Car in intersection
                {'s': distance_to_intersection + 7, 'd': 0.0}   # Another car
            ],
            'blocked_ahead': True,
            'description': 'Green light, but intersection blocked!'
        }
    }
    
    return contexts.get(scenario, contexts['A'])


# ============================================================================
# MAIN DEMO
# ============================================================================

def main():
    """Main intersection scenario demonstration."""
    
    print("\n" + "="*70)
    print("INTERSECTION SCENARIO - RULE-BASED PLANNING")
    print("="*70)
    
    # Connect to CARLA
    print("\n[STEP 1] Connecting to CARLA...")
    try:
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)
        world = client.get_world()
        print(f"✓ Connected (Map: {world.get_map().name})")
    except Exception as e:
        print(f"✗ Failed: {e}")
        return
    
    # Spawn vehicle
    print("\n[STEP 2] Spawning vehicle...")
    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.filter('vehicle.tesla.model3')[0]
    spawn_points = world.get_map().get_spawn_points()
    
    vehicle = world.spawn_actor(vehicle_bp, spawn_points[0])
    print("✓ Vehicle spawned")
    time.sleep(1.0)
    
    # Get vehicle location
    vehicle_transform = vehicle.get_transform()
    vehicle_location = vehicle_transform.location
    
    # Setup camera
    print("\n[STEP 3] Setting up chase camera...")
    spectator = world.get_spectator()
    
    def update_camera():
        vt = vehicle.get_transform()
        vl = vt.location
        vyaw = math.radians(vt.rotation.yaw)
        
        cam_x = vl.x - 15 * math.cos(vyaw)
        cam_y = vl.y - 15 * math.sin(vyaw)
        cam_z = vl.z + 8
        
        spectator.set_transform(carla.Transform(
            carla.Location(cam_x, cam_y, cam_z),
            carla.Rotation(pitch=-20, yaw=vt.rotation.yaw, roll=0)
        ))
    
    update_camera()
    print("✓ Camera positioned")
    
    # Select scenario
    print("\n[STEP 4] Select intersection scenario:")
    print("  A: Green light, clear")
    print("  B: Yellow light")
    print("  C: Red light")
    print("  D: Green light, but intersection BLOCKED")
    
    choice = input("\nEnter scenario (A-D) [default=D]: ").strip().upper()
    if choice not in ['A', 'B', 'C', 'D']:
        choice = 'D'
    
    # Get scenario context
    context = get_scenario_context(choice, vehicle_location)
    
    print(f"\n✓ Scenario {choice}: {context['description']}")
    
    # Draw traffic light
    print("\n[STEP 5] Creating traffic light...")
    traffic_light_location = carla.Location(
        x=vehicle_location.x + context['distance_to_intersection'],
        y=vehicle_location.y,
        z=vehicle_location.z + 5.0
    )
    create_traffic_light_marker(world, traffic_light_location, context['traffic_light'])
    print(f"✓ Traffic light: {context['traffic_light'].upper()}")
    
    # Draw obstacles if present
    if context['obstacles']:
        print(f"✓ {len(context['obstacles'])} obstacles blocking intersection")
        for obs in context['obstacles']:
            obs_loc = carla.Location(
                x=vehicle_location.x + obs['s'],
                y=vehicle_location.y + obs['d'],
                z=vehicle_location.z + 1.0
            )
            world.debug.draw_point(obs_loc, size=0.3,
                                  color=carla.Color(255, 0, 0, 255),
                                  life_time=30.0)
    
    # Generate candidate trajectories
    print("\n[STEP 6] Generating candidate trajectories...")
    
    v0 = 6.0  # Start at safe speed (6 m/s = 22 km/h)
    s0 = 0.0
    
    trajectories = []
    
    # Always generate STOP option
    stop_traj = generate_stop_trajectory(s0, v0, context['distance_to_intersection'] - 2.0)
    trajectories.append(stop_traj)
    print(f"  Generated: STOP")
    
    # Generate STRAIGHT options
    for v_mult in [0.9, 1.0]:
        straight_traj = generate_straight_trajectory(
            s0, v0, context['distance_to_intersection'], v0 * v_mult
        )
        trajectories.append(straight_traj)
        print(f"  Generated: STRAIGHT (v={v0*v_mult:.1f} m/s)")
    
    # Generate RIGHT TURN
    right_traj = generate_right_turn_trajectory(
        s0, v0, context['distance_to_intersection']
    )
    trajectories.append(right_traj)
    print(f"  Generated: RIGHT")
    
    # Generate LEFT TURN (only if might be useful)
    if context['traffic_light'] == 'green':
        left_traj = generate_left_turn_trajectory(
            s0, v0, context['distance_to_intersection']
        )
        trajectories.append(left_traj)
        print(f"  Generated: LEFT")
    
    print(f"✓ Generated {len(trajectories)} candidate trajectories")
    
    # Evaluate with rulebook and select best
    print("\n[STEP 7] Evaluating trajectories with rulebook...")
    selected = select_best_trajectory(trajectories, context)
    
    # Visualize
    print("\n[STEP 8] Visualizing trajectories...")
    visualize_trajectories(world, trajectories, selected, vehicle_transform)
    print("✓ Trajectories drawn")
    print("\nLook at CARLA:")
    print(f"  🔴 Red dots = STOP")
    print(f"  🟢 Green dots = STRAIGHT")
    print(f"  🔵 Blue dots = RIGHT")
    if context['traffic_light'] == 'green':
        print(f"  🟡 Yellow dots = LEFT")
    print(f"  ⚪ White arrow = SELECTED action")
    
    # Summary
    print("\n" + "="*70)
    print("SCENARIO SUMMARY")
    print("="*70)
    print(f"Scenario: {choice} - {context['description']}")
    print(f"Traffic Light: {context['traffic_light'].upper()}")
    print(f"Selected Action: {selected['action']}")
    print("\nWhy this decision?")
    
    if choice == 'A':
        print("  Green light + clear → GO STRAIGHT is fastest and legal ✓")
    elif choice == 'B':
        print("  Yellow light → Could stop OR proceed")
        print("  Decision depends on distance and speed")
    elif choice == 'C':
        print("  Red light → Can't go straight or left")
        print("  RIGHT turn is only legal moving option ✓")
    elif choice == 'D':
        print("  Green light BUT intersection blocked")
        print("  Don't block the box (P2: Courtesy rule)")
        print("  RIGHT turn or STOP are better options ✓")
    
    print("\nVisualization will remain for 30 seconds...")
    print("Press Ctrl+C to exit")
    print("="*70)
    
    # Cleanup
    try:
        time.sleep(30)
    except KeyboardInterrupt:
        pass
    
    vehicle.destroy()
    print("\n✓ Demo complete!\n")


if __name__ == "__main__":
    main()
