"""
Sampling-Based Planner with Path Following - CARLA Demo
========================================================

This script demonstrates:
1. Generate MULTIPLE candidate trajectories (sampling-based planner)
2. Visualize ALL candidates in CARLA with different colors
3. Select best trajectory
4. Follow it with Pure Pursuit controller
5. Camera mounted directly on top, following the vehicle

Author: Ahmad Ahmad
For: Nidhi (Curious Cardinals Mentorship)
Date: January 2026
"""

from http import client
from random import random

import carla
import numpy as np
from typing import List, Tuple, Optional
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
# REFERENCE PATH
# ============================================================================

class ReferencePath:
    """Reference path for Frenet frame transformations."""
    
    def __init__(self, start_location: carla.Location, 
                 length: float = 200.0, heading: float = 0.0):
        self.start_x = start_location.x
        self.start_y = start_location.y
        self.length = length
        self.heading_rad = np.deg2rad(heading)
        
        num_points = int(length / 0.5)
        self.s_samples = np.linspace(0, length, num_points)
        self.x_samples = self.start_x + self.s_samples * np.cos(self.heading_rad)
        self.y_samples = self.start_y + self.s_samples * np.sin(self.heading_rad)
        self.yaw_samples = np.full(num_points, self.heading_rad)
    
    def frenet_to_global(self, s: float, d: float) -> Tuple[float, float, float]:
        idx = np.argmin(np.abs(self.s_samples - s))
        x_ref = self.x_samples[idx]
        y_ref = self.y_samples[idx]
        yaw_ref = self.yaw_samples[idx]
        
        x = x_ref - d * np.sin(yaw_ref)
        y = y_ref + d * np.cos(yaw_ref)
        
        return x, y, yaw_ref


# ============================================================================
# SAMPLING-BASED TRAJECTORY GENERATOR
# ============================================================================

def generate_candidate_trajectories(s0: float, d0: float, s_dot0: float,
                                    reference_path: ReferencePath) -> List[dict]:
    """
    Generate multiple candidate trajectories using sampling-based approach.
    
    Samples:
    - Lateral positions: [-3.5, 0, 3.5] (left lane, center, right lane)
    - Target velocities: [8, 12, 15, 18] m/s
    - Time horizons: [4, 5, 6] seconds
    
    Args:
        s0, d0: Initial position in Frenet frame
        s_dot0: Initial velocity
        reference_path: Reference path object
        
    Returns:
        List of trajectory dictionaries
    """
    trajectories = []
    
    # Sampling parameters
    d_samples = [-3.5, 0.0, 3.5]  # Left lane, center, right lane
    v_samples = [8.0, 12.0, 15.0, 18.0]  # Target velocities
    T_samples = [4.0, 5.0, 6.0]  # Time horizons
    
    # Color mapping for visualization
    d_colors = {
        -3.5: (255, 0, 0),     # Red for left lane
        0.0: (0, 0, 255),      # Blue for center
        3.5: (0, 255, 0)       # Green for right lane
    }
    
    print("\n" + "="*70)
    print("GENERATING CANDIDATE TRAJECTORIES")
    print("="*70)
    print(f"Initial state: s={s0:.1f}m, d={d0:.1f}m, v={s_dot0:.1f}m/s")
    print(f"\nSampling:")
    print(f"  Lateral: {d_samples} m")
    print(f"  Velocity: {v_samples} m/s")
    print(f"  Duration: {T_samples} s")
    print(f"  Total combinations: {len(d_samples) * len(v_samples) * len(T_samples)}")
    print()
    
    traj_id = 0
    for d_target in d_samples:
        for v_target in v_samples:
            for T in T_samples:
                # Generate trajectory
                traj = generate_single_trajectory(
                    s0, d0, s_dot0,
                    d_target, v_target, T,
                    reference_path
                )
                
                # Add metadata
                traj['id'] = traj_id
                traj['d_target'] = d_target
                traj['v_target'] = v_target
                traj['color'] = d_colors[d_target]
                
                # Classify maneuver
                if d_target < d0:
                    traj['maneuver'] = 'LANE_CHANGE_LEFT'
                elif d_target > d0:
                    traj['maneuver'] = 'LANE_CHANGE_RIGHT'
                else:
                    traj['maneuver'] = 'STAY_IN_LANE'
                
                trajectories.append(traj)
                traj_id += 1
    
    print(f"✓ Generated {len(trajectories)} candidate trajectories")
    print("="*70)
    
    return trajectories


def generate_single_trajectory(s0: float, d0: float, s_dot0: float,
                               d_final: float, v_final: float, T: float,
                               reference_path: ReferencePath) -> dict:
    """Generate a single trajectory."""
    # Compute final s position
    s_final = s0 + 0.5 * (s_dot0 + v_final) * T
    
    # Create quintic polynomials
    s_poly = QuinticPolynomial(s0, s_dot0, 0.0, s_final, v_final, 0.0, T)
    d_poly = QuinticPolynomial(d0, 0.0, 0.0, d_final, 0.0, 0.0, T)
    
    # Sample trajectory
    dt = 0.1
    times = np.arange(0, T + dt, dt)
    
    waypoints = []
    for t in times:
        s = s_poly.calc_point(t)
        d = d_poly.calc_point(t)
        s_dot = s_poly.calc_first_derivative(t)
        d_dot = d_poly.calc_first_derivative(t)
        
        # Convert to global coordinates
        x, y, yaw = reference_path.frenet_to_global(s, d)
        
        # Compute velocity
        velocity = math.sqrt(s_dot**2 + d_dot**2)
        
        waypoints.append({
            'x': x,
            'y': y,
            'yaw': yaw,
            'velocity': velocity,
            's': s,
            'd': d,
            'time': t
        })
    
    return {
        'waypoints': waypoints,
        'duration': T,
        'd_final': d_final,
        'v_final': v_final
    }


# ============================================================================
# PURE PURSUIT CONTROLLER
# ============================================================================

class PurePursuitController:
    """Pure Pursuit path tracking controller."""
    
    def __init__(self, wheelbase: float = 2.7, lookahead_distance: float = 8.0):
        self.wheelbase = wheelbase
        self.lookahead_distance = lookahead_distance
        
        # PID gains for throttle control
        self.Kp_throttle = 0.5
        self.Ki_throttle = 0.01
        self.Kd_throttle = 0.1
        
        # State for PID
        self.velocity_error_integral = 0.0
        self.prev_velocity_error = 0.0
    
    def find_lookahead_point(self, vehicle_x: float, vehicle_y: float,
                            waypoints: List[dict]) -> Optional[dict]:
        """Find lookahead point on path."""
        min_dist = float('inf')
        closest_idx = 0
        
        for i, wp in enumerate(waypoints):
            dist = math.sqrt((wp['x'] - vehicle_x)**2 + (wp['y'] - vehicle_y)**2)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        for i in range(closest_idx, len(waypoints)):
            wp = waypoints[i]
            dist = math.sqrt((wp['x'] - vehicle_x)**2 + (wp['y'] - vehicle_y)**2)
            
            if dist >= self.lookahead_distance:
                return wp
        
        if len(waypoints) > 0:
            return waypoints[-1]
        
        return None
    
    def compute_steering_angle(self, vehicle_x: float, vehicle_y: float,
                               vehicle_yaw: float, lookahead_point: dict) -> float:
        """Compute steering angle using Pure Pursuit."""
        dx = lookahead_point['x'] - vehicle_x
        dy = lookahead_point['y'] - vehicle_y
        
        angle_to_point = math.atan2(dy, dx)
        alpha = self._normalize_angle(angle_to_point - vehicle_yaw)
        
        steering = math.atan(2.0 * self.wheelbase * math.sin(alpha) / self.lookahead_distance)
        
        max_steer = math.radians(30)
        steering = np.clip(steering, -max_steer, max_steer)
        
        return steering
    
    def compute_throttle(self, current_velocity: float, target_velocity: float,
                        dt: float = 0.05) -> Tuple[float, float]:
        """Compute throttle and brake using PID."""
        error = target_velocity - current_velocity
        
        P = self.Kp_throttle * error
        self.velocity_error_integral += error * dt
        I = self.Ki_throttle * self.velocity_error_integral
        D = self.Kd_throttle * (error - self.prev_velocity_error) / dt
        
        control = P + I + D
        
        if control > 0:
            throttle = np.clip(control, 0.0, 1.0)
            brake = 0.0
        else:
            throttle = 0.0
            brake = np.clip(-control, 0.0, 1.0)
        
        self.prev_velocity_error = error
        
        return throttle, brake
    
    def compute_control(self, vehicle_transform: carla.Transform,
                       vehicle_velocity: carla.Vector3D,
                       waypoints: List[dict]) -> Optional[carla.VehicleControl]:
        """Compute vehicle control."""
        vehicle_x = vehicle_transform.location.x
        vehicle_y = vehicle_transform.location.y
        vehicle_yaw = math.radians(vehicle_transform.rotation.yaw)
        current_speed = math.sqrt(vehicle_velocity.x**2 + vehicle_velocity.y**2)
        
        lookahead = self.find_lookahead_point(vehicle_x, vehicle_y, waypoints)
        
        if lookahead is None:
            return None
        
        steering = self.compute_steering_angle(vehicle_x, vehicle_y, vehicle_yaw, lookahead)
        target_velocity = lookahead['velocity']
        throttle, brake = self.compute_throttle(current_speed, target_velocity)
        
        control = carla.VehicleControl()
        control.steer = float(steering / math.radians(30))
        control.throttle = float(throttle)
        control.brake = float(brake)
        control.hand_brake = False
        control.manual_gear_shift = False
        
        return control
    
    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


# ============================================================================
# TOP-DOWN CAMERA
# ============================================================================

def update_top_down_camera(spectator: carla.Actor, vehicle: carla.Actor,
                          height: float = 50.0):
    """
    Position camera directly above vehicle (bird's eye view).
    
    Args:
        spectator: CARLA spectator (camera)
        vehicle: Vehicle to follow
        height: Height above vehicle (meters)
    """
    vehicle_transform = vehicle.get_transform()
    vehicle_loc = vehicle_transform.location
    
    # Camera directly above vehicle
    camera_location = carla.Location(
        x=vehicle_loc.x,
        y=vehicle_loc.y,
        z=vehicle_loc.z + height
    )
    
    # Look straight down, rotate with vehicle heading
    camera_rotation = carla.Rotation(
        pitch=-90,  # Look straight down
        yaw=vehicle_transform.rotation.yaw,  # Rotate with vehicle
        roll=0
    )
    
    spectator.set_transform(carla.Transform(camera_location, camera_rotation))


# ============================================================================
# VISUALIZATION
# ============================================================================

def visualize_all_trajectories(world: carla.World, trajectories: List[dict]):
    """
    Draw all candidate trajectories in CARLA.
    
    Args:
        world: CARLA world
        trajectories: List of trajectory dictionaries
    """
    debug = world.debug
    
    print("\n[Visualizing trajectories]")
    print("-" * 70)
    
    for traj in trajectories:
        waypoints = traj['waypoints']
        color = carla.Color(*traj['color'], 150)  # Semi-transparent
        
        # Draw trajectory
        for i in range(len(waypoints) - 1):
            wp1 = waypoints[i]
            wp2 = waypoints[i + 1]
            
            start = carla.Location(x=wp1['x'], y=wp1['y'], z=2.0)
            end = carla.Location(x=wp2['x'], y=wp2['y'], z=2.0)
            
            debug.draw_line(start, end, thickness=0.25,
                           color=color,
                           life_time=60.0)
        
        print(f"  Trajectory {traj['id']:2d}: {traj['maneuver']:20s} "
              f"(d={traj['d_target']:+5.1f}m, v={traj['v_target']:4.1f}m/s, "
              f"T={traj['duration']:.1f}s) - Color: RGB{traj['color']}")
    
    print("-" * 70)
    print(f"✓ Drew {len(trajectories)} trajectories in CARLA")


def visualize_selected_trajectory(world: carla.World, trajectory: dict):
    """
    Highlight the selected trajectory.
    
    Args:
        world: CARLA world
        trajectory: Selected trajectory dictionary
    """
    debug = world.debug
    waypoints = trajectory['waypoints']
    
    # Draw with bright yellow color and thicker line
    for i in range(len(waypoints) - 1):
        wp1 = waypoints[i]
        wp2 = waypoints[i + 1]
        
        start = carla.Location(x=wp1['x'], y=wp1['y'], z=2.5)
        end = carla.Location(x=wp2['x'], y=wp2['y'], z=2.5)
        
        debug.draw_line(start, end, thickness=0.5,
                       color=carla.Color(255, 255, 0, 255),  # Bright yellow
                       life_time=60.0)
    
    # Draw arrow at start
    start_wp = waypoints[0]
    debug.draw_arrow(
        carla.Location(x=start_wp['x'], y=start_wp['y'], z=3.0),
        carla.Location(x=start_wp['x'], y=start_wp['y'], z=5.0),
        thickness=0.3,
        arrow_size=0.5,
        color=carla.Color(255, 255, 0, 255),
        life_time=60.0
    )


def visualize_actual_path(world: carla.World, vehicle_positions: List[Tuple[float, float]]):
    """
    Draw the actual path driven by vehicle.
    
    Args:
        world: CARLA world
        vehicle_positions: List of (x, y) positions
    """
    debug = world.debug
    
    for i in range(len(vehicle_positions) - 1):
        p1 = vehicle_positions[i]
        p2 = vehicle_positions[i + 1]
        
        start = carla.Location(x=p1[0], y=p1[1], z=3.0)
        end = carla.Location(x=p2[0], y=p2[1], z=3.0)
        
        debug.draw_line(start, end, thickness=0.3,
                       color=carla.Color(255, 0, 255, 255),  # Magenta
                       life_time=60.0)


# ============================================================================
# MAIN DEMO
# ============================================================================

def main():
    """Main demonstration."""
    
    print("\n" + "="*70)
    print("SAMPLING-BASED PLANNER WITH TOP-DOWN CAMERA - CARLA DEMO")
    print("="*70)
    
    # Connect to CARLA
    print("\n[STEP 1] Connecting to CARLA...")
    try:
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)
        world = client.get_world()
        print(f"✓ Connected to CARLA (Map: {world.get_map().name})")
    except Exception as e:
        print(f"✗ Failed to connect: {e}")
        return
    
    # Spawn vehicle
    print("\n[STEP 2] Spawning vehicle...")
    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.filter('vehicle.tesla.model3')[0]
    spawn_points = world.get_map().get_spawn_points()
    
    vehicle = world.spawn_actor(vehicle_bp, spawn_points[0])
    print(f"✓ Vehicle spawned")
    time.sleep(1.0)
    # SPAWNING TRAFFIC HERE
    for sp in world.get_map().get_spawn_points()[1:30]: # Loop thru spawn points 1-30
        vehicle_bps = blueprint_library.filter('vehicle.*') # Get all vehicle blueprints, should spawn random types 
        if vehicle_bps: # If we have any blueprints, try to spawn a random one at this spawn point
            v = world.try_spawn_actor(vehicle_bps[0], sp) 
            if v: v.set_autopilot(True, client.get_trafficmanager().get_port()) 
            # If we successfully spawned a vehicle, set it to autopilot so it starts driving around
    # TRAFFIC SPAWN END 
    
    # Pedestrian spawning - Start
    walker_bps = blueprint_library.filter('walker.pedestrian.*')
    vehicle_location = vehicle.get_location()

    for i in range(50):
        # Spawn pedestrians at random locations
        spawn_offsets = [
        carla.Location(x=10, y=0, z=0),
        carla.Location(x=-10, y=0, z=0),
        carla.Location(x=0, y=10, z=0),
        carla.Location(x=0, y=-10, z=0),
        carla.Location(x=15, y=5, z=0),
        carla.Location(x=-15, y=-5, z=0),
        ]

        spawn_location = vehicle_location + spawn_offsets [ i % len(spawn_offsets) ] # Cycle through offsets for variety

        world.try_spawn_actor(walker_bps[i % len(walker_bps)], carla.Transform(spawn_location))
        loc = world.get_random_location_from_navigation()
        print(loc)
    # Pedestrian spawning - end

    # Create reference path
    print("\n[STEP 3] Creating reference path...")
    vehicle_transform = vehicle.get_transform()
    reference_path = ReferencePath(
        start_location=vehicle_transform.location,
        length=150.0,
        heading=vehicle_transform.rotation.yaw
    )
    print("✓ Reference path created")
    
    # Generate candidate trajectories
    print("\n[STEP 4] Generating candidate trajectories...")
    trajectories = generate_candidate_trajectories(
        s0=0.0,
        d0=0.0,
        s_dot0=8.0,  # Start at 8 m/s
        reference_path=reference_path
    )
    
    # Visualize all trajectories
    print("\n[STEP 5] Visualizing all candidate trajectories...")
    visualize_all_trajectories(world, trajectories)
    
    # Position camera (top-down)
    print("\n[STEP 6] Setting up top-down camera...")
    spectator = world.get_spectator()
    update_top_down_camera(spectator, vehicle, height=30.0)
    # update_chase_camera(spectator, vehicle, height=60.0) # Tried, function not defined
    # update_top_down_camera(spectator, vehicle, height=20.0) #Tried, More zoomed out version of camera
    print("✓ Camera positioned directly above vehicle")
    print("  Camera will follow vehicle from bird's eye view!")
    
    # Let user see all trajectories
    print("\n[STEP 7] Displaying all trajectories...")
    print("="*70)
    print("LOOK AT CARLA WINDOW!")
    print("  You should see ALL candidate trajectories:")
    print("    🔴 Red lines = Lane change LEFT trajectories")
    print("    🔵 Blue lines = Stay in lane trajectories")
    print("    🟢 Green lines = Lane change RIGHT trajectories")
    print("="*70)
    print("\nPress Enter to continue and select a trajectory...")
    input()
    
    # Select trajectory
    print("\n[STEP 8] Select trajectory to follow:")
    print("-" * 70)
    
    # Group by maneuver
    left_trajs = [t for t in trajectories if t['maneuver'] == 'LANE_CHANGE_LEFT']
    center_trajs = [t for t in trajectories if t['maneuver'] == 'STAY_IN_LANE']
    right_trajs = [t for t in trajectories if t['maneuver'] == 'LANE_CHANGE_RIGHT']
    
    print(f"\n1. LANE CHANGE LEFT ({len(left_trajs)} options)")
    print(f"2. STAY IN LANE ({len(center_trajs)} options)")
    print(f"3. LANE CHANGE RIGHT ({len(right_trajs)} options)")
    
    choice = input("\nEnter choice (1-3) [default=3]: ").strip()
    
    if choice == '1':
        # Select middle option from left trajectories
        selected = left_trajs[len(left_trajs) // 2]
        print(f"\n✓ Selected: LANE CHANGE LEFT")
    elif choice == '2':
        # Select middle option from center trajectories
        selected = center_trajs[len(center_trajs) // 2]
        print(f"\n✓ Selected: STAY IN LANE")
    else:
        # Select middle option from right trajectories
        selected = right_trajs[len(right_trajs) // 2]
        print(f"\n✓ Selected: LANE CHANGE RIGHT")
    
    print(f"   Trajectory ID: {selected['id']}")
    print(f"   Target lateral: {selected['d_target']:+.1f}m")
    print(f"   Target velocity: {selected['v_target']:.1f}m/s")
    print(f"   Duration: {selected['duration']:.1f}s")
    
    # Highlight selected trajectory
    print("\n[STEP 9] Highlighting selected trajectory...")
    visualize_selected_trajectory(world, selected)
    print("✓ Selected trajectory shown in YELLOW")
    
    # Create controller
    print("\n[STEP 10] Creating controller...")
    controller = PurePursuitController(
        wheelbase=2.7,
        lookahead_distance=8.0
    )
    print("✓ Controller ready")
    
    # Execute trajectory
    print("\n[STEP 11] Following selected trajectory...")
    print("="*70)
    print("WATCH THE CARLA WINDOW!")
    print("  🟡 Yellow line = Selected trajectory (planned)")
    print("  🟣 Magenta line = Actual vehicle path (will appear as you drive)")
    print("  📹 Camera = Top-down view, following vehicle")
    print("="*70)
    
    waypoints = selected['waypoints']
    vehicle_positions = []
    start_time = time.time()
    
    try:
        while True:
            # Get vehicle state
            vehicle_transform = vehicle.get_transform()
            vehicle_velocity = vehicle.get_velocity()
            
            # Record position
            vehicle_positions.append((
                vehicle_transform.location.x,
                vehicle_transform.location.y
            ))
            
            # Update camera to follow vehicle (top-down)
            update_top_down_camera(spectator, vehicle, height=30.0)
            
            # Compute control
            control = controller.compute_control(
                vehicle_transform,
                vehicle_velocity,
                waypoints
            )
            
            if control is None:
                print("\n✓ Trajectory completed!")
                break
            
            # Apply control
            vehicle.apply_control(control)
            
            # Print status
            elapsed = time.time() - start_time
            speed = math.sqrt(vehicle_velocity.x**2 + vehicle_velocity.y**2)
            
            print(f"\rTime: {elapsed:5.2f}s | "
                  f"Speed: {speed:5.2f} m/s | "
                  f"Steering: {control.steer:+6.3f} | "
                  f"Throttle: {control.throttle:5.3f} | "
                  f"Brake: {control.brake:5.3f}",
                  end='', flush=True)
            
            # Sleep to maintain ~20 Hz
            time.sleep(0.05)
            
            # Safety timeout
            if elapsed > selected['duration'] + 5.0:
                print("\n\nTimeout reached")
                break
    
    except KeyboardInterrupt:
        print("\n\nStopped by user")
    
    # Stop vehicle
    vehicle.apply_control(carla.VehicleControl(brake=1.0))
    time.sleep(0.5)
    
    # Visualize actual path
    print("\n\n[STEP 12] Drawing actual path...")
    visualize_actual_path(world, vehicle_positions)
    print("✓ Actual path drawn in MAGENTA")
    
    # Summary
    print("\n" + "="*70)
    print("SUMMARY")
    print("="*70)
    print(f"\n✓ Generated {len(trajectories)} candidate trajectories")
    print(f"✓ Visualized all candidates in CARLA")
    print(f"✓ Selected and followed trajectory {selected['id']}")
    print(f"✓ Path tracking completed successfully")
    
    print("\nWhat you can see in CARLA:")
    print("  🔴 Red = Lane change LEFT options")
    print("  🔵 Blue = Stay in lane options")
    print("  🟢 Green = Lane change RIGHT options")
    print("  🟡 Yellow (thick) = Selected trajectory")
    print("  🟣 Magenta = Actual path driven")
    
    print("\nVisualization will remain for 30 seconds...")
    print("Press Ctrl+C to exit and destroy vehicle")
    
    # Cleanup
    try:
        time.sleep(30)
    except KeyboardInterrupt:
        pass
    
    vehicle.destroy()
    print("\n✓ Vehicle destroyed")
    
    print("\n" + "="*70)
    print("DEMO COMPLETE!")
    print("="*70)
    print("\nWhat you learned:")
    print("  ✓ Sampling-based motion planning (36 candidates!)")
    print("  ✓ Multiple trajectory visualization")
    print("  ✓ Top-down camera that follows vehicle")
    print("  ✓ Trajectory selection and execution")
    print("  ✓ Actual vs planned path comparison")
    print("\nNext: Add rulebook to automatically select best trajectory!")
    print("="*70 + "\n")


if __name__ == "__main__":
    main()