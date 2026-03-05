"""
Path Following Controller - CARLA Demo
=======================================

This script demonstrates:
1. Generate candidate trajectories
2. Select one trajectory
3. Actually FOLLOW it with the vehicle!

Uses Pure Pursuit controller - simple but effective path tracking.

Author: Ahmad Ahmad
For: Nidhi (Curious Cardinals Mentorship)
Date: January 2026
"""

import carla
import numpy as np
import matplotlib.pyplot as plt
from typing import List, Tuple, Optional
import time
import math


# ============================================================================
# QUINTIC POLYNOMIAL (same as before)
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
# REFERENCE PATH (same as before)
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
# TRAJECTORY GENERATOR (simplified)
# ============================================================================

def generate_single_trajectory(s0: float, d0: float, s_dot0: float,
                               d_final: float, v_final: float, T: float,
                               reference_path: ReferencePath) -> dict:
    """
    Generate a single trajectory in Frenet frame and convert to global.
    
    Args:
        s0, d0: Initial position in Frenet frame
        s_dot0: Initial longitudinal velocity
        d_final: Target lateral position (lane)
        v_final: Target velocity
        T: Time duration
        reference_path: Reference path object
    
    Returns:
        Trajectory dictionary with waypoints
    """
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
    """
    Pure Pursuit path tracking controller.
    
    Simple but effective algorithm:
    1. Look ahead along the path by distance L
    2. Steer towards that lookahead point
    3. Adjust throttle to match target velocity
    """
    
    def __init__(self, wheelbase: float = 2.7, lookahead_distance: float = 5.0):
        """
        Initialize controller.
        
        Args:
            wheelbase: Distance between front and rear axles (meters)
            lookahead_distance: How far ahead to look on path (meters)
        """
        self.wheelbase = wheelbase
        self.lookahead_distance = lookahead_distance
        
        # PID gains for throttle control
        self.Kp_throttle = 0.5  # Proportional gain
        self.Ki_throttle = 0.01  # Integral gain
        self.Kd_throttle = 0.1   # Derivative gain
        
        # State for PID
        self.velocity_error_integral = 0.0
        self.prev_velocity_error = 0.0
    
    def find_lookahead_point(self, vehicle_x: float, vehicle_y: float,
                            waypoints: List[dict]) -> Optional[dict]:
        """
        Find the point on the path that is lookahead_distance ahead.
        
        Args:
            vehicle_x, vehicle_y: Current vehicle position
            waypoints: List of waypoint dictionaries
            
        Returns:
            Lookahead waypoint or None if path completed
        """
        # Find closest waypoint to vehicle
        min_dist = float('inf')
        closest_idx = 0
        
        for i, wp in enumerate(waypoints):
            dist = math.sqrt((wp['x'] - vehicle_x)**2 + (wp['y'] - vehicle_y)**2)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        # Look ahead from closest point
        for i in range(closest_idx, len(waypoints)):
            wp = waypoints[i]
            dist = math.sqrt((wp['x'] - vehicle_x)**2 + (wp['y'] - vehicle_y)**2)
            
            if dist >= self.lookahead_distance:
                return wp
        
        # If no point far enough, return last waypoint
        if len(waypoints) > 0:
            return waypoints[-1]
        
        return None
    
    def compute_steering_angle(self, vehicle_x: float, vehicle_y: float,
                               vehicle_yaw: float, lookahead_point: dict) -> float:
        """
        Compute steering angle using Pure Pursuit geometry.
        
        The formula is: steering = atan(2 * L * sin(alpha) / lookahead_distance)
        where:
            L = wheelbase
            alpha = angle between vehicle heading and lookahead point
        
        Args:
            vehicle_x, vehicle_y: Vehicle position
            vehicle_yaw: Vehicle heading (radians)
            lookahead_point: Target point on path
            
        Returns:
            Steering angle in radians
        """
        # Vector from vehicle to lookahead point
        dx = lookahead_point['x'] - vehicle_x
        dy = lookahead_point['y'] - vehicle_y
        
        # Angle to lookahead point
        angle_to_point = math.atan2(dy, dx)
        
        # Alpha: angle between vehicle heading and lookahead point
        alpha = self._normalize_angle(angle_to_point - vehicle_yaw)
        
        # Pure Pursuit formula
        steering = math.atan(2.0 * self.wheelbase * math.sin(alpha) / self.lookahead_distance)
        
        # Clamp to reasonable limits (-30° to +30°)
        max_steer = math.radians(30)
        steering = np.clip(steering, -max_steer, max_steer)
        
        return steering
    
    def compute_throttle(self, current_velocity: float, target_velocity: float,
                        dt: float = 0.05) -> Tuple[float, float]:
        """
        Compute throttle and brake using PID control.
        
        Args:
            current_velocity: Current vehicle speed (m/s)
            target_velocity: Desired speed (m/s)
            dt: Time step (seconds)
            
        Returns:
            (throttle, brake) both in [0, 1]
        """
        # Velocity error
        error = target_velocity - current_velocity
        
        # PID terms
        P = self.Kp_throttle * error
        self.velocity_error_integral += error * dt
        I = self.Ki_throttle * self.velocity_error_integral
        D = self.Kd_throttle * (error - self.prev_velocity_error) / dt
        
        # Combined control signal
        control = P + I + D
        
        # Split into throttle and brake
        if control > 0:
            throttle = np.clip(control, 0.0, 1.0)
            brake = 0.0
        else:
            throttle = 0.0
            brake = np.clip(-control, 0.0, 1.0)
        
        # Update for next iteration
        self.prev_velocity_error = error
        
        return throttle, brake
    
    def compute_control(self, vehicle_transform: carla.Transform,
                       vehicle_velocity: carla.Vector3D,
                       waypoints: List[dict]) -> Optional[carla.VehicleControl]:
        """
        Main control function - computes steering, throttle, and brake.
        
        Args:
            vehicle_transform: Current vehicle transform
            vehicle_velocity: Current vehicle velocity
            waypoints: Path waypoints to follow
            
        Returns:
            VehicleControl object or None if path completed
        """
        # Extract vehicle state
        vehicle_x = vehicle_transform.location.x
        vehicle_y = vehicle_transform.location.y
        vehicle_yaw = math.radians(vehicle_transform.rotation.yaw)
        current_speed = math.sqrt(vehicle_velocity.x**2 + vehicle_velocity.y**2)
        
        # Find lookahead point
        lookahead = self.find_lookahead_point(vehicle_x, vehicle_y, waypoints)
        
        if lookahead is None:
            return None  # Path completed
        
        # Compute steering
        steering = self.compute_steering_angle(vehicle_x, vehicle_y, vehicle_yaw, lookahead)
        
        # Compute throttle/brake
        target_velocity = lookahead['velocity']
        throttle, brake = self.compute_throttle(current_speed, target_velocity)
        
        # Create control command
        control = carla.VehicleControl()
        control.steer = float(steering / math.radians(30))  # Normalize to [-1, 1]
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
# CAMERA FOLLOWING
# ============================================================================

def update_camera(spectator: carla.Actor, vehicle: carla.Actor,
                 camera_height: float = 30.0, camera_distance: float = 20.0):
    """
    Update camera to follow vehicle from above and behind.
    
    Args:
        spectator: CARLA spectator (camera)
        vehicle: Vehicle to follow
        camera_height: Height above vehicle (meters)
        camera_distance: Distance behind vehicle (meters)
    """
    vehicle_transform = vehicle.get_transform()
    vehicle_loc = vehicle_transform.location
    vehicle_yaw = math.radians(vehicle_transform.rotation.yaw)
    
    # Calculate camera position: behind and above vehicle
    camera_x = vehicle_loc.x - camera_distance * math.cos(vehicle_yaw)
    camera_y = vehicle_loc.y - camera_distance * math.sin(vehicle_yaw)
    camera_z = vehicle_loc.z + camera_height
    
    camera_location = carla.Location(x=camera_x, y=camera_y, z=camera_z)
    camera_rotation = carla.Rotation(
        pitch=-45,  # Look down at 45 degrees
        yaw=vehicle_transform.rotation.yaw,  # Match vehicle heading
        roll=0
    )
    
    spectator.set_transform(carla.Transform(camera_location, camera_rotation))


# ============================================================================
# VISUALIZATION
# ============================================================================

def visualize_path_following(world: carla.World, trajectory: dict,
                            vehicle_positions: List[Tuple[float, float]]):
    """
    Draw the trajectory and vehicle path in CARLA.
    
    Args:
        world: CARLA world
        trajectory: Trajectory dictionary with waypoints
        vehicle_positions: List of (x, y) positions vehicle actually followed
    """
    debug = world.debug
    
    # Draw reference trajectory (green)
    waypoints = trajectory['waypoints']
    for i in range(len(waypoints) - 1):
        wp1 = waypoints[i]
        wp2 = waypoints[i + 1]
        
        start = carla.Location(x=wp1['x'], y=wp1['y'], z=2.0)
        end = carla.Location(x=wp2['x'], y=wp2['y'], z=2.0)
        
        debug.draw_line(start, end, thickness=0.3,
                       color=carla.Color(0, 255, 0, 200),
                       life_time=30.0)
    
    # Draw actual vehicle path (red)
    for i in range(len(vehicle_positions) - 1):
        p1 = vehicle_positions[i]
        p2 = vehicle_positions[i + 1]
        
        start = carla.Location(x=p1[0], y=p1[1], z=2.5)
        end = carla.Location(x=p2[0], y=p2[1], z=2.5)
        
        debug.draw_line(start, end, thickness=0.2,
                       color=carla.Color(255, 0, 0, 255),
                       life_time=30.0)


# ============================================================================
# MAIN DEMO
# ============================================================================

def main():
    """Main demonstration of path following."""
    
    print("\n" + "="*70)
    print("PATH FOLLOWING CONTROLLER - CARLA DEMO")
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
    
    # Position camera
    print("\n[STEP 3] Setting up camera...")
    spectator = world.get_spectator()
    
    # Initial camera position (will update dynamically)
    update_camera(spectator, vehicle, camera_height=30.0, camera_distance=20.0)
    print("✓ Camera will follow vehicle from above and behind")
    
    # Create reference path
    print("\n[STEP 4] Creating reference path...")
    vehicle_transform = vehicle.get_transform()
    reference_path = ReferencePath(
        start_location=vehicle_transform.location,
        length=100.0,
        heading=vehicle_transform.rotation.yaw
    )
    print("✓ Reference path created")
    
    # Generate trajectory
    print("\n[STEP 5] Generating trajectory...")
    print("\nChoose trajectory type:")
    print("  1. Stay in lane (straight)")
    print("  2. Lane change RIGHT")
    print("  3. Lane change LEFT")
    
    choice = input("Enter choice (1-3) [default=2]: ").strip()
    if choice == '1':
        d_final = 0.0
        maneuver = "STAY IN LANE"
    elif choice == '3':
        d_final = -3.5
        maneuver = "LANE CHANGE LEFT"
    else:
        d_final = 3.5
        maneuver = "LANE CHANGE RIGHT"
    
    trajectory = generate_single_trajectory(
        s0=0.0,
        d0=0.0,
        s_dot0=5.0,  # Start slow
        d_final=d_final,
        v_final=12.0,  # Target 12 m/s
        T=6.0,  # Take 6 seconds
        reference_path=reference_path
    )
    
    print(f"✓ Generated trajectory: {maneuver}")
    print(f"  Duration: {trajectory['duration']}s")
    print(f"  Waypoints: {len(trajectory['waypoints'])}")
    
    # Create controller
    print("\n[STEP 6] Creating Pure Pursuit controller...")
    controller = PurePursuitController(
        wheelbase=2.7,
        lookahead_distance=8.0
    )
    print("✓ Controller ready")
    
    # Follow the path!
    print("\n[STEP 7] Following trajectory...")
    print("="*70)
    print("WATCH THE CARLA WINDOW!")
    print("  🟢 Green line = Reference trajectory")
    print("  🔴 Red line = Actual vehicle path")
    print("="*70)
    
    waypoints = trajectory['waypoints']
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
            
            # Update camera to follow vehicle
            update_camera(spectator, vehicle, camera_height=30.0, camera_distance=20.0)
            
            # Compute control
            control = controller.compute_control(
                vehicle_transform,
                vehicle_velocity,
                waypoints
            )
            
            if control is None:
                print("\n✓ Path completed!")
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
            if elapsed > trajectory['duration'] + 5.0:
                print("\n\nTimeout reached")
                break
    
    except KeyboardInterrupt:
        print("\n\nStopped by user")
    
    # Stop vehicle
    vehicle.apply_control(carla.VehicleControl(brake=1.0))
    time.sleep(0.5)
    
    # Visualize
    print("\n\n[STEP 8] Drawing visualization...")
    visualize_path_following(world, trajectory, vehicle_positions)
    print("✓ Visualization drawn (lasts 30 seconds)")
    
    # Cleanup
    print("\n[STEP 9] Cleaning up...")
    print("Visualization will remain for 30 seconds...")
    print("Press Ctrl+C to exit and destroy vehicle")
    
    try:
        time.sleep(30)
    except KeyboardInterrupt:
        pass
    
    vehicle.destroy()
    print("✓ Vehicle destroyed")
    
    print("\n" + "="*70)
    print("DEMO COMPLETE!")
    print("="*70)
    print("\nWhat you learned:")
    print("  ✓ How to generate a trajectory")
    print("  ✓ How Pure Pursuit controller works")
    print("  ✓ How to apply vehicle controls")
    print("  ✓ How to track a reference path")
    print("\nNext: Implement MPC for even better tracking!")
    print("="*70 + "\n")


if __name__ == "__main__":
    main()
