"""
CARLA Integration: Sampling-Based Hierarchical Planning
========================================================

Port of Python sampling planner to CARLA simulator.

Author: Ahmad Ahmad
Student: Nidhi
"""

import carla
import numpy as np
import random
import time
from enum import Enum
from dataclasses import dataclass
from typing import List, Tuple


# ========================================
# PART 1: TRAJECTORY REPRESENTATION (CARLA)
# ========================================

@dataclass
class CARLATrajectory:
    """
    CARLA trajectory with 3D waypoints and metrics.
    
    Same metrics as Python version, but with CARLA types!
    """
    id: int
    waypoints: List[carla.Location]  # 3D positions
    
    # Hard constraints
    min_obstacle_distance: float
    max_speed: float
    violates_red_light: bool
    
    # Soft objectives
    comfort: float
    fuel: float
    time: float


# ========================================
# PART 2: CARLA SAMPLING-BASED GENERATOR
# ========================================

class CARLASamplingGenerator:
    """
    Generate trajectories in CARLA using sampling.
    
    Similar to Python version, but uses CARLA map!
    """
    
    def __init__(self, world: carla.World, vehicle: carla.Vehicle,
                 goal_location: carla.Location):
        self.world = world
        self.vehicle = vehicle
        self.goal = goal_location
        self.map = world.get_map()
    
    def sample_waypoint_on_road(self):
        """Sample random waypoint that's actually on a road."""
        # Get random spawn point (guaranteed to be on road)
        spawn_points = self.map.get_spawn_points()
        random_spawn = random.choice(spawn_points)
        return random_spawn.location
    
    def sample_nearby_waypoint(self, location, radius=20.0):
        """Sample waypoint near given location."""
        # Get current waypoint
        current_wp = self.map.get_waypoint(location)
        
        # Sample ahead or to the side
        if random.random() < 0.5:
            # Sample ahead
            distance = random.uniform(5, radius)
            next_wps = current_wp.next(distance)
            if next_wps:
                return next_wps[0].transform.location
        else:
            # Sample lane change
            if random.random() < 0.5 and current_wp.get_left_lane():
                return current_wp.get_left_lane().transform.location
            elif current_wp.get_right_lane():
                return current_wp.get_right_lane().transform.location
        
        return current_wp.transform.location
    
    def generate_path_to_location(self, target: carla.Location, 
                                  n_waypoints=20):
        """
        Generate path from vehicle to target using CARLA map.
        
        Uses CARLA's built-in waypoint system.
        """
        current_location = self.vehicle.get_location()
        
        # Get waypoints along route
        start_wp = self.map.get_waypoint(current_location)
        target_wp = self.map.get_waypoint(target)
        
        # Simple linear interpolation for now
        # (In real system, use A* or CARLA's routing)
        path = []
        for i in range(n_waypoints):
            t = i / (n_waypoints - 1)
            x = current_location.x * (1 - t) + target.x * t
            y = current_location.y * (1 - t) + target.y * t
            z = current_location.z * (1 - t) + target.z * t
            path.append(carla.Location(x, y, z))
        
        return path
    
    def compute_path_length(self, path):
        """Compute total path length."""
        length = 0
        for i in range(len(path) - 1):
            length += path[i].distance(path[i+1])
        return length
    
    def compute_curvature(self, path):
        """Compute path curvature (comfort metric)."""
        if len(path) < 3:
            return 0.0
        
        max_angle = 0.0
        for i in range(len(path) - 2):
            v1 = np.array([path[i+1].x - path[i].x,
                          path[i+1].y - path[i].y])
            v2 = np.array([path[i+2].x - path[i+1].x,
                          path[i+2].y - path[i+1].y])
            
            if np.linalg.norm(v1) > 0 and np.linalg.norm(v2) > 0:
                cos_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * 
                                             np.linalg.norm(v2))
                angle = np.arccos(np.clip(cos_angle, -1, 1))
                max_angle = max(max_angle, angle)
        
        return max_angle
    
    def compute_min_obstacle_distance(self, path):
        """Compute minimum distance to obstacles using CARLA."""
        min_dist = float('inf')
        
        # Get nearby vehicles
        vehicles = self.world.get_actors().filter('vehicle.*')
        walkers = self.world.get_actors().filter('walker.*')
        obstacles = list(vehicles) + list(walkers)
        
        # Exclude our own vehicle
        obstacles = [obs for obs in obstacles if obs.id != self.vehicle.id]
        
        for waypoint in path:
            for obstacle in obstacles:
                obs_loc = obstacle.get_location()
                dist = waypoint.distance(obs_loc)
                
                # Subtract obstacle bounding box
                if hasattr(obstacle, 'bounding_box'):
                    extent = obstacle.bounding_box.extent
                    obs_radius = max(extent.x, extent.y)
                    dist -= obs_radius
                
                min_dist = min(min_dist, dist)
        
        return max(0, min_dist)
    
    def check_traffic_light_violation(self, path):
        """Check if path violates traffic light."""
        # Check if vehicle is at traffic light
        if self.vehicle.is_at_traffic_light():
            traffic_light = self.vehicle.get_traffic_light()
            
            # If red or yellow
            if traffic_light.state != carla.TrafficLightState.Green:
                # Check if path crosses stop line
                # (Simplified: assume path crosses if it continues forward)
                if len(path) > 5:
                    # If path moves forward significantly
                    distance_moved = path[0].distance(path[5])
                    if distance_moved > 3.0:  # 3 meters
                        return True
        
        return False
    
    def generate_trajectories(self, n=200, seed=42):
        """
        Generate n candidate trajectories in CARLA.
        
        Strategy:
        - 20% straight to goal
        - 80% via sampled waypoints
        """
        random.seed(seed)
        np.random.seed(seed)
        
        trajectories = []
        
        for i in range(n):
            try:
                # Generate path
                if random.random() < 0.2:
                    # Straight to goal
                    path = self.generate_path_to_location(self.goal, 
                                                          n_waypoints=20)
                else:
                    # Via random waypoint
                    current_loc = self.vehicle.get_location()
                    intermediate = self.sample_nearby_waypoint(current_loc, 
                                                              radius=30)
                    
                    # Path: current → intermediate → goal
                    path1 = self.generate_path_to_location(intermediate, 
                                                           n_waypoints=10)
                    path2 = self.generate_path_to_location(self.goal, 
                                                           n_waypoints=10)
                    path = path1 + path2
                
                # Compute metrics
                path_length = self.compute_path_length(path)
                curvature = self.compute_curvature(path)
                min_dist = self.compute_min_obstacle_distance(path)
                violates_light = self.check_traffic_light_violation(path)
                
                # Create trajectory
                traj = CARLATrajectory(
                    id=i,
                    waypoints=path,
                    
                    # Hard constraints
                    min_obstacle_distance=min_dist,
                    max_speed=random.uniform(8, 18),  # m/s
                    violates_red_light=violates_light,
                    
                    # Soft objectives
                    comfort=curvature * 10,
                    fuel=path_length * 0.15,
                    time=path_length / 10
                )
                
                trajectories.append(traj)
                
            except Exception as e:
                print(f"Warning: Failed to generate trajectory {i}: {e}")
                continue
        
        return trajectories


# ========================================
# PART 3: CARLA HIERARCHICAL PLANNER
# ========================================

class Context(Enum):
    HIGHWAY = "highway"
    CITY = "city"
    PARKING = "parking"


class CARLAHierarchicalPlanner:
    """
    Hierarchical planner for CARLA.
    
    Same algorithm, CARLA integration!
    """
    
    def __init__(self, safety_margin=1.5, speed_limit=15.0):
        self.safety_margin = safety_margin
        self.speed_limit = speed_limit
    
    def filter_safety(self, trajectories):
        """P0: Safety filter for CARLA."""
        safe = []
        for traj in trajectories:
            if traj.min_obstacle_distance >= self.safety_margin:
                safe.append(traj)
        
        print(f"  Safety: {len(trajectories)} → {len(safe)}")
        return safe
    
    def filter_legal(self, trajectories):
        """P1: Legal filter for CARLA."""
        legal = []
        for traj in trajectories:
            if not traj.violates_red_light and \
               traj.max_speed <= self.speed_limit:
                legal.append(traj)
        
        print(f"  Legal: {len(trajectories)} → {len(legal)}")
        return legal
    
    def detect_context(self, vehicle, world):
        """Detect context from CARLA state."""
        # Get current waypoint
        waypoint = world.get_map().get_waypoint(vehicle.get_location())
        
        # Get speed
        velocity = vehicle.get_velocity()
        speed = 3.6 * np.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)  # km/h
        
        # Detect context
        if speed > 90 and waypoint.lane_type == carla.LaneType.Driving:
            return Context.HIGHWAY
        elif speed < 10:
            return Context.PARKING
        else:
            return Context.CITY
    
    def get_total_order(self, context):
        """Get context-specific total order."""
        if context == Context.HIGHWAY:
            return lambda t: (t.time, t.fuel, t.comfort)
        elif context == Context.CITY:
            return lambda t: (t.comfort, t.time, t.fuel)
        else:  # PARKING
            return lambda t: (t.comfort, t.time, t.fuel)
    
    def plan(self, candidates, vehicle, world):
        """Main hierarchical planning for CARLA."""
        print(f"\n{'='*60}")
        print("CARLA HIERARCHICAL PLANNING")
        print(f"{'='*60}")
        
        print(f"\nInitial: {len(candidates)} trajectories")
        
        # Stage 1: Filtering
        print(f"\nSTAGE 1: HARD FILTERING")
        safe = self.filter_safety(candidates)
        if not safe:
            print("⚠️  NO SAFE OPTIONS - EMERGENCY BRAKE!")
            return self.emergency_brake(vehicle)
        
        legal = self.filter_legal(safe)
        if not legal:
            print("⚠️  NO LEGAL OPTIONS - Using best safe")
            legal = safe
        
        # Stage 2: Context
        print(f"\nSTAGE 2: CONTEXT DETECTION")
        context = self.detect_context(vehicle, world)
        print(f"  Context: {context.value}")
        
        # Stage 3: Selection
        print(f"\nSTAGE 3: SELECTION")
        order_fn = self.get_total_order(context)
        best = min(legal, key=order_fn)
        print(f"  Winner: Trajectory {best.id}")
        
        return best
    
    def emergency_brake(self, vehicle):
        """Emergency brake trajectory."""
        print("🚨 EMERGENCY BRAKE!")
        
        # Create stopping trajectory
        current_loc = vehicle.get_location()
        brake_path = [current_loc]
        
        for i in range(10):
            # Straight ahead, decelerating
            brake_path.append(current_loc)
        
        return CARLATrajectory(
            id=-1,
            waypoints=brake_path,
            min_obstacle_distance=0,
            max_speed=0,
            violates_red_light=False,
            comfort=10,  # Hard brake
            fuel=0,
            time=2
        )


# ========================================
# PART 4: TRAJECTORY EXECUTION
# ========================================

def execute_trajectory(trajectory, vehicle):
    """
    Execute trajectory in CARLA.
    
    Convert trajectory to vehicle control commands.
    """
    if len(trajectory.waypoints) == 0:
        return carla.VehicleControl()
    
    # Get target waypoint (first in path)
    target = trajectory.waypoints[0]
    current_location = vehicle.get_location()
    
    # Compute steering
    direction = target - current_location
    forward = vehicle.get_transform().get_forward_vector()
    
    # Cross product for steering
    cross = direction.x * forward.y - direction.y * forward.x
    steer = np.clip(cross * 2.0, -1.0, 1.0)
    
    # Compute throttle/brake
    current_speed = vehicle.get_velocity().length()
    target_speed = trajectory.time  # Use time as proxy for speed
    
    speed_error = target_speed - current_speed
    
    if speed_error > 0:
        throttle = np.clip(speed_error * 0.5, 0, 1)
        brake = 0.0
    else:
        throttle = 0.0
        brake = np.clip(-speed_error * 0.5, 0, 1)
    
    control = carla.VehicleControl(
        throttle=throttle,
        steer=steer,
        brake=brake
    )
    
    return control


# ========================================
# PART 5: MAIN CARLA LOOP
# ========================================

def main_carla():
    """
    Complete CARLA demo.
    
    1. Connect to CARLA
    2. Spawn vehicle
    3. Generate trajectories (sampling)
    4. Evaluate (hierarchical)
    5. Execute best
    """
    
    print("="*70)
    print(" CARLA SAMPLING-BASED HIERARCHICAL PLANNING")
    print("="*70)
    
    try:
        # Connect to CARLA
        print("\n1. Connecting to CARLA...")
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)
        world = client.get_world()
        print("   ✅ Connected!")
        
        # Spawn vehicle
        print("\n2. Spawning vehicle...")
        blueprint_library = world.get_blueprint_library()
        vehicle_bp = blueprint_library.find('vehicle.tesla.model3')
        
        spawn_points = world.get_map().get_spawn_points()
        spawn_point = spawn_points[0]
        
        vehicle = world.spawn_actor(vehicle_bp, spawn_point)
        print(f"   ✅ Spawned Tesla Model 3 at {spawn_point.location}")
        
        # Set goal
        goal_point = spawn_points[min(10, len(spawn_points)-1)]
        goal = goal_point.location
        print(f"   Goal: {goal}")
        
        # Wait for vehicle to settle
        time.sleep(2)
        
        # Generate trajectories
        print("\n3. Generating trajectories...")
        generator = CARLASamplingGenerator(world, vehicle, goal)
        
        start_time = time.time()
        trajectories = generator.generate_trajectories(n=50, seed=42)
        gen_time = time.time() - start_time
        
        print(f"   Generated: {len(trajectories)} trajectories")
        print(f"   Time: {gen_time*1000:.1f}ms")
        
        # Evaluate with planner
        print("\n4. Evaluating with hierarchical planner...")
        planner = CARLAHierarchicalPlanner(
            safety_margin=2.0,
            speed_limit=15.0
        )
        
        best = planner.plan(trajectories, vehicle, world)
        
        if best and best.id != -1:
            print(f"\n   ✅ Selected trajectory {best.id}")
            print(f"   Path length: {generator.compute_path_length(best.waypoints):.1f}m")
            print(f"   Comfort: {best.comfort:.2f}")
            
            # Execute for a few steps
            print("\n5. Executing trajectory...")
            for step in range(20):
                control = execute_trajectory(best, vehicle)
                vehicle.apply_control(control)
                time.sleep(0.1)
            
            print("   ✅ Execution complete!")
        
        # Cleanup
        print("\n6. Cleaning up...")
        vehicle.destroy()
        print("   ✅ Vehicle destroyed")
        
        print(f"\n{'='*70}")
        print("CARLA DEMO COMPLETE! 🎉")
        print(f"{'='*70}")
        
    except Exception as e:
        print(f"\n❌ Error: {e}")
        print("\nMake sure CARLA is running:")
        print("  1. Start CARLA: ./CarlaUE4.sh")
        print("  2. Run this script")


if __name__ == "__main__":
    main_carla()
