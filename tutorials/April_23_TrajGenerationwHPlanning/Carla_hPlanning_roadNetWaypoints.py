"""
CARLA Demo: Visualized Waypoints + Road Following
==================================================

Shows waypoints as spheres and follows actual CARLA roads!

Author: Ahmad Ahmad
"""

import carla
import numpy as np
import random
import time
from enum import Enum
from dataclasses import dataclass
from typing import List


# ========================================
# TRAJECTORY & CONTEXT
# ========================================

@dataclass
class CARLATrajectory:
    """CARLA trajectory with waypoints and metadata."""
    id: int
    waypoints: List[carla.Location]
    
    # Metadata for filtering
    min_obstacle_distance: float
    max_speed: float
    violates_red_light: bool
    comfort: float
    fuel: float
    time: float


class Context(Enum):
    HIGHWAY = "highway"
    CITY = "city"
    PARKING = "parking"


# ========================================
# ROAD-FOLLOWING GENERATOR
# ========================================

class RoadFollowingGenerator:
    """Generate trajectories that follow CARLA roads."""
    
    def __init__(self, world, vehicle, goal):
        self.world = world
        self.vehicle = vehicle
        self.goal = goal
        self.map = world.get_map()
    
    def generate_road_path(self, distance_ahead=50.0, n_points=30):
        """
        Generate path following the road ahead.
        
        Uses CARLA's waypoint system to stay on roads!
        """
        # Get current waypoint
        vehicle_location = self.vehicle.get_location()
        current_waypoint = self.map.get_waypoint(vehicle_location)
        
        path = [vehicle_location]
        
        # Follow road ahead
        next_waypoints = current_waypoint.next(distance_ahead / n_points)
        
        for _ in range(n_points - 1):
            if next_waypoints:
                # Take first option (straight ahead)
                current_waypoint = next_waypoints[0]
                path.append(current_waypoint.transform.location)
                
                # Get next waypoints
                next_waypoints = current_waypoint.next(distance_ahead / n_points)
            else:
                # No more road, stop here
                break
        
        return path
    
    def generate_lane_change_path(self, direction='left', n_points=30):
        """Generate path with lane change."""
        vehicle_location = self.vehicle.get_location()
        current_waypoint = self.map.get_waypoint(vehicle_location)
        
        path = [vehicle_location]
        
        # Try to change lanes
        if direction == 'left' and current_waypoint.get_left_lane():
            target_lane = current_waypoint.get_left_lane()
        elif direction == 'right' and current_waypoint.get_right_lane():
            target_lane = current_waypoint.get_right_lane()
        else:
            # Can't change lanes, just go straight
            return self.generate_road_path(n_points=n_points)
        
        # Generate smooth transition
        for i in range(1, n_points):
            t = i / n_points
            
            # Interpolate between current and target lane
            if t < 0.3:  # First 30%: stay in current lane
                next_wps = current_waypoint.next(2.0 * i)
                if next_wps:
                    path.append(next_wps[0].transform.location)
            else:  # Last 70%: move to target lane
                next_wps = target_lane.next(2.0 * i)
                if next_wps:
                    path.append(next_wps[0].transform.location)
        
        return path
    
    def compute_path_length(self, path):
        """Compute total path length."""
        length = 0
        for i in range(len(path) - 1):
            length += path[i].distance(path[i+1])
        return length
    
    def compute_min_obstacle_distance(self, path):
        """Estimate min obstacle distance."""
        vehicles = self.world.get_actors().filter('vehicle.*')
        
        if len(vehicles) <= 1:
            return 10.0
        
        min_dist = float('inf')
        for waypoint in path[::5]:
            for other in vehicles:
                if other.id == self.vehicle.id:
                    continue
                dist = waypoint.distance(other.get_location())
                min_dist = min(min_dist, dist)
        
        return max(min_dist - 2.0, 0.0)
    
    def generate_trajectories(self, n=20):
        """Generate trajectories that follow roads."""
        trajectories = []
        
        print(f"   Generating {n} road-following trajectories...")
        
        for i in range(n):
            # Generate different types of paths
            if i % 3 == 0 and i > 0:
                # Lane change left
                path = self.generate_lane_change_path('left', n_points=30)
            elif i % 3 == 1 and i > 0:
                # Lane change right
                path = self.generate_lane_change_path('right', n_points=30)
            else:
                # Straight ahead on road
                distance = random.uniform(30, 60)
                path = self.generate_road_path(distance, n_points=30)
            
            # Skip if path is too short
            if len(path) < 10:
                continue
            
            # Compute metadata
            length = self.compute_path_length(path)
            min_dist = self.compute_min_obstacle_distance(path)
            
            traj = CARLATrajectory(
                id=i,
                waypoints=path,
                min_obstacle_distance=min_dist,
                max_speed=random.uniform(8, 15),
                violates_red_light=False,
                comfort=random.uniform(1, 10),
                fuel=length * 0.15,
                time=length / 10.0
            )
            
            trajectories.append(traj)
        
        print(f"   ✅ Generated {len(trajectories)} road-following trajectories")
        return trajectories


# ========================================
# WAYPOINT VISUALIZER
# ========================================

class WaypointVisualizer:
    """Draw waypoints in CARLA for debugging."""
    
    def __init__(self, world):
        self.world = world
        self.debug = world.debug
    
    def draw_waypoints(self, waypoints, color=carla.Color(0, 255, 0), 
                       life_time=30.0, size=0.2):
        """
        Draw waypoints as spheres in CARLA.
        
        Args:
            waypoints: List of carla.Location
            color: Color of spheres
            life_time: How long to show (seconds)
            size: Sphere radius
        """
        print(f"\n   Drawing {len(waypoints)} waypoints...")
        
        for i, waypoint in enumerate(waypoints):
            # Draw sphere at waypoint
            self.debug.draw_point(
                waypoint + carla.Location(z=0.5),  # Slightly above ground
                size=size,
                color=color,
                life_time=life_time
            )
            
            # Draw line to next waypoint
            if i < len(waypoints) - 1:
                self.debug.draw_line(
                    waypoint + carla.Location(z=0.5),
                    waypoints[i+1] + carla.Location(z=0.5),
                    thickness=0.1,
                    color=color,
                    life_time=life_time
                )
        
        # Draw start (green) and end (red) markers
        if waypoints:
            self.debug.draw_point(
                waypoints[0] + carla.Location(z=1.0),
                size=0.4,
                color=carla.Color(0, 255, 0),  # Green start
                life_time=life_time
            )
            self.debug.draw_point(
                waypoints[-1] + carla.Location(z=1.0),
                size=0.4,
                color=carla.Color(255, 0, 0),  # Red end
                life_time=life_time
            )
        
        print(f"   ✅ Waypoints visible in CARLA (green=start, red=end)")
    
    def clear_all(self):
        """Clear all debug drawings."""
        # Note: CARLA doesn't have a clear function
        # Drawings expire after life_time
        pass


# ========================================
# HIERARCHICAL PLANNER (same as before)
# ========================================

class CARLAHierarchicalPlanner:
    """Hierarchical planner with metadata filtering."""
    
    def __init__(self, safety_margin=2.0, speed_limit=15.0):
        self.safety_margin = safety_margin
        self.speed_limit = speed_limit
    
    def filter_by_safety_metadata(self, trajectories):
        """Filter using safety metadata."""
        safe = [t for t in trajectories 
                if t.min_obstacle_distance >= self.safety_margin]
        print(f"  Safety metadata: {len(trajectories)} → {len(safe)}")
        return safe
    
    def filter_by_legal_metadata(self, trajectories):
        """Filter using legal metadata."""
        legal = [t for t in trajectories 
                 if not t.violates_red_light and 
                    t.max_speed <= self.speed_limit]
        print(f"  Legal metadata: {len(trajectories)} → {len(legal)}")
        return legal
    
    def detect_context(self, vehicle):
        """Detect driving context."""
        velocity = vehicle.get_velocity()
        speed = 3.6 * np.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
        
        if speed > 90:
            return Context.HIGHWAY
        elif speed < 10:
            return Context.PARKING
        else:
            return Context.CITY
    
    def select_by_soft_metadata(self, trajectories, context):
        """Select best using soft metadata."""
        if context == Context.HIGHWAY:
            order_fn = lambda t: (t.time, t.fuel, t.comfort)
        elif context == Context.CITY:
            order_fn = lambda t: (t.comfort, t.time, t.fuel)
        else:
            order_fn = lambda t: (t.comfort, t.time, t.fuel)
        
        return min(trajectories, key=order_fn)
    
    def plan(self, candidates, vehicle, world):
        """Main hierarchical planning."""
        print(f"\n{'='*60}")
        print("HIERARCHICAL PLANNING WITH METADATA FILTERING")
        print(f"{'='*60}")
        print(f"\nInitial: {len(candidates)} trajectories")
        
        print(f"\nSTAGE 1: FILTER BY SAFETY METADATA")
        safe = self.filter_by_safety_metadata(candidates)
        if not safe:
            print("⚠️  NO SAFE TRAJECTORIES!")
            return None
        
        print(f"\nSTAGE 2: FILTER BY LEGAL METADATA")
        legal = self.filter_by_legal_metadata(safe)
        if not legal:
            print("⚠️  NO LEGAL TRAJECTORIES!")
            legal = safe
        
        print(f"\nSTAGE 3: CONTEXT DETECTION")
        context = self.detect_context(vehicle)
        print(f"  Context: {context.value}")
        
        print(f"\nSTAGE 4: SELECT BY SOFT METADATA")
        best = self.select_by_soft_metadata(legal, context)
        print(f"  Winner: Trajectory {best.id}")
        
        return best


# ========================================
# TRAJECTORY EXECUTOR (same as before)
# ========================================

class TrajectoryExecutor:
    """Execute trajectory with better control."""
    
    def __init__(self, vehicle, world):
        self.vehicle = vehicle
        self.world = world
        self.current_waypoint_index = 0
    
    def compute_control(self, trajectory, target_speed=8.0):
        """Compute vehicle control to follow trajectory."""
        
        if self.current_waypoint_index >= len(trajectory.waypoints):
            return carla.VehicleControl(throttle=0, brake=1.0)
        
        target = trajectory.waypoints[self.current_waypoint_index]
        current_location = self.vehicle.get_location()
        
        distance_to_waypoint = current_location.distance(target)
        if distance_to_waypoint < 2.0:
            self.current_waypoint_index += 1
            if self.current_waypoint_index >= len(trajectory.waypoints):
                return carla.VehicleControl(throttle=0, brake=1.0)
            target = trajectory.waypoints[self.current_waypoint_index]
        
        transform = self.vehicle.get_transform()
        forward_vector = transform.get_forward_vector()
        
        target_vector = target - current_location
        target_direction = np.array([target_vector.x, target_vector.y, 0])
        target_direction = target_direction / (np.linalg.norm(target_direction) + 1e-6)
        
        forward = np.array([forward_vector.x, forward_vector.y, 0])
        forward = forward / (np.linalg.norm(forward) + 1e-6)
        
        cross = np.cross(forward, target_direction)
        steer = np.clip(cross[2] * 3.0, -1.0, 1.0)
        
        current_speed = self.vehicle.get_velocity().length()
        speed_error = target_speed - current_speed
        
        if speed_error > 0:
            throttle = np.clip(speed_error * 0.3, 0.0, 0.8)
            brake = 0.0
        else:
            throttle = 0.0
            brake = np.clip(-speed_error * 0.5, 0.0, 1.0)
        
        return carla.VehicleControl(
            throttle=throttle,
            steer=steer,
            brake=brake
        )
    
    def execute(self, trajectory, visualizer, duration=15.0, target_speed=8.0):
        """Execute trajectory with visualization."""
        print(f"\n{'='*60}")
        print("EXECUTING TRAJECTORY")
        print(f"{'='*60}")
        
        # Draw the trajectory waypoints
        visualizer.draw_waypoints(
            trajectory.waypoints,
            color=carla.Color(0, 255, 0),  # Green path
            life_time=duration + 10.0
        )
        
        print(f"Duration: {duration}s")
        print(f"Target speed: {target_speed} m/s")
        print(f"Waypoints: {len(trajectory.waypoints)}")
        
        self.current_waypoint_index = 0
        start_time = time.time()
        step = 0
        
        spectator = self.world.get_spectator()
        
        try:
            while time.time() - start_time < duration:
                control = self.compute_control(trajectory, target_speed)
                self.vehicle.apply_control(control)
                
                if step % 5 == 0:
                    vehicle_transform = self.vehicle.get_transform()
                    spectator_transform = carla.Transform(
                        vehicle_transform.location + carla.Location(z=50),
                        carla.Rotation(pitch=-90)
                    )
                    spectator.set_transform(spectator_transform)
                
                if step % 10 == 0:
                    current_speed = self.vehicle.get_velocity().length()
                    progress = self.current_waypoint_index / len(trajectory.waypoints) * 100
                    
                    print(f"  Step {step:3d}: "
                          f"Speed={current_speed:4.1f} m/s, "
                          f"Progress={progress:5.1f}%, "
                          f"Waypoint {self.current_waypoint_index}/{len(trajectory.waypoints)}")
                
                if self.current_waypoint_index >= len(trajectory.waypoints):
                    print(f"\n  ✅ REACHED GOAL!")
                    break
                
                step += 1
                time.sleep(0.1)
        
        except KeyboardInterrupt:
            print(f"\n  ⚠️  Execution interrupted")
        
        self.vehicle.apply_control(carla.VehicleControl(brake=1.0))
        print(f"\n  Total steps: {step}")


# ========================================
# MAIN DEMO
# ========================================

def main():
    """Complete demo with waypoint visualization."""
    
    print("="*70)
    print(" CARLA: ROAD-FOLLOWING WITH WAYPOINT VISUALIZATION")
    print("="*70)
    
    client = None
    vehicle = None
    
    try:
        # Connect
        print("\n1. Connecting to CARLA...")
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)
        world = client.get_world()
        print("   ✅ Connected!")
        
        # Create visualizer
        visualizer = WaypointVisualizer(world)
        
        # Spawn vehicle
        print("\n2. Spawning vehicle...")
        blueprint_library = world.get_blueprint_library()
        vehicle_bp = blueprint_library.find('vehicle.tesla.model3')
        
        spawn_points = world.get_map().get_spawn_points()
        spawn_point = spawn_points[0]
        
        vehicle = world.spawn_actor(vehicle_bp, spawn_point)
        print(f"   ✅ Spawned Tesla at {spawn_point.location}")
        
        time.sleep(2)
        
        # Set goal
        goal = spawn_points[min(10, len(spawn_points)-1)].location
        print(f"   Goal: {goal}")
        
        # Generate trajectories (following roads!)
        print("\n3. Generating ROAD-FOLLOWING trajectories...")
        generator = RoadFollowingGenerator(world, vehicle, goal)
        trajectories = generator.generate_trajectories(n=200)
        
        # Plan
        print("\n4. Hierarchical planning...")
        planner = CARLAHierarchicalPlanner()
        best = planner.plan(trajectories, vehicle, world)
        
        if not best:
            print("\n❌ No valid trajectory!")
            return
        
        print(f"\n   Selected: Trajectory {best.id}")
        print(f"   Waypoints: {len(best.waypoints)}")
        
        # Execute with visualization
        print("\n5. Executing (GREEN SPHERES = WAYPOINTS)...")
        print("   Look at CARLA - you'll see green spheres showing the path!")
        
        executor = TrajectoryExecutor(vehicle, world)
        executor.execute(best, visualizer, duration=20.0, target_speed=6.0)
        
        print(f"\n{'='*70}")
        print("DEMO COMPLETE! 🎉")
        print(f"{'='*70}")
        print("\nThe green spheres show where the car is trying to go!")
        print("Now it follows roads instead of going through buildings!")
        
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
        
    finally:
        print("\n6. Cleaning up...")
        if vehicle:
            time.sleep(3)  # Keep visible
            vehicle.destroy()
            print("   ✅ Vehicle destroyed")
            print("\n   (Waypoint spheres will fade after 30s)")


if __name__ == "__main__":
    main()