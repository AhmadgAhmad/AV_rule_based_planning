"""
CARLA Road Network: Complete Guide
===================================

Understanding how CARLA represents and provides road information.

Author: Ahmad Ahmad
"""

import carla
import random


# ========================================
# PART 1: THE MAP OBJECT
# ========================================

def explore_carla_map(world):
    """
    The Map is CARLA's representation of the road network.
    
    It contains:
    - Road geometry (lanes, intersections)
    - Road topology (connections between roads)
    - Traffic rules (speed limits, lane markings)
    - Landmarks (traffic lights, signs)
    """
    
    print("="*70)
    print("PART 1: THE MAP OBJECT")
    print("="*70)
    
    # Get the map
    carla_map = world.get_map()
    
    print(f"\n1. Map name: {carla_map.name}")
    print(f"   Example: 'Town01', 'Town02', etc.")
    
    # Get spawn points (predefined safe locations to spawn vehicles)
    spawn_points = carla_map.get_spawn_points()
    print(f"\n2. Spawn points: {len(spawn_points)} available")
    print(f"   These are safe locations on roads where vehicles can start")
    
    # Get topology (how roads connect)
    topology = carla_map.get_topology()
    print(f"\n3. Topology: {len(topology)} road segments")
    print(f"   Topology = pairs of (start_waypoint, end_waypoint)")
    print(f"   Shows how road segments connect to each other")
    
    return carla_map


# ========================================
# PART 2: WAYPOINTS - THE KEY CONCEPT
# ========================================

def explore_waypoints(world, vehicle):
    """
    Waypoints are THE CORE of CARLA's road network.
    
    A waypoint represents a DIRECTED POINT on a lane:
    - Position (x, y, z)
    - Orientation (which direction the lane goes)
    - Lane information (type, width, markings)
    - Road information (road ID, section)
    """
    
    print("\n" + "="*70)
    print("PART 2: WAYPOINTS")
    print("="*70)
    
    carla_map = world.get_map()
    
    # Get vehicle's current location
    vehicle_location = vehicle.get_location()
    print(f"\n1. Vehicle location: {vehicle_location}")
    
    # Get the waypoint at this location
    # This is THE FUNDAMENTAL OPERATION!
    waypoint = carla_map.get_waypoint(vehicle_location)
    
    print(f"\n2. Waypoint at vehicle location:")
    print(f"   Position: {waypoint.transform.location}")
    print(f"   Rotation: {waypoint.transform.rotation}")
    print(f"   Road ID: {waypoint.road_id}")
    print(f"   Section ID: {waypoint.section_id}")
    print(f"   Lane ID: {waypoint.lane_id}")
    
    # Waypoint contains rich information
    print(f"\n3. Lane information:")
    print(f"   Lane type: {waypoint.lane_type}")
    print(f"      - Driving: Normal driving lane")
    print(f"      - Parking: Parking area")
    print(f"      - Shoulder: Road shoulder")
    print(f"      - Sidewalk: Pedestrian sidewalk")
    
    print(f"\n   Lane width: {waypoint.lane_width:.2f} meters")
    
    print(f"\n   Lane change allowed:")
    print(f"      - Left: {waypoint.lane_change}")
    print(f"      - Right: {waypoint.lane_change}")
    
    print(f"\n4. Road information:")
    print(f"   Is junction: {waypoint.is_junction}")
    print(f"      (Junction = intersection)")
    
    # Get landmark information (traffic lights, signs, etc.)
    landmarks = waypoint.get_landmarks(distance=50.0)
    print(f"\n5. Landmarks nearby: {len(landmarks)}")
    for landmark in landmarks[:3]:  # Show first 3
        print(f"   - {landmark.type} at {landmark.distance:.1f}m")
    
    return waypoint


# ========================================
# PART 3: NAVIGATING THE ROAD NETWORK
# ========================================

def navigate_road_network(world, waypoint):
    """
    How to move along the road network using waypoints.
    
    Key methods:
    - next(distance) → waypoints ahead
    - previous(distance) → waypoints behind
    - get_left_lane() → waypoint in left lane
    - get_right_lane() → waypoint in right lane
    """
    
    print("\n" + "="*70)
    print("PART 3: NAVIGATING THE ROAD NETWORK")
    print("="*70)
    
    # Method 1: next() - Get waypoints ahead
    print("\n1. next(distance) - Move forward along the road:")
    
    distance = 10.0  # meters
    next_waypoints = waypoint.next(distance)
    
    print(f"   Starting at: {waypoint.transform.location}")
    print(f"   Waypoints {distance}m ahead: {len(next_waypoints)}")
    print(f"   (Multiple if road splits/branches)")
    
    for i, wp in enumerate(next_waypoints):
        print(f"   Option {i+1}: {wp.transform.location}")
        print(f"            Road ID: {wp.road_id}, Lane ID: {wp.lane_id}")
    
    # Method 2: previous() - Get waypoints behind
    print(f"\n2. previous(distance) - Move backward along the road:")
    
    prev_waypoints = waypoint.previous(distance)
    print(f"   Waypoints {distance}m behind: {len(prev_waypoints)}")
    
    # Method 3: get_left_lane() - Change lanes
    print(f"\n3. get_left_lane() - Move to adjacent lane:")
    
    left_waypoint = waypoint.get_left_lane()
    if left_waypoint:
        print(f"   ✅ Left lane exists!")
        print(f"      Lane ID: {waypoint.lane_id} → {left_waypoint.lane_id}")
        print(f"      Position: {left_waypoint.transform.location}")
    else:
        print(f"   ❌ No left lane (at road edge)")
    
    # Method 4: get_right_lane() - Change lanes
    print(f"\n4. get_right_lane() - Move to adjacent lane:")
    
    right_waypoint = waypoint.get_right_lane()
    if right_waypoint:
        print(f"   ✅ Right lane exists!")
        print(f"      Lane ID: {waypoint.lane_id} → {right_waypoint.lane_id}")
    else:
        print(f"   ❌ No right lane (at road edge)")


# ========================================
# PART 4: GENERATING PATHS ON ROADS
# ========================================

def generate_road_following_path(world, waypoint, distance=50.0, step=2.0):
    """
    Generate a path that follows the road network.
    
    This is what we use for trajectory generation!
    """
    
    print("\n" + "="*70)
    print("PART 4: GENERATING PATHS ON ROADS")
    print("="*70)
    
    print(f"\n1. Generating path following the road:")
    print(f"   Starting waypoint: {waypoint.transform.location}")
    print(f"   Total distance: {distance}m")
    print(f"   Step size: {step}m")
    
    path = [waypoint]
    current = waypoint
    
    total_distance = 0
    while total_distance < distance:
        # Get next waypoints
        next_wps = current.next(step)
        
        if not next_wps:
            print(f"   ⚠️  Road ended at {total_distance:.1f}m")
            break
        
        # Take first option (straight ahead)
        current = next_wps[0]
        path.append(current)
        total_distance += step
    
    print(f"   ✅ Generated path with {len(path)} waypoints")
    print(f"   Total distance: {total_distance:.1f}m")
    
    # Extract locations
    locations = [wp.transform.location for wp in path]
    
    print(f"\n2. Path waypoints (first 5):")
    for i, loc in enumerate(locations[:5]):
        print(f"   {i+1}. ({loc.x:.1f}, {loc.y:.1f}, {loc.z:.1f})")
    
    return locations


# ========================================
# PART 5: DIFFERENT PATH TYPES
# ========================================

def generate_different_paths(world, waypoint):
    """
    Generate different types of paths.
    """
    
    print("\n" + "="*70)
    print("PART 5: DIFFERENT PATH TYPES")
    print("="*70)
    
    # Type 1: Straight ahead
    print("\n1. STRAIGHT AHEAD:")
    straight_path = []
    current = waypoint
    for _ in range(10):
        straight_path.append(current.transform.location)
        next_wps = current.next(5.0)
        if next_wps:
            current = next_wps[0]  # Take first (straight)
    print(f"   Generated {len(straight_path)} waypoints going straight")
    
    # Type 2: Lane change left
    print("\n2. LANE CHANGE LEFT:")
    if waypoint.get_left_lane():
        lane_change_path = []
        
        # Go straight for a bit
        current = waypoint
        for i in range(5):
            lane_change_path.append(current.transform.location)
            next_wps = current.next(3.0)
            if next_wps:
                current = next_wps[0]
        
        # Change to left lane
        left_lane = current.get_left_lane()
        if left_lane:
            lane_change_path.append(left_lane.transform.location)
            
            # Continue in left lane
            current = left_lane
            for i in range(5):
                next_wps = current.next(3.0)
                if next_wps:
                    current = next_wps[0]
                    lane_change_path.append(current.transform.location)
            
            print(f"   Generated {len(lane_change_path)} waypoints with lane change")
        else:
            print(f"   ❌ Can't change lanes (no left lane available)")
    else:
        print(f"   ❌ No left lane available")
    
    # Type 3: At intersection
    print("\n3. AT INTERSECTION:")
    if waypoint.is_junction:
        print(f"   Current waypoint IS at intersection")
        
        # Get all possible next waypoints
        next_wps = waypoint.next(10.0)
        print(f"   Possible directions: {len(next_wps)}")
        
        for i, wp in enumerate(next_wps):
            print(f"      Option {i+1}: Road {wp.road_id}, Lane {wp.lane_id}")
    else:
        print(f"   Current waypoint is NOT at intersection")


# ========================================
# PART 6: COMPLETE EXAMPLE
# ========================================

def complete_example(world, vehicle):
    """
    Complete example: Generate multiple road-following trajectories.
    """
    
    print("\n" + "="*70)
    print("PART 6: COMPLETE TRAJECTORY GENERATION")
    print("="*70)
    
    carla_map = world.get_map()
    
    # Get current position
    vehicle_location = vehicle.get_location()
    start_waypoint = carla_map.get_waypoint(vehicle_location)
    
    print(f"\nGenerating 5 different trajectories:")
    
    trajectories = []
    
    # Trajectory 1: Straight ahead, short
    print(f"\n1. Straight ahead (30m):")
    path1 = []
    current = start_waypoint
    for _ in range(15):
        path1.append(current.transform.location)
        next_wps = current.next(2.0)
        if next_wps:
            current = next_wps[0]
    print(f"   ✅ {len(path1)} waypoints")
    trajectories.append(path1)
    
    # Trajectory 2: Straight ahead, long
    print(f"\n2. Straight ahead (60m):")
    path2 = []
    current = start_waypoint
    for _ in range(30):
        path2.append(current.transform.location)
        next_wps = current.next(2.0)
        if next_wps:
            current = next_wps[0]
    print(f"   ✅ {len(path2)} waypoints")
    trajectories.append(path2)
    
    # Trajectory 3: Lane change left (if possible)
    print(f"\n3. Lane change left:")
    if start_waypoint.get_left_lane():
        path3 = []
        current = start_waypoint
        
        # Go straight first
        for _ in range(5):
            path3.append(current.transform.location)
            next_wps = current.next(2.0)
            if next_wps:
                current = next_wps[0]
        
        # Change to left lane
        current = current.get_left_lane()
        
        # Continue in left lane
        for _ in range(10):
            if current:
                path3.append(current.transform.location)
                next_wps = current.next(2.0)
                if next_wps:
                    current = next_wps[0]
        
        print(f"   ✅ {len(path3)} waypoints (with lane change)")
        trajectories.append(path3)
    else:
        print(f"   ❌ No left lane available")
    
    # Trajectory 4: Lane change right (if possible)
    print(f"\n4. Lane change right:")
    if start_waypoint.get_right_lane():
        path4 = []
        current = start_waypoint
        
        for _ in range(5):
            path4.append(current.transform.location)
            next_wps = current.next(2.0)
            if next_wps:
                current = next_wps[0]
        
        current = current.get_right_lane()
        
        for _ in range(10):
            if current:
                path4.append(current.transform.location)
                next_wps = current.next(2.0)
                if next_wps:
                    current = next_wps[0]
        
        print(f"   ✅ {len(path4)} waypoints (with lane change)")
        trajectories.append(path4)
    else:
        print(f"   ❌ No right lane available")
    
    # Trajectory 5: Fast (larger steps)
    print(f"\n5. Fast path (5m steps):")
    path5 = []
    current = start_waypoint
    for _ in range(10):
        path5.append(current.transform.location)
        next_wps = current.next(5.0)  # Bigger steps!
        if next_wps:
            current = next_wps[0]
    print(f"   ✅ {len(path5)} waypoints")
    trajectories.append(path5)
    
    print(f"\n{'='*70}")
    print(f"TOTAL: Generated {len(trajectories)} different trajectories")
    print(f"All follow actual roads (no buildings!)")
    print(f"{'='*70}")
    
    return trajectories


# ========================================
# MAIN DEMO
# ========================================

def main():
    """
    Complete demonstration of CARLA's road network API.
    """
    
    print("="*70)
    print(" UNDERSTANDING CARLA'S ROAD NETWORK")
    print("="*70)
    
    try:
        # Connect
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)
        world = client.get_world()
        
        # Spawn vehicle
        blueprint_library = world.get_blueprint_library()
        vehicle_bp = blueprint_library.find('vehicle.tesla.model3')
        spawn_points = world.get_map().get_spawn_points()
        vehicle = world.spawn_actor(vehicle_bp, spawn_points[0])
        
        # Explore!
        carla_map = explore_carla_map(world)
        waypoint = explore_waypoints(world, vehicle)
        navigate_road_network(world, waypoint)
        path = generate_road_following_path(world, waypoint)
        generate_different_paths(world, waypoint)
        trajectories = complete_example(world, vehicle)
        
        # Cleanup
        vehicle.destroy()
        
        print("\n" + "="*70)
        print(" KEY TAKEAWAYS")
        print("="*70)
        print("""
1. MAP = representation of road network
   - Get with: world.get_map()

2. WAYPOINT = directed point on a lane
   - Get with: map.get_waypoint(location)
   - Contains: position, orientation, lane info, road info

3. NAVIGATION methods:
   - waypoint.next(distance) → waypoints ahead
   - waypoint.previous(distance) → waypoints behind
   - waypoint.get_left_lane() → left lane
   - waypoint.get_right_lane() → right lane

4. TO GENERATE PATHS:
   - Start with current waypoint
   - Use next() repeatedly to follow road
   - All waypoints are ON ROADS (no buildings!)

5. CARLA GUARANTEES:
   - All waypoints are on valid road positions
   - next() only returns valid continuations
   - Lane changes only succeed if lane exists
        """)
        
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()