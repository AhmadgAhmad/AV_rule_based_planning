# Today's Session: Sampling-Based Planning + Hierarchical Evaluation
**From Python to CARLA**

---

## 🎯 Session Overview

**Goal:** Build a complete motion planning system that:
1. Generates trajectories using sampling (Python)
2. Evaluates them with hierarchical planner (Python)
3. Ports to CARLA simulator (CARLA)

**Time:** 90 minutes total
- Part 1: Python implementation (45 min)
- Part 2: CARLA integration (45 min)

**What makes this different from original plan:**
- More concrete (actual geometric paths, not abstract)
- Connects to previous sessions (RRT/sampling)
- Bridges to CARLA smoothly
- Visual feedback (matplotlib + CARLA)

---

## 📚 Part 1: Python Implementation (45 min)

### **Step 1: Create Environment (5 min)**

**Code together:**
```python
class Environment:
    def __init__(self, width=100, height=100):
        self.width = width
        self.height = height
        self.obstacles = []
    
    def add_obstacle(self, x, y, radius):
        self.obstacles.append(Obstacle(x, y, radius))
    
    def is_collision_free(self, x, y, safety_margin=1.5):
        for obs in self.obstacles:
            dist = sqrt((x - obs.x)^2 + (y - obs.y)^2)
            if dist < obs.radius + safety_margin:
                return False
        return True

# Create environment
env = Environment(100, 100)
env.add_obstacle(30, 30, 5)
env.add_obstacle(50, 50, 7)
env.add_obstacle(70, 30, 6)
```

**Teaching points:**
- "This is like a top-down view of a parking lot"
- "Obstacles are other cars, walls, etc."
- "Safety margin = how close we're willing to get"

**Test:** Check if point (40, 40) is collision-free

---

### **Step 2: Trajectory Representation (5 min)**

**Show the dataclass:**
```python
@dataclass
class Trajectory:
    id: int
    path: List[Tuple[float, float]]  # [(x1,y1), (x2,y2), ...]
    
    # Same metrics as before!
    min_obstacle_distance: float
    max_speed: float
    violates_red_light: bool
    comfort: float
    fuel: float
    time: float
```

**Key insight:**
"This is the bridge! Path = geometric, Metrics = evaluation"

---

### **Step 3: Sampling-Based Generator (15 min)**

**Build incrementally:**

```python
class SamplingBasedGenerator:
    def __init__(self, env, start, goal):
        self.env = env
        self.start = start
        self.goal = goal
    
    def sample_waypoint(self):
        """Sample random point in environment."""
        x = random.uniform(0, self.env.width)
        y = random.uniform(0, self.env.height)
        return (x, y)
    
    def generate_straight_path(self, n_waypoints=10):
        """Straight line: start → goal."""
        path = []
        for i in range(n_waypoints):
            t = i / (n_waypoints - 1)
            x = self.start[0] * (1-t) + self.goal[0] * t
            y = self.start[1] * (1-t) + self.goal[1] * t
            path.append((x, y))
        return path
    
    def generate_curved_path(self, waypoint):
        """Curved: start → waypoint → goal."""
        path = []
        # Start to waypoint
        for i in range(5):
            t = i / 5
            x = self.start[0] * (1-t) + waypoint[0] * t
            y = self.start[1] * (1-t) + waypoint[1] * t
            path.append((x, y))
        # Waypoint to goal
        for i in range(5):
            t = i / 5
            x = waypoint[0] * (1-t) + self.goal[0] * t
            y = waypoint[1] * (1-t) + self.goal[1] * t
            path.append((x, y))
        return path
```

**Teaching points:**
- "Sample random waypoints - like RRT!"
- "Some paths straight, some curved"
- "Generate many candidates, pick best later"

**Test:** Generate 5 paths, plot them

---

### **Step 4: Compute Metrics (10 min)**

```python
def compute_path_length(self, path):
    """Sum of distances between waypoints."""
    length = 0
    for i in range(len(path) - 1):
        dx = path[i+1][0] - path[i][0]
        dy = path[i+1][1] - path[i][1]
        length += sqrt(dx^2 + dy^2)
    return length

def compute_curvature(self, path):
    """How sharp are the turns?"""
    max_angle = 0
    for i in range(len(path) - 2):
        # Compute angle between segments
        v1 = path[i+1] - path[i]
        v2 = path[i+2] - path[i+1]
        angle = angle_between(v1, v2)
        max_angle = max(max_angle, angle)
    return max_angle

def compute_min_obstacle_distance(self, path):
    """Closest we get to any obstacle."""
    min_dist = infinity
    for (x, y) in path:
        for obs in self.env.obstacles:
            dist = distance((x,y), obs) - obs.radius
            min_dist = min(min_dist, dist)
    return min_dist
```

**Teaching points:**
- "Length → fuel + time"
- "Curvature → comfort (sharp turns = uncomfortable)"
- "Min distance → safety"

---

### **Step 5: Hierarchical Evaluation (5 min)**

```python
# Use the SAME planner from before!
planner = HierarchicalPlanner(safety_margin=1.5)

# Generate candidates
generator = SamplingBasedGenerator(env, start, goal)
trajectories = generator.generate_trajectories(n=200)

# Evaluate
best = planner.plan(trajectories)

print(f"Selected: Trajectory {best.id}")
print(f"Path: {len(best.path)} waypoints")
```

**Key insight:**
"Same hierarchical planner! Just different input (geometric paths)"

---

### **Step 6: Visualization (5 min)**

```python
def visualize_planning(env, trajectories, best):
    plt.figure(figsize=(12, 6))
    
    # Left: All trajectories
    plt.subplot(1, 2, 1)
    for traj in trajectories[:50]:
        path = np.array(traj.path)
        plt.plot(path[:,0], path[:,1], 'b-', alpha=0.1)
    # Draw obstacles
    for obs in env.obstacles:
        circle = plt.Circle((obs.x, obs.y), obs.radius, 
                           color='red', alpha=0.3)
        plt.gca().add_patch(circle)
    plt.title("All Candidates")
    
    # Right: Selected
    plt.subplot(1, 2, 2)
    path = np.array(best.path)
    plt.plot(path[:,0], path[:,1], 'g-', linewidth=3)
    # Draw obstacles
    for obs in env.obstacles:
        circle = plt.Circle((obs.x, obs.y), obs.radius, 
                           color='red', alpha=0.3)
        plt.gca().add_patch(circle)
    plt.title(f"Selected: Traj {best.id}")
    
    plt.show()
```

**Result:** Beautiful visualization showing all paths + selected winner!

---

## 🚗 Part 2: CARLA Integration (45 min)

### **Step 1: CARLA Setup (10 min)**

**Launch CARLA:**
```bash
# Terminal 1: Start CARLA
cd ~/CARLA
./CarlaUE4.sh

# Terminal 2: Run Python script
python3 carla_sampling_hierarchical.py
```

**Connect to CARLA:**
```python
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.get_world()

print("✅ Connected to CARLA!")
```

**Spawn vehicle:**
```python
blueprint = world.get_blueprint_library().find('vehicle.tesla.model3')
spawn_point = world.get_map().get_spawn_points()[0]
vehicle = world.spawn_actor(blueprint, spawn_point)

print(f"✅ Spawned at {spawn_point.location}")
```

---

### **Step 2: CARLA Trajectory Generator (15 min)**

**Key difference: Use CARLA map!**

```python
class CARLASamplingGenerator:
    def __init__(self, world, vehicle, goal):
        self.world = world
        self.vehicle = vehicle
        self.goal = goal
        self.map = world.get_map()
    
    def sample_waypoint_on_road(self):
        """Sample waypoint that's ACTUALLY on a road."""
        spawn_points = self.map.get_spawn_points()
        random_spawn = random.choice(spawn_points)
        return random_spawn.location
    
    def generate_path_to_location(self, target):
        """Generate path using CARLA waypoints."""
        current = self.vehicle.get_location()
        
        # Linear interpolation for now
        path = []
        for i in range(20):
            t = i / 19
            x = current.x * (1-t) + target.x * t
            y = current.y * (1-t) + target.y * t
            z = current.z * (1-t) + target.z * t
            path.append(carla.Location(x, y, z))
        
        return path
```

**Teaching points:**
- "CARLA has a real map with roads!"
- "We sample on roads, not random space"
- "Paths are 3D (x, y, z)"

---

### **Step 3: CARLA Metrics (10 min)**

**Collision checking in CARLA:**
```python
def compute_min_obstacle_distance(self, path):
    """Check distance to other cars in CARLA."""
    
    # Get all vehicles in world
    vehicles = self.world.get_actors().filter('vehicle.*')
    
    # Compute min distance
    min_dist = infinity
    for waypoint in path:
        for other_vehicle in vehicles:
            if other_vehicle.id == self.vehicle.id:
                continue  # Skip ourselves!
            
            other_loc = other_vehicle.get_location()
            dist = waypoint.distance(other_loc)
            min_dist = min(min_dist, dist)
    
    return min_dist
```

**Traffic light checking:**
```python
def check_traffic_light_violation(self, path):
    """Check if path runs red light."""
    
    if self.vehicle.is_at_traffic_light():
        light = self.vehicle.get_traffic_light()
        
        # If red or yellow
        if light.state != carla.TrafficLightState.Green:
            # Check if path continues forward
            if path[0].distance(path[5]) > 3.0:  # Crosses line
                return True
    
    return False
```

---

### **Step 4: Evaluate in CARLA (5 min)**

```python
# Generate trajectories
generator = CARLASamplingGenerator(world, vehicle, goal)
trajectories = generator.generate_trajectories(n=50)

# Evaluate with hierarchical planner
planner = CARLAHierarchicalPlanner(safety_margin=2.0)
best = planner.plan(trajectories, vehicle, world)

print(f"Selected: Trajectory {best.id}")
```

**Same algorithm! Just CARLA data!**

---

### **Step 5: Execute in CARLA (5 min)**

```python
def execute_trajectory(trajectory, vehicle):
    """Convert trajectory to vehicle control."""
    
    target = trajectory.waypoints[0]
    current = vehicle.get_location()
    
    # Compute steering
    direction = target - current
    forward = vehicle.get_transform().get_forward_vector()
    cross = direction.x * forward.y - direction.y * forward.x
    steer = clip(cross * 2.0, -1, 1)
    
    # Compute throttle
    speed_error = 10 - vehicle.get_velocity().length()
    throttle = clip(speed_error * 0.5, 0, 1)
    
    control = carla.VehicleControl(
        throttle=throttle,
        steer=steer
    )
    
    return control

# Execute
for step in range(100):
    control = execute_trajectory(best, vehicle)
    vehicle.apply_control(control)
    time.sleep(0.1)
```

**Watch it drive in CARLA!** 🚗

---

## 🎯 Key Teaching Moments

### **When building sampling generator:**
```
"Remember RRT from Session 2? This is the same idea!
Sample random points, connect them, evaluate quality."
```

### **When computing metrics:**
```
"Path length → fuel + time
Curvature → comfort
Min distance → safety

Same metrics, different representation!"
```

### **When showing hierarchical evaluation:**
```
"Look - same 3 stages!
1. Filter unsafe (82% eliminated!)
2. Detect context
3. Pick best for context

Works in Python AND CARLA!"
```

### **When it runs in CARLA:**
```
"This is what Waymo does! Every 100ms:
1. Sample 200 paths
2. Filter + evaluate
3. Execute winner

You just built it!"
```

---

## 📊 Expected Results

### **Python (Part 1):**
```
Generated: 200 trajectories (92ms)
After safety: 58 (29%)
After legal: 36 (18%)
Winner: Trajectory 55
  Path: 110.3m
  Time: 12.7s
  Min dist: 1.9m
```

### **CARLA (Part 2):**
```
Generated: 50 trajectories (150ms)
After safety: 25 (50%)
After legal: 18 (36%)
Winner: Trajectory 12
  Path: 87.5m
  Executes smoothly in simulator!
```

---

## ✅ Success Criteria

### **By end of session, Nidhi should:**

**Understand:**
- How sampling generates trajectories
- How to compute geometric metrics
- Connection: sampling → evaluation
- Python → CARLA mapping

**Implement:**
- Environment with obstacles
- Sampling-based generator
- Metric computation
- Hierarchical evaluation
- (Bonus) CARLA integration

**See:**
- Visualization of all paths
- Selected path avoiding obstacles
- Car driving in CARLA!

---

## 🚀 Session Flow (90 min)

```
0:00-0:05   Intro & overview
0:05-0:25   Build sampling generator (Python)
0:25-0:35   Add hierarchical evaluation
0:35-0:45   Visualize results
--- BREAK (5 min) ---
0:50-1:00   CARLA setup
1:00-1:15   CARLA generator
1:15-1:25   CARLA evaluation
1:25-1:30   Execute in CARLA
```

---

## 💡 Advantages of This Approach

**Compared to original abstract plan:**

✅ **More concrete:** Actual geometric paths, not abstract numbers
✅ **Visual feedback:** See paths in matplotlib + CARLA
✅ **Connects sessions:** Uses RRT/sampling from Session 2
✅ **Smooth transition:** Python → CARLA is natural
✅ **More engaging:** Watching car drive is exciting!
✅ **Real-world:** This IS how AVs work (Waymo, etc.)

**Still teaches hierarchical planning:**
✅ Same 3 stages
✅ Same partial order exploitation
✅ Same filtering efficiency

---

## 📁 Files Created

1. `sampling_based_hierarchical.py` - Complete Python implementation
2. `carla_sampling_hierarchical.py` - CARLA integration
3. `sampling_based_planning.png` - Visualization output

---

## 🎯 Next Steps

**After this session:**
1. Nidhi understands complete pipeline
2. Has working code in Python + CARLA
3. Ready for advanced topics (perception, prediction)

**Homework options:**
- Improve sampling strategy
- Add more obstacle types
- Implement better CARLA routing
- Add dynamic obstacles

---

**This approach is more concrete, visual, and engaging while still teaching hierarchical planning! 🚗⚡**
