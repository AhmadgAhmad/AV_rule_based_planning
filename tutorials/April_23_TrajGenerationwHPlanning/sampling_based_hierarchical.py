"""
Sampling-Based Motion Planning with Hierarchical Evaluation
============================================================

Session Goal: 
1. Generate trajectories using RRT-like sampling
2. Evaluate them with hierarchical planner
3. Prepare for CARLA integration

Author: Ahmad Ahmad
Student: Nidhi
"""

import numpy as np
import random
import time
from dataclasses import dataclass
from typing import List, Tuple
import matplotlib.pyplot as plt
from enum import Enum


# ========================================
# PART 1: ENVIRONMENT & OBSTACLES
# ========================================

@dataclass
class Obstacle:
    """Circular obstacle in 2D space."""
    x: float
    y: float
    radius: float


class Environment:
    """2D driving environment with obstacles."""
    
    def __init__(self, width=100, height=100):
        self.width = width
        self.height = height
        self.obstacles = []
        
    def add_obstacle(self, x, y, radius):
        """Add circular obstacle."""
        self.obstacles.append(Obstacle(x, y, radius))
    
    def is_collision_free(self, x, y, safety_margin=1.5):
        """Check if point (x,y) is collision-free."""
        for obs in self.obstacles:
            dist = np.sqrt((x - obs.x)**2 + (y - obs.y)**2)
            if dist < obs.radius + safety_margin:
                return False
        return True
    
    def is_path_collision_free(self, path, safety_margin=1.5):
        """Check if entire path is collision-free."""
        for (x, y) in path:
            if not self.is_collision_free(x, y, safety_margin):
                return False
        return True


# ========================================
# PART 2: TRAJECTORY REPRESENTATION
# ========================================

@dataclass
class Trajectory:
    """
    Trajectory with geometric path and metrics.
    
    Attributes:
        id: Unique identifier
        path: List of (x, y) waypoints
        
        # Hard constraints (P0, P1)
        min_obstacle_distance: Closest approach to obstacles (m)
        max_speed: Maximum speed along trajectory (m/s)
        violates_red_light: Whether trajectory runs red light
        
        # Soft objectives (P2, P3, P4)
        comfort: Smoothness metric (lower is better)
        fuel: Fuel consumption estimate (L)
        time: Time to complete trajectory (s)
    """
    id: int
    path: List[Tuple[float, float]] # the actual geometric trajectory (list of waypoints)
    
    # Hard constraints
    min_obstacle_distance: float
    max_speed: float
    violates_red_light: bool
    
    # Soft objectives
    comfort: float
    fuel: float
    time: float


# ========================================
# PART 3: SAMPLING-BASED GENERATOR
# ========================================

class SamplingBasedGenerator:
    """
    Generate trajectories using RRT-like sampling.
    
    Strategy:
    1. Sample random waypoints in environment
    2. Connect start → sampled points → goal
    3. Compute metrics for each trajectory
    """
    
    def __init__(self, environment: Environment, start: Tuple[float, float], 
                 goal: Tuple[float, float]):
        self.env = environment
        self.start = start
        self.goal = goal
    
    def sample_waypoint(self):
        """Sample random point in environment."""
        x = random.uniform(0, self.env.width)
        y = random.uniform(0, self.env.height)
        return (x, y)
    
    def compute_path_length(self, path):
        """Compute total path length."""
        length = 0
        for i in range(len(path) - 1):
            dx = path[i+1][0] - path[i][0]
            dy = path[i+1][1] - path[i][1]
            length += np.sqrt(dx**2 + dy**2)
        return length
    
    def compute_curvature(self, path):
        """Compute maximum curvature (smoothness metric)."""
        if len(path) < 3:
            return 0.0
        
        max_curvature = 0.0
        for i in range(len(path) - 2):
            p0 = np.array(path[i])
            p1 = np.array(path[i+1])
            p2 = np.array(path[i+2])
            
            # Compute angle change
            v1 = p1 - p0
            v2 = p2 - p1
            
            if np.linalg.norm(v1) > 0 and np.linalg.norm(v2) > 0:
                angle = np.arccos(np.clip(np.dot(v1, v2) / 
                                         (np.linalg.norm(v1) * np.linalg.norm(v2)), 
                                         -1.0, 1.0))
                max_curvature = max(max_curvature, angle)
        
        return max_curvature
    
    def compute_min_obstacle_distance(self, path):
        """Compute minimum distance to any obstacle."""
        min_dist = float('inf')
        
        for (x, y) in path:
            for obs in self.env.obstacles:
                dist = np.sqrt((x - obs.x)**2 + (y - obs.y)**2) - obs.radius
                min_dist = min(min_dist, dist)
        
        return max(0, min_dist)
    
    def generate_straight_path(self, n_waypoints=10):
        """Generate straight-line path from start to goal."""
        path = []
        for i in range(n_waypoints):
            t = i / (n_waypoints - 1)
            x = self.start[0] * (1 - t) + self.goal[0] * t
            y = self.start[1] * (1 - t) + self.goal[1] * t
            path.append((x, y))
        return path
    
    def generate_curved_path(self, waypoint, n_waypoints=10):
        """Generate path: start → waypoint → goal."""
        path = []
        
        # Start to waypoint
        for i in range(n_waypoints // 2):
            t = i / (n_waypoints // 2)
            x = self.start[0] * (1 - t) + waypoint[0] * t
            y = self.start[1] * (1 - t) + waypoint[1] * t
            path.append((x, y))
        
        # Waypoint to goal
        for i in range(n_waypoints // 2):
            t = i / (n_waypoints // 2)
            x = waypoint[0] * (1 - t) + self.goal[0] * t
            y = waypoint[1] * (1 - t) + self.goal[1] * t
            path.append((x, y))
        
        return path
    
    def generate_trajectories(self, n=200, seed=42):
        """
        Generate n candidate trajectories using sampling.
        
        Strategy:
        - 20% straight paths
        - 80% curved paths (via sampled waypoints)
        """
        random.seed(seed)
        np.random.seed(seed)
        
        trajectories = []
        
        for i in range(n):
            # Generate path
            if random.random() < 0.2:
                # Straight path
                path = self.generate_straight_path(n_waypoints=20)
            else:
                # Curved path via random waypoint
                waypoint = self.sample_waypoint()
                path = self.generate_curved_path(waypoint, n_waypoints=20)
            
            # Compute metrics
            path_length = self.compute_path_length(path)
            curvature = self.compute_curvature(path)
            min_dist = self.compute_min_obstacle_distance(path)
            
            # Create trajectory
            traj = Trajectory(
                id=i,
                path=path,
                
                # Hard constraints
                min_obstacle_distance=min_dist,
                
                # FIXME: These are just placeholders. In a real implementation, these would be computed based on the path and environment. 
                # For example, max_speed could be based on curvature (tighter curves → lower speed), and violates_red_light could be determined by checking if the path crosses any red light locations.
                # For this demo, we will randomly assign these values to simulate a variety of trajectories.
                #  TODO controbution: Use Temporal Logic to define the rules. 
                
                max_speed=random.uniform(8, 18),  # Sampled for now
                violates_red_light=(random.random() < 0.1),  # 10% violate
                
                # Soft objectives
                comfort=curvature * 10,  # Higher curvature = less comfort
                fuel=path_length * 0.15 + random.uniform(-1, 1),  # Length-based
                time=path_length / 10 + random.uniform(-2, 2)  # Speed-based
            )
            
            trajectories.append(traj)
        
        return trajectories


# ========================================
# PART 4: HIERARCHICAL EVALUATOR
# ========================================

class Context(Enum):
    """Driving contexts."""
    HIGHWAY = "highway"
    CITY = "city"
    PARKING = "parking"


class HierarchicalPlanner:
    """
    Hierarchical planner for trajectory evaluation.
    
    Same algorithm as before, now with geometric trajectories!
    """
    
    def __init__(self, safety_margin=1.5, speed_limit=15.0):
        self.safety_margin = safety_margin
        self.speed_limit = speed_limit
        self.stats = {
            'initial': 0,
            'after_safety': 0,
            'after_legal': 0
        }
    
    def filter_safety(self, trajectories):
        """P0: Safety filter."""
        safe = []
        for traj in trajectories:
            if traj.min_obstacle_distance >= self.safety_margin:
                safe.append(traj)
        
        print(f"  Safety: {len(trajectories)} → {len(safe)} "
              f"({100*len(safe)/len(trajectories):.1f}%)")
        return safe
    
    def filter_legal(self, trajectories):
        """P1: Legal filter."""
        legal = []
        for traj in trajectories:
            if not traj.violates_red_light and \
               traj.max_speed <= self.speed_limit:
                legal.append(traj)
        
        print(f"  Legal: {len(trajectories)} → {len(legal)} "
              f"({100*len(legal)/len(trajectories):.1f}%)")
        return legal
    
    def detect_context(self, speed=20.0, has_traffic_lights=True):
        """Detect driving context."""
        if speed > 25.0 and not has_traffic_lights:
            return Context.HIGHWAY
        elif speed < 3.0:
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
    
    def plan(self, candidates, speed=20.0, has_traffic_lights=True):
        """Main hierarchical planning."""
        print(f"\n{'='*60}")
        print("HIERARCHICAL PLANNING WITH SAMPLING-BASED GENERATION")
        print(f"{'='*60}")
        
        self.stats['initial'] = len(candidates)
        print(f"\nInitial candidates: {len(candidates)}")
        
        # Stage 1: Hard filtering
        print(f"\nSTAGE 1: HARD FILTERING")
        safe = self.filter_safety(candidates)
        self.stats['after_safety'] = len(safe)
        
        if not safe:
            print("\n⚠️  NO SAFE OPTIONS!")
            return None
        
        legal = self.filter_legal(safe)
        self.stats['after_legal'] = len(legal)
        
        if not legal:
            print("\n⚠️  NO LEGAL OPTIONS!")
            return None
        
        reduction = 100 * (1 - len(legal) / len(candidates))
        print(f"\n  Total reduction: {reduction:.1f}%")
        
        # Stage 2: Context detection
        print(f"\nSTAGE 2: CONTEXT DETECTION")
        context = self.detect_context(speed, has_traffic_lights)
        print(f"  Context: {context.value.upper()}")
        
        # Stage 3: Selection
        print(f"\nSTAGE 3: SELECTION")
        order_fn = self.get_total_order(context)
        best = min(legal, key=order_fn)
        
        print(f"  Winner: Trajectory {best.id}")
        
        return best


# ========================================
# PART 5: VISUALIZATION
# ========================================

def visualize_planning(env, trajectories, best_traj=None):
    """Visualize environment, trajectories, and selected path."""
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 7))
    
    # Left plot: All trajectories
    ax1.set_xlim(0, env.width)
    ax1.set_ylim(0, env.height)
    ax1.set_aspect('equal')
    ax1.set_title('All Candidate Trajectories', fontsize=14, fontweight='bold')
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.grid(True, alpha=0.3)
    
    # Draw obstacles
    for obs in env.obstacles:
        circle = plt.Circle((obs.x, obs.y), obs.radius, 
                           color='red', alpha=0.3, label='Obstacle')
        ax1.add_patch(circle)
    
    # Draw all trajectories
    for traj in trajectories[:50]:  # Show first 50 to avoid clutter
        path = np.array(traj.path)
        ax1.plot(path[:, 0], path[:, 1], 'b-', alpha=0.1, linewidth=0.5)
    
    # Draw start and goal
    start = trajectories[0].path[0]
    goal = trajectories[0].path[-1]
    ax1.plot(start[0], start[1], 'go', markersize=15, label='Start', zorder=10)
    ax1.plot(goal[0], goal[1], 'r*', markersize=20, label='Goal', zorder=10)
    
    # Right plot: Selected trajectory
    ax2.set_xlim(0, env.width)
    ax2.set_ylim(0, env.height)
    ax2.set_aspect('equal')
    ax2.set_title('Selected Trajectory (Hierarchical Planning)', 
                  fontsize=14, fontweight='bold')
    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Y (m)')
    ax2.grid(True, alpha=0.3)
    
    # Draw obstacles
    for obs in env.obstacles:
        circle = plt.Circle((obs.x, obs.y), obs.radius, 
                           color='red', alpha=0.3)
        ax2.add_patch(circle)
    
    # Draw selected trajectory
    if best_traj:
        path = np.array(best_traj.path)
        ax2.plot(path[:, 0], path[:, 1], 'g-', linewidth=3, 
                label=f'Traj {best_traj.id}', zorder=5)
        
        # Add metrics
        metrics_text = (
            f"Time: {best_traj.time:.1f}s\n"
            f"Fuel: {best_traj.fuel:.1f}L\n"
            f"Comfort: {best_traj.comfort:.2f}\n"
            f"Min dist: {best_traj.min_obstacle_distance:.1f}m"
        )
        ax2.text(0.02, 0.98, metrics_text, transform=ax2.transAxes,
                fontsize=10, verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    ax2.plot(start[0], start[1], 'go', markersize=15, label='Start', zorder=10)
    ax2.plot(goal[0], goal[1], 'r*', markersize=20, label='Goal', zorder=10)
    ax2.legend()
    
    plt.tight_layout()
    plt.savefig('sampling_based_planning.png', dpi=150, bbox_inches='tight')
    print(f"\n✅ Visualization saved: sampling_based_planning.png")


# ========================================
# PART 6: MAIN DEMO
# ========================================

def main():
    """Complete demo: sampling → evaluation → visualization."""
    
    print("="*70)
    print(" SAMPLING-BASED PLANNING + HIERARCHICAL EVALUATION")
    print("="*70)
    
    # Create environment
    print("\n1. Creating environment...")
    env = Environment(width=100, height=100)
    
    # Add obstacles
    env.add_obstacle(30, 30, 5)
    env.add_obstacle(50, 50, 7)
    env.add_obstacle(70, 30, 6)
    env.add_obstacle(40, 70, 5)
    
    print(f"   Environment: {env.width}×{env.height}m")
    print(f"   Obstacles: {len(env.obstacles)}")
    
    # Create generator
    print("\n2. Creating sampling-based generator...")
    start = (10, 10)
    goal = (90, 90)
    generator = SamplingBasedGenerator(env, start, goal)
    
    print(f"   Start: {start}")
    print(f"   Goal: {goal}")
    
    # Generate trajectories
    print("\n3. Generating trajectories...")
    start_time = time.time()
    trajectories = generator.generate_trajectories(n=200, seed=42)
    gen_time = time.time() - start_time
    
    print(f"   Generated: {len(trajectories)} trajectories")
    print(f"   Time: {gen_time*1000:.1f}ms")
    
    # Evaluate with hierarchical planner
    print("\n4. Evaluating with hierarchical planner...")
    planner = HierarchicalPlanner(safety_margin=1.5, speed_limit=15.0)
    
    best = planner.plan(
        trajectories,
        speed=20.0,  # City driving
        has_traffic_lights=True
    )
    
    # Show results
    if best:
        print(f"\n{'='*60}")
        print("SELECTED TRAJECTORY")
        print(f"{'='*60}")
        print(f"ID:           {best.id}")
        print(f"Path length:  {generator.compute_path_length(best.path):.1f}m")
        print(f"Time:         {best.time:.1f}s")
        print(f"Fuel:         {best.fuel:.1f}L")
        print(f"Comfort:      {best.comfort:.2f}")
        print(f"Min distance: {best.min_obstacle_distance:.1f}m")
    
    # Visualize
    print("\n5. Creating visualization...")
    visualize_planning(env, trajectories, best)
    
    # Summary
    print(f"\n{'='*60}")
    print("SESSION COMPLETE! 🎉")
    print(f"{'='*60}")
    print("\nWhat we did:")
    print("  ✅ Created 2D environment with obstacles")
    print("  ✅ Generated 200 trajectories using sampling")
    print("  ✅ Evaluated with hierarchical planner")
    print("  ✅ Selected best trajectory for context")
    print("  ✅ Visualized results")
    
    print("\nKey results:")
    reduction = 100 * (1 - planner.stats['after_legal'] / planner.stats['initial'])
    print(f"  • Filtered {reduction:.1f}% of candidates")
    print(f"  • Generation time: {gen_time*1000:.1f}ms")
    print(f"  • Selected trajectory avoids all obstacles")
    
    print("\nNext step: Port this to CARLA! 🚗")


if __name__ == "__main__":
    main()
