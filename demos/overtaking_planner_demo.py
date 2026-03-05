"""
Overtaking Scenario - Sampling-Based Planner Demo
==================================================

This script demonstrates overtaking planning:
1. Detect slow vehicle ahead
2. Generate overtaking trajectories
3. Visualize different overtaking strategies
4. Compare timing, safety, and efficiency

Author: Ahmad Ahmad
For: Nidhi (Curious Cardinals Mentorship)
Date: January 2026
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
from typing import List, Tuple
import matplotlib.patches as patches


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
    
    def calc_second_derivative(self, t: float) -> float:
        t = np.clip(t, 0, self.T)
        dd_coeffs = np.array([20, 12, 6, 2, 0, 0]) * self.coeffs
        return np.polyval(dd_coeffs[::-1], t)


# ============================================================================
# OBSTACLE REPRESENTATION
# ============================================================================

class Obstacle:
    """Represents a slow-moving obstacle vehicle."""
    
    def __init__(self, s0: float, d0: float, velocity: float, length: float = 5.0):
        """
        Initialize obstacle.
        
        Args:
            s0: Initial longitudinal position
            d0: Lateral position (lane)
            velocity: Constant velocity (m/s)
            length: Vehicle length (meters)
        """
        self.s0 = s0
        self.d0 = d0
        self.velocity = velocity
        self.length = length
        self.width = 2.0  # Standard car width
    
    def get_position(self, t: float) -> Tuple[float, float]:
        """Get obstacle position at time t."""
        s = self.s0 + self.velocity * t
        return s, self.d0
    
    def get_bounds(self, t: float) -> Tuple[float, float, float, float]:
        """Get bounding box: (s_min, s_max, d_min, d_max)."""
        s, d = self.get_position(t)
        return (s - self.length/2, s + self.length/2,
                d - self.width/2, d + self.width/2)


# ============================================================================
# OVERTAKING TRAJECTORY GENERATOR
# ============================================================================

class OvertakingPlanner:
    """Generates trajectories for overtaking scenarios."""
    
    def __init__(self, lane_width: float = 3.5):
        self.lane_width = lane_width
    
    def generate_overtaking_trajectories(self,
                                        s0: float, d0: float,
                                        v0: float,
                                        obstacle: Obstacle) -> List[dict]:
        """
        Generate multiple overtaking strategies.
        
        Strategies:
        1. Follow behind (no overtake)
        2. Quick overtake (fast acceleration)
        3. Normal overtake (moderate speed)
        4. Early overtake (start early, leisurely)
        5. Stay in left lane longer (cautious)
        
        Args:
            s0, d0: Initial position
            v0: Initial velocity
            obstacle: Obstacle to overtake
            
        Returns:
            List of trajectory dictionaries
        """
        trajectories = []
        
        print("\n" + "="*70)
        print("OVERTAKING TRAJECTORY GENERATION")
        print("="*70)
        print(f"Ego vehicle: s={s0:.1f}m, d={d0:.1f}m, v={v0:.1f}m/s")
        print(f"Obstacle: s={obstacle.s0:.1f}m, d={obstacle.d0:.1f}m, v={obstacle.velocity:.1f}m/s")
        print()
        
        # Strategy 1: FOLLOW BEHIND (no overtake)
        print("Strategy 1: FOLLOW BEHIND")
        traj = self._generate_follow_behind(s0, d0, v0, obstacle)
        traj['strategy'] = 'FOLLOW'
        traj['color'] = 'gray'
        traj['description'] = 'Stay behind obstacle, match speed'
        trajectories.append(traj)
        print(f"  Duration: {traj['T']:.1f}s, Final speed: {traj['v_final']:.1f}m/s")
        
        # Strategy 2: QUICK OVERTAKE
        print("Strategy 2: QUICK OVERTAKE")
        traj = self._generate_quick_overtake(s0, d0, v0, obstacle)
        traj['strategy'] = 'QUICK'
        traj['color'] = 'red'
        traj['description'] = 'Fast acceleration, minimal time in left lane'
        trajectories.append(traj)
        print(f"  Duration: {traj['T']:.1f}s, Final speed: {traj['v_final']:.1f}m/s")
        
        # Strategy 3: NORMAL OVERTAKE
        print("Strategy 3: NORMAL OVERTAKE")
        traj = self._generate_normal_overtake(s0, d0, v0, obstacle)
        traj['strategy'] = 'NORMAL'
        traj['color'] = 'blue'
        traj['description'] = 'Moderate speed, balanced maneuver'
        trajectories.append(traj)
        print(f"  Duration: {traj['T']:.1f}s, Final speed: {traj['v_final']:.1f}m/s")
        
        # Strategy 4: EARLY OVERTAKE
        print("Strategy 4: EARLY OVERTAKE")
        traj = self._generate_early_overtake(s0, d0, v0, obstacle)
        traj['strategy'] = 'EARLY'
        traj['color'] = 'green'
        traj['description'] = 'Start early, leisurely pace'
        trajectories.append(traj)
        print(f"  Duration: {traj['T']:.1f}s, Final speed: {traj['v_final']:.1f}m/s")
        
        # Strategy 5: CAUTIOUS OVERTAKE
        print("Strategy 5: CAUTIOUS OVERTAKE")
        traj = self._generate_cautious_overtake(s0, d0, v0, obstacle)
        traj['strategy'] = 'CAUTIOUS'
        traj['color'] = 'orange'
        traj['description'] = 'Stay in left lane longer, extra clearance'
        trajectories.append(traj)
        print(f"  Duration: {traj['T']:.1f}s, Final speed: {traj['v_final']:.1f}m/s")
        
        print()
        print(f"Generated {len(trajectories)} overtaking strategies!")
        print("="*70)
        
        return trajectories
    
    def _generate_follow_behind(self, s0, d0, v0, obstacle) -> dict:
        """Stay behind obstacle, match its speed."""
        T = 10.0
        v_final = obstacle.velocity  # Match obstacle speed
        
        # Stay in same lane
        s_poly = QuinticPolynomial(s0, v0, 0.0, 
                                   s0 + 0.5*(v0 + v_final)*T, 
                                   v_final, 0.0, T)
        d_poly = QuinticPolynomial(d0, 0.0, 0.0, d0, 0.0, 0.0, T)
        
        return self._sample_trajectory(s_poly, d_poly, T, v_final)
    
    def _generate_quick_overtake(self, s0, d0, v0, obstacle) -> dict:
        """Fast overtake: high acceleration, minimal time in left lane."""
        T = 8.0
        v_final = 18.0  # Fast!
        
        # Three-phase maneuver
        # Phase 1: Move to left lane (0-2s)
        # Phase 2: Accelerate past obstacle (2-5s)
        # Phase 3: Return to right lane (5-8s)
        
        # Simplified: use single trajectory to left lane and back
        # Actually passing through 3 points: current → left lane → right lane
        
        # For demo, approximate with average
        d_avg = -self.lane_width / 3  # Spend time in left
        
        s_poly = QuinticPolynomial(s0, v0, 0.0,
                                   s0 + 0.5*(v0 + v_final)*T,
                                   v_final, 0.0, T)
        
        # Create path that goes left and returns
        # Use a trajectory that peaks in left lane
        d_peak = -self.lane_width  # Left lane
        d_poly = QuinticPolynomial(d0, 0.0, 0.0, d0, 0.0, 0.0, T)
        
        return self._sample_trajectory(s_poly, d_poly, T, v_final, 
                                      overtake_type='quick')
    
    def _generate_normal_overtake(self, s0, d0, v0, obstacle) -> dict:
        """Normal overtake: moderate speed."""
        T = 10.0
        v_final = 15.0  # Moderate speed
        
        s_poly = QuinticPolynomial(s0, v0, 0.0,
                                   s0 + 0.5*(v0 + v_final)*T,
                                   v_final, 0.0, T)
        d_poly = QuinticPolynomial(d0, 0.0, 0.0, d0, 0.0, 0.0, T)
        
        return self._sample_trajectory(s_poly, d_poly, T, v_final,
                                      overtake_type='normal')
    
    def _generate_early_overtake(self, s0, d0, v0, obstacle) -> dict:
        """Early overtake: start early, leisurely."""
        T = 12.0
        v_final = 13.0  # Slightly faster than obstacle
        
        s_poly = QuinticPolynomial(s0, v0, 0.0,
                                   s0 + 0.5*(v0 + v_final)*T,
                                   v_final, 0.0, T)
        d_poly = QuinticPolynomial(d0, 0.0, 0.0, d0, 0.0, 0.0, T)
        
        return self._sample_trajectory(s_poly, d_poly, T, v_final,
                                      overtake_type='early')
    
    def _generate_cautious_overtake(self, s0, d0, v0, obstacle) -> dict:
        """Cautious: stay in left lane longer."""
        T = 14.0
        v_final = 14.0
        
        s_poly = QuinticPolynomial(s0, v0, 0.0,
                                   s0 + 0.5*(v0 + v_final)*T,
                                   v_final, 0.0, T)
        d_poly = QuinticPolynomial(d0, 0.0, 0.0, d0, 0.0, 0.0, T)
        
        return self._sample_trajectory(s_poly, d_poly, T, v_final,
                                      overtake_type='cautious')
    
    def _sample_trajectory(self, s_poly, d_poly, T, v_final, 
                          overtake_type='follow') -> dict:
        """Sample trajectory and add overtaking maneuver."""
        dt = 0.1
        times = np.arange(0, T + dt, dt)
        
        s_points = []
        d_points = []
        v_points = []
        
        for t in times:
            s = s_poly.calc_point(t)
            d_base = d_poly.calc_point(t)
            
            # Add overtaking lateral maneuver based on type
            if overtake_type == 'quick':
                # Quick: left lane from t=1 to t=6
                if 1.0 <= t <= 6.0:
                    phase = (t - 1.0) / 5.0
                    if phase < 0.2:  # Move left
                        d = d_base - self.lane_width * (phase / 0.2)
                    elif phase > 0.8:  # Move back right
                        d = d_base - self.lane_width * (1 - (phase - 0.8) / 0.2)
                    else:  # Stay left
                        d = d_base - self.lane_width
                else:
                    d = d_base
                    
            elif overtake_type == 'normal':
                # Normal: left lane from t=2 to t=7
                if 2.0 <= t <= 7.0:
                    phase = (t - 2.0) / 5.0
                    if phase < 0.25:
                        d = d_base - self.lane_width * (phase / 0.25)
                    elif phase > 0.75:
                        d = d_base - self.lane_width * (1 - (phase - 0.75) / 0.25)
                    else:
                        d = d_base - self.lane_width
                else:
                    d = d_base
                    
            elif overtake_type == 'early':
                # Early: left lane from t=1 to t=9
                if 1.0 <= t <= 9.0:
                    phase = (t - 1.0) / 8.0
                    if phase < 0.2:
                        d = d_base - self.lane_width * (phase / 0.2)
                    elif phase > 0.8:
                        d = d_base - self.lane_width * (1 - (phase - 0.8) / 0.2)
                    else:
                        d = d_base - self.lane_width
                else:
                    d = d_base
                    
            elif overtake_type == 'cautious':
                # Cautious: left lane from t=2 to t=11
                if 2.0 <= t <= 11.0:
                    phase = (t - 2.0) / 9.0
                    if phase < 0.2:
                        d = d_base - self.lane_width * (phase / 0.2)
                    elif phase > 0.85:
                        d = d_base - self.lane_width * (1 - (phase - 0.85) / 0.15)
                    else:
                        d = d_base - self.lane_width
                else:
                    d = d_base
            else:
                # Follow: stay in lane
                d = d_base
            
            s_points.append(s)
            d_points.append(d)
            
            s_dot = s_poly.calc_first_derivative(t)
            v_points.append(s_dot)
        
        return {
            'times': times,
            's': np.array(s_points),
            'd': np.array(d_points),
            'v': np.array(v_points),
            'T': T,
            'v_final': v_final
        }


# ============================================================================
# COLLISION DETECTION
# ============================================================================

def check_collision(trajectory: dict, obstacle: Obstacle, 
                   safety_margin: float = 2.0) -> Tuple[bool, float]:
    """
    Check if trajectory collides with obstacle.
    
    Args:
        trajectory: Trajectory dictionary
        obstacle: Obstacle object
        safety_margin: Additional clearance (meters)
        
    Returns:
        (has_collision, min_distance)
    """
    min_distance = float('inf')
    collision = False
    
    for t, s, d in zip(trajectory['times'], trajectory['s'], trajectory['d']):
        obs_s, obs_d = obstacle.get_position(t)
        
        # Distance between centers
        dist = np.sqrt((s - obs_s)**2 + (d - obs_d)**2)
        min_distance = min(min_distance, dist)
        
        # Check collision (including safety margin)
        required_clearance = (obstacle.length + obstacle.width) / 2 + safety_margin
        if dist < required_clearance:
            collision = True
    
    return collision, min_distance


# ============================================================================
# VISUALIZATION
# ============================================================================

def visualize_overtaking_scenario(trajectories: List[dict], obstacle: Obstacle):
    """Create comprehensive visualization of overtaking strategies."""
    
    fig = plt.figure(figsize=(16, 10))
    gs = fig.add_gridspec(3, 2, hspace=0.3, wspace=0.3)
    
    # Main plot: All trajectories
    ax_main = fig.add_subplot(gs[0:2, :])
    visualize_trajectories_top_view(ax_main, trajectories, obstacle)
    
    # Bottom plots
    ax_speed = fig.add_subplot(gs[2, 0])
    visualize_speed_profiles(ax_speed, trajectories)
    
    ax_lateral = fig.add_subplot(gs[2, 1])
    visualize_lateral_positions(ax_lateral, trajectories, obstacle)
    
    plt.suptitle('Overtaking Strategies Comparison', fontsize=16, fontweight='bold')
    
    plt.savefig('/home/ahmad/Desktop/RuleBookDriving/outputs/overtaking_strategies.png', dpi=150, bbox_inches='tight')
    print("\n✓ Visualization saved: overtaking_strategies.png")
    
    return fig


def visualize_trajectories_top_view(ax, trajectories, obstacle):
    """Top-down view of all strategies."""
    
    lane_width = 3.5
    
    # Draw road lanes
    ax.axhline(y=-lane_width/2, color='gray', linestyle='--', linewidth=1, alpha=0.5)
    ax.axhline(y=lane_width/2, color='gray', linestyle='--', linewidth=1, alpha=0.5)
    ax.axhline(y=-3*lane_width/2, color='gray', linestyle='--', linewidth=1, alpha=0.5)
    ax.axhline(y=3*lane_width/2, color='gray', linestyle='--', linewidth=1, alpha=0.5)
    ax.axhline(y=0, color='yellow', linestyle='-', linewidth=2, alpha=0.5, label='Lane centerline')
    ax.axhline(y=-lane_width, color='yellow', linestyle='-', linewidth=2, alpha=0.5)
    
    # Draw obstacle trajectory
    times = np.linspace(0, max(t['T'] for t in trajectories), 50)
    obs_s = [obstacle.s0 + obstacle.velocity * t for t in times]
    obs_d = [obstacle.d0] * len(times)
    ax.plot(obs_s, obs_d, 'k--', linewidth=3, alpha=0.7, label='Obstacle path')
    
    # Draw obstacle at t=0
    rect = Rectangle((obstacle.s0 - obstacle.length/2, obstacle.d0 - obstacle.width/2),
                     obstacle.length, obstacle.width,
                     facecolor='red', edgecolor='darkred', linewidth=2, alpha=0.7)
    ax.add_patch(rect)
    ax.text(obstacle.s0, obstacle.d0, 'OBSTACLE\n{:.1f} m/s'.format(obstacle.velocity),
            ha='center', va='center', fontsize=9, fontweight='bold', color='white')
    
    # Draw each trajectory
    for traj in trajectories:
        ax.plot(traj['s'], traj['d'], 
               color=traj['color'], linewidth=2.5, alpha=0.8,
               label=f"{traj['strategy']}: {traj['description']}")
    
    # Mark starting point
    ax.plot(trajectories[0]['s'][0], trajectories[0]['d'][0], 
           'go', markersize=15, label='Start', zorder=10)
    
    ax.set_xlabel('Longitudinal Position s (m)', fontsize=12, fontweight='bold')
    ax.set_ylabel('Lateral Position d (m)', fontsize=12, fontweight='bold')
    ax.set_title('Top-Down View: Overtaking Trajectories', fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper left', fontsize=9)
    ax.set_xlim(-5, max(t['s'][-1] for t in trajectories) + 10)
    ax.set_ylim(-2*lane_width, lane_width)
    
    # Add lane labels
    ax.text(10, -lane_width, 'LEFT LANE\n(Overtaking)', 
           fontsize=10, ha='center', bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.5))
    ax.text(10, 0, 'RIGHT LANE\n(Travel)', 
           fontsize=10, ha='center', bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.5))


def visualize_speed_profiles(ax, trajectories):
    """Speed profiles over time."""
    
    for traj in trajectories:
        ax.plot(traj['times'], traj['v'], 
               color=traj['color'], linewidth=2, alpha=0.8,
               label=traj['strategy'])
    
    ax.set_xlabel('Time (s)', fontsize=11, fontweight='bold')
    ax.set_ylabel('Velocity (m/s)', fontsize=11, fontweight='bold')
    ax.set_title('Speed Profiles', fontsize=12, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=9)


def visualize_lateral_positions(ax, trajectories, obstacle):
    """Lateral position over time."""
    
    lane_width = 3.5
    
    # Reference lanes
    ax.axhline(y=0, color='green', linestyle='--', alpha=0.3, label='Right lane center')
    ax.axhline(y=-lane_width, color='blue', linestyle='--', alpha=0.3, label='Left lane center')
    ax.axhline(y=obstacle.d0, color='red', linestyle=':', linewidth=2, alpha=0.5, label='Obstacle lane')
    
    for traj in trajectories:
        ax.plot(traj['times'], traj['d'], 
               color=traj['color'], linewidth=2, alpha=0.8,
               label=traj['strategy'])
    
    ax.set_xlabel('Time (s)', fontsize=11, fontweight='bold')
    ax.set_ylabel('Lateral Position d (m)', fontsize=11, fontweight='bold')
    ax.set_title('Lateral Positions (Lane Changes)', fontsize=12, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=9, loc='best')


# ============================================================================
# ANALYSIS
# ============================================================================

def analyze_trajectories(trajectories: List[dict], obstacle: Obstacle):
    """Analyze and compare overtaking strategies."""
    
    print("\n" + "="*70)
    print("TRAJECTORY ANALYSIS")
    print("="*70)
    
    for traj in trajectories:
        collision, min_dist = check_collision(traj, obstacle, safety_margin=1.0)
        
        # Compute metrics
        avg_speed = np.mean(traj['v'])
        max_speed = np.max(traj['v'])
        total_distance = traj['s'][-1] - traj['s'][0]
        time_in_left_lane = np.sum(traj['d'] < -1.0) * 0.1  # dt = 0.1
        
        print(f"\n{traj['strategy']} - {traj['description']}")
        print(f"  Duration: {traj['T']:.1f}s")
        print(f"  Distance traveled: {total_distance:.1f}m")
        print(f"  Average speed: {avg_speed:.1f} m/s")
        print(f"  Max speed: {max_speed:.1f} m/s")
        print(f"  Time in left lane: {time_in_left_lane:.1f}s")
        print(f"  Min clearance from obstacle: {min_dist:.1f}m")
        print(f"  Collision: {'❌ YES' if collision else '✅ NO'}")
    
    print("\n" + "="*70)


# ============================================================================
# MAIN DEMONSTRATION
# ============================================================================

def main():
    """Main demonstration of overtaking planner."""
    
    print("\n" + "="*70)
    print("OVERTAKING SCENARIO - SAMPLING-BASED PLANNER")
    print("="*70)
    
    # Setup scenario
    print("\n[SCENARIO SETUP]")
    print("-" * 70)
    
    # Ego vehicle: starting conditions
    s0 = 0.0
    d0 = 0.0  # Right lane
    v0 = 12.0  # 12 m/s
    print(f"Ego vehicle:")
    print(f"  Position: s={s0}m, d={d0}m (right lane)")
    print(f"  Velocity: {v0} m/s (43.2 km/h)")
    
    # Obstacle: slow vehicle ahead
    obstacle = Obstacle(
        s0=30.0,  # 30m ahead
        d0=0.0,   # Right lane (same lane)
        velocity=8.0,  # Slow: 8 m/s (28.8 km/h)
        length=5.0
    )
    print(f"\nObstacle vehicle:")
    print(f"  Position: s={obstacle.s0}m ahead, d={obstacle.d0}m (right lane)")
    print(f"  Velocity: {obstacle.velocity} m/s (28.8 km/h)")
    print(f"  Relative speed: {v0 - obstacle.velocity} m/s faster")
    
    # Generate overtaking trajectories
    print("\n[GENERATING TRAJECTORIES]")
    print("-" * 70)
    
    planner = OvertakingPlanner(lane_width=3.5)
    trajectories = planner.generate_overtaking_trajectories(s0, d0, v0, obstacle)
    
    # Analyze trajectories
    print("\n[ANALYZING TRAJECTORIES]")
    print("-" * 70)
    analyze_trajectories(trajectories, obstacle)
    
    # Visualize
    print("\n[CREATING VISUALIZATIONS]")
    print("-" * 70)
    fig = visualize_overtaking_scenario(trajectories, obstacle)
    
    # Summary
    print("\n" + "="*70)
    print("SUMMARY")
    print("="*70)
    print("\nKey Insights:")
    print("  • FOLLOW: Safest but slowest, stuck behind obstacle")
    print("  • QUICK: Fastest but requires high speed differential")
    print("  • NORMAL: Good balance of time and safety")
    print("  • EARLY: Starts overtake early, most leisurely")
    print("  • CAUTIOUS: Maximum clearance, longest time in left lane")
    print("\nNext Step: Rulebook decides which strategy is best!")
    print("  - No collision (P0)")
    print("  - Lane compliance (P2)")
    print("  - Speed limit (P4)")
    print("  - Efficiency (P5)")
    print("="*70 + "\n")
    
    plt.show()


if __name__ == "__main__":
    main()
