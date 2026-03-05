"""
Sampling-Based Motion Planner - Educational Demo
================================================

This script demonstrates how the sampling planner works:
1. Generate quintic polynomial trajectories
2. Sample different terminal states (lateral position, velocity, time)
3. Visualize the generated trajectories

Author: Ahmad Ahmad
For: Nidhi (Curious Cardinals Mentorship)
Date: January 2026
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from typing import List, Tuple


# ============================================================================
# PART 1: QUINTIC POLYNOMIAL CLASS
# ============================================================================

class QuinticPolynomial:
    """
    Generates a quintic (5th order) polynomial trajectory.
    
    A quintic polynomial has the form:
        p(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
    
    It has 6 coefficients, which we determine from 6 boundary conditions:
        - Initial position, velocity, acceleration
        - Final position, velocity, acceleration
    """
    
    def __init__(self, x0: float, v0: float, a0: float,
                 xf: float, vf: float, af: float, T: float):
        """
        Initialize quintic polynomial with boundary conditions.
        
        Args:
            x0: Initial position
            v0: Initial velocity
            a0: Initial acceleration
            xf: Final position
            vf: Final velocity
            af: Final acceleration
            T: Time duration
        """
        self.x0 = x0
        self.v0 = v0
        self.a0 = a0
        self.xf = xf
        self.vf = vf
        self.af = af
        self.T = T
        
        # Solve for the 6 coefficients
        self.coeffs = self._compute_coefficients()
    
    def _compute_coefficients(self) -> np.ndarray:
        """
        Compute the 6 polynomial coefficients by solving linear system.
        
        The boundary conditions give us 6 equations:
            p(0) = x0,   p'(0) = v0,   p''(0) = a0
            p(T) = xf,   p'(T) = vf,   p''(T) = af
        
        Returns:
            Array of 6 coefficients [a0, a1, a2, a3, a4, a5]
        """
        T = self.T
        
        # Construct the coefficient matrix A
        # Each row corresponds to one boundary condition
        A = np.array([
            [0,     0,     0,      0,       0,      1],      # p(0) = x0
            [T**5,  T**4,  T**3,   T**2,    T,      1],      # p(T) = xf
            [0,     0,     0,      0,       1,      0],      # p'(0) = v0
            [5*T**4, 4*T**3, 3*T**2, 2*T,   1,      0],      # p'(T) = vf
            [0,     0,     0,      2,       0,      0],      # p''(0) = a0
            [20*T**3, 12*T**2, 6*T, 2,      0,      0]       # p''(T) = af
        ])
        
        # Right-hand side vector b
        b = np.array([self.x0, self.xf, self.v0, self.vf, self.a0, self.af])
        
        # Solve the linear system: A * coeffs = b
        coeffs = np.linalg.solve(A, b)
        
        return coeffs
    
    def calc_point(self, t: float) -> float:
        """Calculate position at time t."""
        t = np.clip(t, 0, self.T)
        # Evaluate polynomial: a0 + a1*t + a2*t^2 + ... + a5*t^5
        return np.polyval(self.coeffs[::-1], t)
    
    def calc_first_derivative(self, t: float) -> float:
        """Calculate velocity at time t."""
        t = np.clip(t, 0, self.T)
        # Derivative coefficients: [5*a5, 4*a4, 3*a3, 2*a2, 1*a1, 0*a0]
        d_coeffs = np.array([5, 4, 3, 2, 1, 0]) * self.coeffs
        return np.polyval(d_coeffs[::-1], t)
    
    def calc_second_derivative(self, t: float) -> float:
        """Calculate acceleration at time t."""
        t = np.clip(t, 0, self.T)
        # Second derivative coefficients
        dd_coeffs = np.array([20, 12, 6, 2, 0, 0]) * self.coeffs
        return np.polyval(dd_coeffs[::-1], t)


# ============================================================================
# PART 2: TRAJECTORY CLASS
# ============================================================================

class FrenetTrajectory:
    """
    Represents a trajectory in Frenet frame (s, d coordinates).
    
    We plan longitudinal (s) and lateral (d) motion independently,
    each using a quintic polynomial.
    """
    
    def __init__(self, 
                 s_poly: QuinticPolynomial,
                 d_poly: QuinticPolynomial):
        """
        Initialize trajectory from longitudinal and lateral polynomials.
        
        Args:
            s_poly: Quintic polynomial for longitudinal motion s(t)
            d_poly: Quintic polynomial for lateral motion d(t)
        """
        self.s_poly = s_poly
        self.d_poly = d_poly
        self.T = s_poly.T
        
        # Sample the trajectory at fixed time intervals
        self.dt = 0.1  # Sample every 0.1 seconds
        self.times = np.arange(0, self.T + self.dt, self.dt)
        
        # Compute trajectory points
        self.s = np.array([s_poly.calc_point(t) for t in self.times])
        self.d = np.array([d_poly.calc_point(t) for t in self.times])
        self.s_dot = np.array([s_poly.calc_first_derivative(t) for t in self.times])
        self.d_dot = np.array([d_poly.calc_first_derivative(t) for t in self.times])
        self.s_ddot = np.array([s_poly.calc_second_derivative(t) for t in self.times])
        self.d_ddot = np.array([d_poly.calc_second_derivative(t) for t in self.times])


# ============================================================================
# PART 3: SAMPLING PLANNER
# ============================================================================

class SamplingPlanner:
    """
    Generates multiple candidate trajectories by sampling terminal states.
    
    Sampling dimensions:
        1. Lateral position d_f: which lane to target
        2. Target velocity v_f: how fast to go
        3. Time horizon T: how long the maneuver takes
    """
    
    def __init__(self, 
                 lane_width: float = 3.5,
                 target_velocities: List[float] = [8.0, 12.0, 15.0, 18.0],
                 time_horizons: List[float] = [3.0, 4.0, 5.0]):
        """
        Initialize sampling planner.
        
        Args:
            lane_width: Width of a single lane in meters
            target_velocities: List of velocities to sample (m/s)
            time_horizons: List of time durations to sample (seconds)
        """
        self.lane_width = lane_width
        self.target_velocities = target_velocities
        self.time_horizons = time_horizons
        
        # Define lateral positions to sample (left lane, current, right lane)
        self.lateral_samples = [-lane_width, 0.0, lane_width]
    
    def generate_trajectories(self,
                            s0: float, d0: float,
                            s_dot0: float, d_dot0: float,
                            s_ddot0: float = 0.0, d_ddot0: float = 0.0
                            ) -> List[FrenetTrajectory]:
        """
        Generate all candidate trajectories from current state.
        
        This is the CORE of the sampling planner. We:
        1. Loop through lateral positions (which lane?)
        2. Loop through target velocities (how fast?)
        3. Loop through time horizons (how long?)
        4. For each combination, create a quintic polynomial trajectory
        
        Args:
            s0, d0: Initial longitudinal and lateral position
            s_dot0, d_dot0: Initial velocities
            s_ddot0, d_ddot0: Initial accelerations
            
        Returns:
            List of candidate trajectories
        """
        trajectories = []
        
        print("\n" + "="*60)
        print("GENERATING CANDIDATE TRAJECTORIES")
        print("="*60)
        print(f"Initial state: s={s0:.1f}m, d={d0:.1f}m, v={s_dot0:.1f}m/s")
        print()
        
        trajectory_count = 0
        
        # SAMPLING LOOP
        # --------------
        
        # 1. Sample different lateral positions (lanes)
        self.lateral_samples = np.random.uniform(-self.lane_width, self.lane_width, 20)
        self.target_velocities = np.random.uniform(8.0, 18.0, 10)
        self.time_horizons = np.random.uniform(3.0, 5.0, 6)
        # line = np.linspace(0,99999, 100000)
        # plt.scatter(line, self.lateral_samples)
        # a = 1 
        
        for d_f in self.lateral_samples:
            
            # 2. Sample different target velocities
            for v_f in self.target_velocities:
                
                # 3. Sample different time horizons
                for T in self.time_horizons:
                    
                    trajectory_count += 1
                    
                    # LONGITUDINAL MOTION: s(t)
                    # --------------------------
                    # Start at s0, end at position based on average velocity
                    s_f = s0 + 0.5 * (s_dot0 + v_f) * T
                    
                    # Create quintic polynomial for s(t)
                    s_poly = QuinticPolynomial(
                        x0=s0, v0=s_dot0, a0=s_ddot0,
                        xf=s_f, vf=v_f, af=0.0,
                        T=T
                    )
                    
                    # LATERAL MOTION: d(t)
                    # --------------------
                    # Start at d0, end at target lane d_f
                    # Zero velocity and acceleration at endpoints (smooth lane change)
                    
                    d_poly = QuinticPolynomial(
                        x0=d0, v0=d_dot0, a0=d_ddot0,
                        xf=d_f, vf=0.0, af=0.0,
                        T=T
                    )
                    
                    # Create trajectory
                    traj = FrenetTrajectory(s_poly, d_poly)
                    trajectories.append(traj)
                    
                    # Print info about this trajectory
                    if d_f == 0:
                        maneuver = "STAY IN LANE"
                    elif d_f > 0:
                        maneuver = "LANE CHANGE RIGHT"
                    else:
                        maneuver = "LANE CHANGE LEFT"
                    
                    print(f"Trajectory {trajectory_count:2d}: {maneuver:20s} | "
                          f"v_target={v_f:4.1f}m/s | T={T:.1f}s | "
                          f"s_final={s_f:5.1f}m, d_final={d_f:4.1f}m")
        
        print()
        print(f"Generated {len(trajectories)} total trajectories!")
        print("="*60)
        
        return trajectories


# ============================================================================
# PART 4: VISUALIZATION
# ============================================================================

def visualize_trajectories(trajectories: List[FrenetTrajectory],
                          lane_width: float = 3.5):
    """
    Visualize all generated trajectories.
    
    Creates multiple plots:
    1. Trajectories in Frenet space (s vs d)
    2. Lateral position over time d(t)
    3. Longitudinal position over time s(t)
    4. Velocity profiles
    """
    
    # Create figure with subplots
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('Sampling-Based Motion Planner: Generated Trajectories', 
                 fontsize=16, fontweight='bold')
    
    # Define colors for different trajectory types
    colors = {
        'left': 'red',
        'center': 'blue',
        'right': 'green'
    }
    
    # ========================================
    # Plot 1: Trajectories in Frenet space
    # ========================================
    ax1 = axes[0, 0]
    
    # Draw lane boundaries
    ax1.axhline(y=-lane_width/2, color='gray', linestyle='--', linewidth=1, alpha=0.5)
    ax1.axhline(y=lane_width/2, color='gray', linestyle='--', linewidth=1, alpha=0.5)
    ax1.axhline(y=-3*lane_width/2, color='gray', linestyle='--', linewidth=1, alpha=0.5)
    ax1.axhline(y=3*lane_width/2, color='gray', linestyle='--', linewidth=1, alpha=0.5)
    ax1.axhline(y=0, color='blue', linestyle='-', linewidth=2, alpha=0.3, label='Centerline')
    
    # Plot each trajectory
    for traj in trajectories[0:100]:
        # Determine color based on final lateral position
        if traj.d[-1] < -lane_width/2:
            color = colors['left']
            alpha = 0.3
        elif traj.d[-1] > lane_width/2:
            color = colors['right']
            alpha = 0.3
        else:
            color = colors['center']
            alpha = 0.5
        
        ax1.plot(traj.s, traj.d, color=color, alpha=alpha, linewidth=1.5)
    
    # Mark starting point
    ax1.plot(trajectories[0].s[0], trajectories[0].d[0], 
            'ko', markersize=10, label='Start', zorder=10)
    
    ax1.set_xlabel('Longitudinal Position s (m)', fontsize=12)
    ax1.set_ylabel('Lateral Position d (m)', fontsize=12)
    ax1.set_title('Trajectories in Frenet Frame', fontsize=14, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.legend()
    
    # Add lane labels
    ax1.text(10, -lane_width, 'Left Lane', fontsize=10, ha='center')
    ax1.text(10, 0, 'Current Lane', fontsize=10, ha='center')
    ax1.text(10, lane_width, 'Right Lane', fontsize=10, ha='center')
    
    # ========================================
    # Plot 2: Lateral position over time
    # ========================================
    ax2 = axes[0, 1]
    
    for traj in trajectories:
        if traj.d[-1] < -lane_width/2:
            color = colors['left']
            label = 'Lane Change Left' if 'Lane Change Left' not in ax2.get_legend_handles_labels()[1] else ''
        elif traj.d[-1] > lane_width/2:
            color = colors['right']
            label = 'Lane Change Right' if 'Lane Change Right' not in ax2.get_legend_handles_labels()[1] else ''
        else:
            color = colors['center']
            label = 'Stay in Lane' if 'Stay in Lane' not in ax2.get_legend_handles_labels()[1] else ''
        
        ax2.plot(traj.times, traj.d, color=color, alpha=0.4, linewidth=1.5, label=label)
    
    ax2.axhline(y=0, color='blue', linestyle='--', linewidth=2, alpha=0.5)
    ax2.axhline(y=-lane_width, color='red', linestyle='--', linewidth=1, alpha=0.3)
    ax2.axhline(y=lane_width, color='green', linestyle='--', linewidth=1, alpha=0.3)
    
    ax2.set_xlabel('Time (s)', fontsize=12)
    ax2.set_ylabel('Lateral Position d(t) (m)', fontsize=12)
    ax2.set_title('Lateral Motion Over Time', fontsize=14, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    ax2.legend()
    
    # ========================================
    # Plot 3: Longitudinal position over time
    # ========================================
    ax3 = axes[1, 0]
    
    for traj in trajectories:
        # Color by target velocity
        velocity_ratio = traj.s_dot[-1] / max(t.s_dot[-1] for t in trajectories)
        color = plt.cm.viridis(velocity_ratio)
        ax3.plot(traj.times, traj.s, color=color, alpha=0.5, linewidth=1.5)
    
    ax3.set_xlabel('Time (s)', fontsize=12)
    ax3.set_ylabel('Longitudinal Position s(t) (m)', fontsize=12)
    ax3.set_title('Longitudinal Motion Over Time', fontsize=14, fontweight='bold')
    ax3.grid(True, alpha=0.3)
    
    # Add colorbar for velocity
    sm = plt.cm.ScalarMappable(cmap=plt.cm.viridis, 
                               norm=plt.Normalize(vmin=min(t.s_dot[-1] for t in trajectories),
                                                 vmax=max(t.s_dot[-1] for t in trajectories)))
    sm.set_array([])
    cbar = plt.colorbar(sm, ax=ax3)
    cbar.set_label('Target Velocity (m/s)', fontsize=10)
    
    # ========================================
    # Plot 4: Velocity profiles
    # ========================================
    ax4 = axes[1, 1]
    
    for traj in trajectories:
        velocity = np.sqrt(traj.s_dot**2 + traj.d_dot**2)
        
        if traj.d[-1] < -lane_width/2:
            color = colors['left']
        elif traj.d[-1] > lane_width/2:
            color = colors['right']
        else:
            color = colors['center']
        
        ax4.plot(traj.times, velocity, color=color, alpha=0.4, linewidth=1.5)
    
    ax4.set_xlabel('Time (s)', fontsize=12)
    ax4.set_ylabel('Velocity (m/s)', fontsize=12)
    ax4.set_title('Velocity Profiles', fontsize=14, fontweight='bold')
    ax4.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('/home/ahmad/Desktop/RuleBookDriving/outputs/sampling_planner_visualization.png', dpi=150, bbox_inches='tight')
    print("\n✓ Visualization saved to: sampling_planner_visualization.png")
    
    return fig


def demonstrate_single_quintic():
    """
    Demonstrate a single quintic polynomial trajectory.
    Shows how boundary conditions affect the shape.
    """
    print("\n" + "="*60)
    print("DEMONSTRATING SINGLE QUINTIC POLYNOMIAL")
    print("="*60)
    
    # Lane change example
    d0 = 0.0      # Start in center of lane
    d_dot0 = 0.0  # Not moving laterally
    d_ddot0 = 0.0 # No lateral acceleration
    
    df = 3.5      # End in right lane
    d_dotf = 0.0  # Stop moving laterally
    d_ddotf = 0.0 # No lateral acceleration at end
    
    T = 4.0       # Take 4 seconds
    
    print(f"\nBoundary Conditions:")
    print(f"  Initial: d(0) = {d0:.1f}m, d'(0) = {d_dot0:.1f}m/s, d''(0) = {d_ddot0:.1f}m/s²")
    print(f"  Final:   d({T}) = {df:.1f}m, d'({T}) = {d_dotf:.1f}m/s, d''({T}) = {d_ddotf:.1f}m/s²")
    
    # Create quintic polynomial
    poly = QuinticPolynomial(d0, d_dot0, d_ddot0, df, d_dotf, d_ddotf, T)
    
    print(f"\nComputed Coefficients:")
    print(f"  a0 = {poly.coeffs[5]:.4f}") 
    print(f"  a1 = {poly.coeffs[4]:.4f}")
    print(f"  a2 = {poly.coeffs[3]:.4f}")
    print(f"  a3 = {poly.coeffs[2]:.4f}")
    print(f"  a4 = {poly.coeffs[1]:.4f}")
    print(f"  a5 = {poly.coeffs[0]:.4f}")
    
    # 0 = a0 x^0 + a1 x^1 + a2 x^2 + a3 x^3 + a4 x^4 + a5 x^5

    # Sample the trajectory
    times = np.linspace(0, T, 100)
    positions = [poly.calc_point(t) for t in times]
    velocities = [poly.calc_first_derivative(t) for t in times]
    accelerations = [poly.calc_second_derivative(t) for t in times]
    
    # Plot
    fig, axes = plt.subplots(3, 1, figsize=(10, 8))
    fig.suptitle('Single Quintic Polynomial: Lane Change Maneuver', 
                 fontsize=14, fontweight='bold')
    
    # Position
    axes[0].plot(times, positions, 'b-', linewidth=2)
    axes[0].axhline(y=d0, color='gray', linestyle='--', alpha=0.5)
    axes[0].axhline(y=df, color='gray', linestyle='--', alpha=0.5)
    axes[0].plot(0, d0, 'go', markersize=10, label='Start')
    axes[0].plot(T, df, 'ro', markersize=10, label='End')
    axes[0].set_ylabel('Position d(t) (m)', fontsize=12)
    axes[0].set_title('Lateral Position', fontsize=12)
    axes[0].grid(True, alpha=0.3)
    axes[0].legend()
    
    # Velocity
    axes[1].plot(times, velocities, 'g-', linewidth=2)
    axes[1].axhline(y=0, color='gray', linestyle='--', alpha=0.5)
    axes[1].set_ylabel('Velocity d\'(t) (m/s)', fontsize=12)
    axes[1].set_title('Lateral Velocity', fontsize=12)
    axes[1].grid(True, alpha=0.3)
    
    # Acceleration
    axes[2].plot(times, accelerations, 'r-', linewidth=2)
    axes[2].axhline(y=0, color='gray', linestyle='--', alpha=0.5)
    axes[2].set_ylabel('Acceleration d\'\'(t) (m/s²)', fontsize=12)
    axes[2].set_xlabel('Time (s)', fontsize=12)
    axes[2].set_title('Lateral Acceleration', fontsize=12)
    axes[2].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('/home/ahmad/Desktop/RuleBookDriving/outputs/quintic_polynomial_demo.png', dpi=150, bbox_inches='tight')
    print("\n✓ Quintic demo saved to: quintic_polynomial_demo.png")
    
    return fig


# ============================================================================
# MAIN DEMONSTRATION
# ============================================================================

def main():
    """
    Main function to demonstrate the sampling planner.
    """
    print("\n" + "="*60)
    print("SAMPLING-BASED MOTION PLANNER DEMONSTRATION")
    print("="*60)
    print("\nThis demonstration shows how we generate multiple candidate")
    print("trajectories by sampling different terminal states.")
    print()
    
    # ========================================
    # STEP 1: Demonstrate single quintic
    # ========================================
    print("\n[STEP 1] Understanding Quintic Polynomials")
    print("-" * 60)
    demonstrate_single_quintic()
    
    # ========================================
    # STEP 2: Create sampling planner
    # ========================================
    print("\n[STEP 2] Creating Sampling Planner")
    print("-" * 60)
    


    planner = SamplingPlanner(
        lane_width=3.5,
        target_velocities=[8.0, 12.0, 15.0, 18.0],  # m/s
        time_horizons=[3.0, 4.0, 5.0]  # seconds
    )
    
    print(f"\nPlanner Configuration:")
    print(f"  Lane width: {planner.lane_width} m")
    print(f"  Lateral samples: {planner.lateral_samples}")
    print(f"  Target velocities: {planner.target_velocities} m/s")
    print(f"  Time horizons: {planner.time_horizons} s")
    print(f"\n  Total combinations: {len(planner.lateral_samples)} × "
          f"{len(planner.target_velocities)} × {len(planner.time_horizons)} = "
          f"{len(planner.lateral_samples) * len(planner.target_velocities) * len(planner.time_horizons)} trajectories")
    
    # ========================================
    # STEP 3: Generate trajectories
    # ========================================
    print("\n[STEP 3] Generating Candidate Trajectories")
    print("-" * 60)
    
    # Initial state (vehicle starting conditions)
    s0 = 0.0        # Start at s = 0
    d0 = 0.0        # Start in center of current lane
    s_dot0 = 10.0   # Initial speed: 10 m/s
    d_dot0 = 0.0    # No lateral velocity
    
    trajectories = planner.generate_trajectories(s0, d0, s_dot0, d_dot0)
    
    # ========================================
    # STEP 4: Visualize trajectories
    # ========================================
    print("\n[STEP 4] Visualizing Generated Trajectories")
    print("-" * 60)
    
    fig = visualize_trajectories(trajectories, lane_width=planner.lane_width)
    
    # ========================================
    # STEP 5: Summary statistics
    # ========================================
    print("\n[STEP 5] Summary Statistics")
    print("-" * 60)
    
    print(f"\nTotal trajectories generated: {len(trajectories)}")
    
    random_sample = np.random.uniform(low = 0, high = 1)


    # Count by maneuver type
    left_count = sum(1 for t in trajectories if t.d[-1] < -1.0)
    center_count = sum(1 for t in trajectories if abs(t.d[-1]) <= 1.0)
    right_count = sum(1 for t in trajectories if t.d[-1] > 1.0)
    
    print(f"  Lane change left:  {left_count}")
    print(f"  Stay in lane:      {center_count}")
    print(f"  Lane change right: {right_count}")
    
    # Velocity statistics
    final_velocities = [np.sqrt(t.s_dot[-1]**2 + t.d_dot[-1]**2) for t in trajectories]
    print(f"\nFinal velocity range:")
    print(f"  Min: {min(final_velocities):.1f} m/s")
    print(f"  Max: {max(final_velocities):.1f} m/s")
    print(f"  Mean: {np.mean(final_velocities):.1f} m/s")
    
    # Duration statistics
    durations = [t.T for t in trajectories]
    print(f"\nTrajectory durations:")
    print(f"  Min: {min(durations):.1f} s")
    print(f"  Max: {max(durations):.1f} s")
    
    print("\n" + "="*60)
    print("DEMONSTRATION COMPLETE!")
    print("="*60)
    print("\nKey Takeaways:")
    print("  1. Quintic polynomials ensure smooth trajectories")
    print("  2. Sampling explores different maneuver options")
    print("  3. We separate longitudinal (s) and lateral (d) planning")
    print("  4. Each trajectory has unique speed, lane, and duration")
    print("\nNext step: Feed these to the rulebook for evaluation!")
    print("="*60 + "\n")
    
    plt.show()


if __name__ == "__main__":
    main()