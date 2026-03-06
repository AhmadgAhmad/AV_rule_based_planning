"""
Rule-Based Planning: Interactive Tutorial
==========================================

This script teaches rule evaluation and lexicographic ordering through
simple 2D trajectory examples and visualizations.

Topics:
1. Defining rules (scoring functions)
2. Evaluating single trajectories
3. Comparing multiple trajectories
4. Lexicographic ordering (total order)
5. Understanding priority hierarchies

Author: Ahmad Ahmad
For: Nidhi (Curious Cardinals Mentorship)
Date: January 2026
"""

import numpy as np
import matplotlib.pyplot as plt
from typing import List, Tuple, Callable
from dataclasses import dataclass


# ============================================================================
# TRAJECTORY REPRESENTATION
# ============================================================================

@dataclass
class Trajectory:
    """
    Simple 2D trajectory representation.
    
    Attributes:
        x: X coordinates (list or array)
        y: Y coordinates (list or array)
        name: Human-readable name
        color: Color for plotting
    """
    x: np.ndarray
    y: np.ndarray
    name: str
    color: str = 'blue'
    
    def __post_init__(self):
        """Convert to numpy arrays if needed."""
        self.x = np.array(self.x)
        self.y = np.array(self.y)
    
    @property
    def length(self):
        """Total path length."""
        dx = np.diff(self.x)
        dy = np.diff(self.y)
        return np.sum(np.sqrt(dx**2 + dy**2))
    
    @property
    def max_y(self):
        """Maximum lateral deviation."""
        return np.max(np.abs(self.y))
    
    @property
    def smoothness(self):
        """Measure of smoothness (lower is smoother)."""
        # Second derivative approximation
        if len(self.x) < 3:
            return 0.0
        d2x = np.diff(self.x, 2)
        d2y = np.diff(self.y, 2)
        return np.sum(d2x**2 + d2y**2)


# ============================================================================
# TRAJECTORY GENERATORS
# ============================================================================

def generate_straight_trajectory(length: float = 10.0) -> Trajectory:
    """Generate straight line trajectory."""
    x = np.linspace(0, length, 50)
    y = np.zeros_like(x)
    return Trajectory(x, y, "Straight", color='green')


def generate_curved_trajectory(length: float = 10.0, 
                               amplitude: float = 2.0) -> Trajectory:
    """Generate smooth curved trajectory."""
    x = np.linspace(0, length, 50)
    y = amplitude * np.sin(2 * np.pi * x / length)
    return Trajectory(x, y, "Curved", color='blue')


def generate_zigzag_trajectory(length: float = 10.0,
                               amplitude: float = 2.0,
                               n_zigs: int = 3) -> Trajectory:
    """Generate zigzag (non-smooth) trajectory."""
    x = np.linspace(0, length, 50)
    y = amplitude * np.sign(np.sin(2 * np.pi * n_zigs * x / length))
    return Trajectory(x, y, "Zigzag", color='red')


def generate_long_curved_trajectory(length: float = 15.0,
                                    amplitude: float = 1.5) -> Trajectory:
    """Generate longer curved trajectory."""
    x = np.linspace(0, length, 50)
    y = amplitude * np.sin(2 * np.pi * x / length)
    return Trajectory(x, y, "Long Curved", color='purple')


# ============================================================================
# RULE DEFINITIONS (Scoring Functions)
# ============================================================================

def rule_path_length(traj: Trajectory) -> float:
    """
    Rule: Minimize path length.
    
    Returns:
        Path length (lower is better)
    """
    return traj.length


def rule_lateral_deviation(traj: Trajectory) -> float:
    """
    Rule: Minimize lateral deviation from centerline (y=0).
    
    Returns:
        Maximum absolute y value (lower is better)
    """
    return traj.max_y


def rule_smoothness(traj: Trajectory) -> float:
    """
    Rule: Maximize smoothness (minimize jerk).
    
    Returns:
        Smoothness metric (lower is better)
    """
    return traj.smoothness


def rule_safety_margin(traj: Trajectory, obstacle_y: float = 1.5) -> float:
    """
    Rule: Maintain safety margin from obstacle at y = obstacle_y.
    
    Returns:
        Violation metric (0 = safe, >0 = violation)
    """
    violations = np.maximum(0, obstacle_y - np.abs(traj.y))
    return np.sum(violations)


# ============================================================================
# VISUALIZATION FUNCTIONS
# ============================================================================

def visualize_single_trajectory_with_rule(traj: Trajectory, 
                                         rule_func: Callable,
                                         rule_name: str):
    """
    Visualize a single trajectory with its rule score.
    
    Args:
        traj: Trajectory to visualize
        rule_func: Rule function to evaluate
        rule_name: Name of the rule for display
    """
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))
    
    # Plot trajectory
    ax1.plot(traj.x, traj.y, color=traj.color, linewidth=2, label=traj.name)
    ax1.axhline(y=0, color='gray', linestyle='--', linewidth=1, alpha=0.5, label='Centerline')
    ax1.grid(True, alpha=0.3)
    ax1.set_xlabel('X (m)', fontsize=12)
    ax1.set_ylabel('Y (m)', fontsize=12)
    ax1.set_title(f'Trajectory: {traj.name}', fontsize=14, fontweight='bold')
    ax1.legend()
    ax1.set_aspect('equal')
    
    # Compute and display score
    score = rule_func(traj)
    
    ax2.text(0.5, 0.7, f'{rule_name}', 
            ha='center', va='center', fontsize=16, fontweight='bold',
            transform=ax2.transAxes)
    
    ax2.text(0.5, 0.5, f'Score: {score:.3f}',
            ha='center', va='center', fontsize=20, 
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5),
            transform=ax2.transAxes)
    
    ax2.text(0.5, 0.3, '(Lower is better)',
            ha='center', va='center', fontsize=12, style='italic',
            transform=ax2.transAxes)
    
    ax2.axis('off')
    
    plt.tight_layout()
    plt.show()


def compare_trajectories_single_rule(trajectories: List[Trajectory],
                                     rule_func: Callable,
                                     rule_name: str):
    """
    Compare multiple trajectories using a single rule.
    
    Args:
        trajectories: List of trajectories to compare
        rule_func: Rule function to evaluate
        rule_name: Name of the rule
    """
    fig = plt.figure(figsize=(16, 6))
    
    # Trajectories plot
    ax1 = plt.subplot(1, 2, 1)
    for traj in trajectories:
        ax1.plot(traj.x, traj.y, color=traj.color, linewidth=2, 
                label=traj.name, marker='o', markersize=3, markevery=5)
    
    ax1.axhline(y=0, color='gray', linestyle='--', linewidth=1, alpha=0.5)
    ax1.grid(True, alpha=0.3)
    ax1.set_xlabel('X (m)', fontsize=12)
    ax1.set_ylabel('Y (m)', fontsize=12)
    ax1.set_title('All Trajectories', fontsize=14, fontweight='bold')
    ax1.legend()
    
    # Scores comparison
    ax2 = plt.subplot(1, 2, 2)
    
    names = [t.name for t in trajectories]
    scores = [rule_func(t) for t in trajectories]
    colors = [t.color for t in trajectories]
    
    bars = ax2.barh(names, scores, color=colors, alpha=0.7, edgecolor='black', linewidth=2)
    
    # Highlight best (lowest score)
    best_idx = np.argmin(scores)
    bars[best_idx].set_edgecolor('gold')
    bars[best_idx].set_linewidth(4)
    
    ax2.set_xlabel('Score (lower is better)', fontsize=12)
    ax2.set_title(f'Rule: {rule_name}', fontsize=14, fontweight='bold')
    ax2.grid(True, axis='x', alpha=0.3)
    
    # Annotate winner
    ax2.text(scores[best_idx], best_idx, ' ★ BEST', 
            va='center', fontsize=12, fontweight='bold', color='gold')
    
    plt.tight_layout()
    plt.show()
    
    # Print ranking
    sorted_indices = np.argsort(scores)
    print("\n" + "="*50)
    print(f"RANKING by {rule_name}")
    print("="*50)
    for rank, idx in enumerate(sorted_indices, 1):
        star = "★" if rank == 1 else " "
        print(f"{star} {rank}. {trajectories[idx].name:15s} Score: {scores[idx]:7.3f}")
    print("="*50 + "\n")


def compare_trajectories_multiple_rules(trajectories: List[Trajectory],
                                       rules: List[Tuple[Callable, str]]):
    """
    Compare trajectories using multiple rules (lexicographic ordering).
    
    Args:
        trajectories: List of trajectories
        rules: List of (rule_function, rule_name) tuples in priority order
    """
    n_rules = len(rules)
    n_traj = len(trajectories)
    
    # Compute all scores
    scores_matrix = np.zeros((n_traj, n_rules))
    for i, traj in enumerate(trajectories):
        for j, (rule_func, _) in enumerate(rules):
            scores_matrix[i, j] = rule_func(traj)
    
    # Plot
    fig = plt.figure(figsize=(16, 10))
    
    # Trajectories
    ax1 = plt.subplot(2, 2, 1)
    for traj in trajectories:
        ax1.plot(traj.x, traj.y, color=traj.color, linewidth=2,
                label=traj.name, marker='o', markersize=3, markevery=5)
    ax1.axhline(y=0, color='gray', linestyle='--', linewidth=1, alpha=0.5)
    ax1.grid(True, alpha=0.3)
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_title('All Trajectories', fontweight='bold')
    ax1.legend()
    
    # Scores heatmap
    ax2 = plt.subplot(2, 2, 2)
    im = ax2.imshow(scores_matrix.T, aspect='auto', cmap='RdYlGn_r')
    ax2.set_xticks(range(n_traj))
    ax2.set_xticklabels([t.name for t in trajectories], rotation=45, ha='right')
    ax2.set_yticks(range(n_rules))
    ax2.set_yticklabels([f"P{i}: {name}" for i, (_, name) in enumerate(rules)])
    ax2.set_title('Score Matrix\n(Green=Low/Good, Red=High/Bad)', fontweight='bold')
    
    # Annotate values
    for i in range(n_traj):
        for j in range(n_rules):
            text = ax2.text(i, j, f'{scores_matrix[i, j]:.2f}',
                          ha="center", va="center", color="black", fontsize=10)
    
    plt.colorbar(im, ax=ax2)
    
    # Individual rule comparisons
    for idx, (rule_func, rule_name) in enumerate(rules):
        ax = plt.subplot(2, n_rules, n_rules + idx + 1)
        
        names = [t.name for t in trajectories]
        scores = scores_matrix[:, idx]
        colors = [t.color for t in trajectories]
        
        bars = ax.barh(names, scores, color=colors, alpha=0.7, edgecolor='black')
        
        # Highlight best
        best_idx = np.argmin(scores)
        bars[best_idx].set_edgecolor('gold')
        bars[best_idx].set_linewidth(3)
        
        ax.set_xlabel('Score')
        ax.set_title(f'P{idx}: {rule_name}', fontsize=10, fontweight='bold')
        ax.grid(True, axis='x', alpha=0.3)
    
    plt.tight_layout()
    plt.show()
    
    # Lexicographic ordering
    print("\n" + "="*70)
    print("LEXICOGRAPHIC ORDERING (Total Order)")
    print("="*70)
    print("\nRules in priority order:")
    for i, (_, name) in enumerate(rules):
        print(f"  P{i}: {name}")
    
    # Convert to list of tuples for sorting
    traj_scores = [(i, tuple(scores_matrix[i, :])) for i in range(n_traj)]
    
    # Sort lexicographically
    traj_scores.sort(key=lambda x: x[1])
    
    print("\nRanking:")
    print("-" * 70)
    for rank, (idx, scores) in enumerate(traj_scores, 1):
        traj = trajectories[idx]
        star = "★" if rank == 1 else " "
        score_str = " ".join([f"P{i}={s:.2f}" for i, s in enumerate(scores)])
        print(f"{star} {rank}. {traj.name:15s} [{score_str}]")
    
    # Explain winner
    print("\n" + "="*70)
    print("WHY IS THIS THE WINNER?")
    print("="*70)
    winner_idx = traj_scores[0][0]
    winner = trajectories[winner_idx]
    
    print(f"\nWinner: {winner.name}")
    
    # Compare with runner-up
    if len(traj_scores) > 1:
        runnerup_idx = traj_scores[1][0]
        runnerup = trajectories[runnerup_idx]
        
        print(f"\nComparing with runner-up: {runnerup.name}")
        
        for i, (_, rule_name) in enumerate(rules):
            winner_score = scores_matrix[winner_idx, i]
            runnerup_score = scores_matrix[runnerup_idx, i]
            
            if winner_score < runnerup_score:
                print(f"\n  P{i} ({rule_name}):")
                print(f"    {winner.name}: {winner_score:.3f} ✓ BETTER")
                print(f"    {runnerup.name}: {runnerup_score:.3f}")
                print(f"    → {winner.name} WINS at priority {i}!")
                break
            elif winner_score > runnerup_score:
                print(f"\n  P{i} ({rule_name}):")
                print(f"    {winner.name}: {winner_score:.3f}")
                print(f"    {runnerup.name}: {runnerup_score:.3f} ✓ BETTER")
                print(f"    → ERROR! This shouldn't happen!")
                break
            else:
                print(f"\n  P{i} ({rule_name}):")
                print(f"    Both tied: {winner_score:.3f}")
                print(f"    → Continue to next priority...")
    
    print("="*70 + "\n")


# ============================================================================
# INTERACTIVE EXERCISES
# ============================================================================

def exercise_1_define_your_rule():
    """
    Exercise 1: Define your own rule and test it.
    """
    print("\n" + "="*70)
    print("EXERCISE 1: Define Your Own Rule")
    print("="*70)
    print("\nTask: Create a rule that penalizes trajectories that go above y=1.0")
    print("\nHint: Count how many points have y > 1.0")
    print("\nYour turn! Complete this function:\n")
    
    print("""
def my_custom_rule(traj: Trajectory) -> float:
    '''Penalize trajectories above y=1.0'''
    # YOUR CODE HERE
    violations = 0
    for y_val in traj.y:
        if y_val > 1.0:
            violations += 1
    return violations
    """)
    
    # Test trajectories
    traj1 = generate_straight_trajectory()
    traj2 = generate_curved_trajectory(amplitude=0.5)
    traj3 = generate_curved_trajectory(amplitude=2.0)
    
    print("\nTest your rule on these trajectories:")
    print(f"  Straight (y always 0)")
    print(f"  Small curve (max y ≈ 0.5)")
    print(f"  Large curve (max y ≈ 2.0)")
    print("\nExpected scores: 0, 0, ~25 (approximately)")


def exercise_2_compare_two_rules():
    """
    Exercise 2: Compare trajectories with two rules in different orders.
    """
    print("\n" + "="*70)
    print("EXERCISE 2: Rule Priority Matters")
    print("="*70)
    print("\nQuestion: Does priority order matter?")
    print("\nTest case:")
    print("  Trajectory A: Short but wiggly")
    print("  Trajectory B: Long but smooth")
    print("\nRules:")
    print("  R1: Minimize length")
    print("  R2: Minimize wiggliness")
    print("\nTry both orders:")
    print("  Order 1: [R1, R2] (length first)")
    print("  Order 2: [R2, R1] (smoothness first)")
    print("\nWhich trajectory wins in each case?")


def exercise_3_design_rulebook():
    """
    Exercise 3: Design a rulebook for a specific scenario.
    """
    print("\n" + "="*70)
    print("EXERCISE 3: Design Your Rulebook")
    print("="*70)
    print("\nScenario: Autonomous vehicle must:")
    print("  1. Avoid collision (CRITICAL)")
    print("  2. Stay in lane (IMPORTANT)")
    print("  3. Be fuel efficient (NICE TO HAVE)")
    print("\nQuestion: What should the priority order be?")
    print("\nOptions:")
    print("  A. [Collision, Lane, Fuel]")
    print("  B. [Fuel, Lane, Collision]")
    print("  C. [Lane, Fuel, Collision]")
    print("\nThink: Which makes most sense for safety?")


# ============================================================================
# DEMONSTRATION SCENARIOS
# ============================================================================

def demo_single_rule():
    """Demo: Evaluate trajectories with a single rule."""
    print("\n" + "="*70)
    print("DEMO 1: Single Rule Evaluation")
    print("="*70)
    
    # Generate trajectories
    traj1 = generate_straight_trajectory()
    traj2 = generate_curved_trajectory()
    traj3 = generate_zigzag_trajectory()
    
    trajectories = [traj1, traj2, traj3]
    
    # Test with path length rule
    print("\nRule: Minimize Path Length")
    compare_trajectories_single_rule(trajectories, rule_path_length, "Path Length")
    
    input("Press Enter to continue...")
    
    # Test with smoothness rule
    print("\nRule: Minimize Jerk (Maximize Smoothness)")
    compare_trajectories_single_rule(trajectories, rule_smoothness, "Smoothness (Jerk)")


def demo_lexicographic_ordering():
    """Demo: Lexicographic ordering with multiple rules."""
    print("\n" + "="*70)
    print("DEMO 2: Lexicographic Ordering (Total Order)")
    print("="*70)
    
    # Generate diverse trajectories
    traj1 = generate_straight_trajectory(length=10.0)
    traj2 = generate_curved_trajectory(length=10.0, amplitude=1.0)
    traj3 = generate_long_curved_trajectory(length=15.0, amplitude=1.5)
    traj4 = generate_zigzag_trajectory(length=10.0, amplitude=2.0)
    
    trajectories = [traj1, traj2, traj3, traj4]
    
    # Define rulebook
    rules = [
        (rule_smoothness, "Smoothness"),
        (rule_lateral_deviation, "Lateral Deviation"),
        (rule_path_length, "Path Length")
    ]
    
    compare_trajectories_multiple_rules(trajectories, rules)


def demo_priority_matters():
    """Demo: Show that priority order changes the winner."""
    print("\n" + "="*70)
    print("DEMO 3: Priority Order Matters!")
    print("="*70)
    
    # Create two contrasting trajectories
    short_wiggly = generate_zigzag_trajectory(length=8.0, amplitude=1.5)
    short_wiggly.name = "Short Wiggly"
    short_wiggly.color = 'red'
    
    long_smooth = generate_curved_trajectory(length=12.0, amplitude=0.5)
    long_smooth.name = "Long Smooth"
    long_smooth.color = 'green'
    
    trajectories = [short_wiggly, long_smooth]
    
    print("\n--- ORDER 1: Length First, Then Smoothness ---")
    rules_order1 = [
        (rule_path_length, "Path Length"),
        (rule_smoothness, "Smoothness")
    ]
    compare_trajectories_multiple_rules(trajectories, rules_order1)
    
    input("\nPress Enter to see the opposite priority order...")
    
    print("\n--- ORDER 2: Smoothness First, Then Length ---")
    rules_order2 = [
        (rule_smoothness, "Smoothness"),
        (rule_path_length, "Path Length")
    ]
    compare_trajectories_multiple_rules(trajectories, rules_order2)
    
    print("\n" + "="*70)
    print("KEY INSIGHT: Priority order determines the winner!")
    print("="*70)


# ============================================================================
# MAIN MENU
# ============================================================================

def main():
    """Main interactive menu."""
    print("\n" + "="*70)
    print("RULE-BASED PLANNING: Interactive Tutorial")
    print("="*70)
    print("\nLearning Objectives:")
    print("  1. Understand how to define rules (scoring functions)")
    print("  2. Evaluate trajectories with single rules")
    print("  3. Compare trajectories using multiple rules")
    print("  4. Understand lexicographic ordering (total order)")
    print("  5. Recognize that priority order matters")
    
    while True:
        print("\n" + "="*70)
        print("MENU")
        print("="*70)
        print("\nDemonstrations:")
        print("  1. Single Rule Evaluation")
        print("  2. Lexicographic Ordering (Multiple Rules)")
        print("  3. Priority Order Matters")
        print("\nExercises:")
        print("  4. Exercise 1: Define Your Own Rule")
        print("  5. Exercise 2: Compare Two Rules")
        print("  6. Exercise 3: Design a Rulebook")
        print("\n  0. Exit")
        
        choice = input("\nEnter your choice: ").strip()
        
        if choice == '1':
            demo_single_rule()
        elif choice == '2':
            demo_lexicographic_ordering()
        elif choice == '3':
            demo_priority_matters()
        elif choice == '4':
            exercise_1_define_your_rule()
        elif choice == '5':
            exercise_2_compare_two_rules()
        elif choice == '6':
            exercise_3_design_rulebook()
        elif choice == '0':
            print("\nThank you for learning about rule-based planning!")
            print("Key takeaways:")
            print("  ✓ Rules are just scoring functions")
            print("  ✓ Lower scores are better")
            print("  ✓ Lexicographic ordering compares priority by priority")
            print("  ✓ Higher priorities ALWAYS win")
            print("  ✓ This gives us principled decision making!")
            break
        else:
            print("Invalid choice. Please try again.")


if __name__ == "__main__":
    main()
