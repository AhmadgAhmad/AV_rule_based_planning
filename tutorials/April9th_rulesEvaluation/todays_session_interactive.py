"""
Today's Session: Intersection Scenarios Interactive Demo
=========================================================

Work through 4 intersection scenarios using rule-based planning.

This script helps you:
1. Understand the rulebook (5 priorities)
2. Evaluate each scenario step-by-step
3. See how lexicographic ordering picks the winner
4. Visualize decisions with tables and diagrams

For: Ahmad & Nidhi's Session 2
"""

import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass
from typing import List, Tuple
from enum import Enum


# ============================================================================
# RULEBOOK DEFINITION
# ============================================================================

class Priority(Enum):
    """The five priorities in our rulebook."""
    P0_COLLISION = 0
    P1_LEGAL = 1
    P2_BLOCKING = 2
    P3_PROGRESS = 3
    P4_EFFICIENCY = 4


@dataclass
class Action:
    """Represents a possible action at intersection."""
    name: str  # STOP, STRAIGHT, RIGHT, LEFT
    collision: float  # P0: 0=safe, 1=collision
    legal: float  # P1: 0=legal, 1+=violations
    blocking: float  # P2: 0=clear, 1=blocking
    progress: float  # P3: distance (negative to maximize)
    efficiency: float  # P4: time in seconds
    
    @property
    def score_vector(self):
        """Return score as tuple for lexicographic comparison."""
        return (self.collision, self.legal, self.blocking, 
                self.progress, self.efficiency)
    
    def __lt__(self, other):
        """Enable direct comparison using lexicographic ordering."""
        return self.score_vector < other.score_vector


# ============================================================================
# SCENARIO DEFINITIONS
# ============================================================================

def scenario_a_green_clear():
    """
    Scenario A: Green light, clear intersection
    
    Expected winner: STRAIGHT (best progress)
    """
    print("\n" + "="*70)
    print("SCENARIO A: Green Light, Clear Intersection 🟢")
    print("="*70)
    print("\nContext:")
    print("  Traffic Light: GREEN")
    print("  Intersection: CLEAR (no obstacles)")
    print("  Distance: 25m")
    print("  Speed: 10 m/s (36 km/h)")
    print("\nWhat should we do?")
    
    actions = [
        Action("STOP", 
               collision=0, legal=0, blocking=0, 
               progress=-23, efficiency=float('inf')),
        Action("STRAIGHT", 
               collision=0, legal=0, blocking=0, 
               progress=-40, efficiency=5),
        Action("RIGHT", 
               collision=0, legal=0, blocking=0, 
               progress=-25, efficiency=8),
        Action("LEFT", 
               collision=0, legal=0, blocking=0, 
               progress=-35, efficiency=7),
    ]
    
    return actions, "All actions safe and legal. STRAIGHT has best progress."


def scenario_b_yellow_clear():
    """
    Scenario B: Yellow light
    
    Expected winner: STOP (conservative, can stop safely)
    """
    print("\n" + "="*70)
    print("SCENARIO B: Yellow Light 🟡")
    print("="*70)
    print("\nContext:")
    print("  Traffic Light: YELLOW")
    print("  Distance: 25m")
    print("  Speed: 10 m/s")
    print("  Can stop? Just barely (stopping distance = 25m)")
    print("\nWhat should we do?")
    
    actions = [
        Action("STOP", 
               collision=0, legal=0, blocking=0, 
               progress=-23, efficiency=3),
        Action("STRAIGHT", 
               collision=0, legal=0, blocking=0, 
               progress=-40, efficiency=5),
        Action("RIGHT", 
               collision=0, legal=0, blocking=0, 
               progress=-25, efficiency=8),
    ]
    
    return actions, "All safe and legal. Conservative choice is STOP."


def scenario_c_red_clear():
    """
    Scenario C: Red light
    
    Expected winner: RIGHT (only legal moving option)
    """
    print("\n" + "="*70)
    print("SCENARIO C: Red Light 🔴")
    print("="*70)
    print("\nContext:")
    print("  Traffic Light: RED")
    print("  Intersection: CLEAR")
    print("\nWhat can we legally do?")
    
    actions = [
        Action("STOP", 
               collision=0, legal=0, blocking=0, 
               progress=-23, efficiency=float('inf')),
        Action("STRAIGHT", 
               collision=0, legal=1, blocking=0,  # ILLEGAL!
               progress=-40, efficiency=5),
        Action("RIGHT", 
               collision=0, legal=0, blocking=0,  # Legal on red
               progress=-25, efficiency=8),
        Action("LEFT", 
               collision=0, legal=1, blocking=0,  # ILLEGAL!
               progress=-35, efficiency=7),
    ]
    
    return actions, "STRAIGHT and LEFT are illegal (P1 violation). RIGHT is only legal moving option."


def scenario_d_green_blocked():
    """
    Scenario D: Green light but intersection blocked
    
    Expected winner: RIGHT (doesn't block, makes progress)
    """
    print("\n" + "="*70)
    print("SCENARIO D: Green Light, BUT Blocked! 🟢🚗")
    print("="*70)
    print("\nContext:")
    print("  Traffic Light: GREEN (legal to go)")
    print("  Intersection: BLOCKED by traffic jam")
    print("  Other cars are stuck in the intersection!")
    print("\nCan we go on green?")
    
    actions = [
        Action("STOP", 
               collision=0, legal=0, blocking=0, 
               progress=-23, efficiency=float('inf')),
        Action("STRAIGHT", 
               collision=0, legal=0, blocking=1,  # Would block!
               progress=-40, efficiency=5),
        Action("RIGHT", 
               collision=0, legal=0, blocking=0,  # Clears intersection
               progress=-25, efficiency=8),
        Action("LEFT", 
               collision=0, legal=0, blocking=1,  # Would block!
               progress=-35, efficiency=7),
    ]
    
    return actions, "Green doesn't always mean GO! Don't block the box (P2 violation)."


# ============================================================================
# VISUALIZATION AND COMPARISON
# ============================================================================

def print_score_table(actions: List[Action]):
    """Print nicely formatted score table."""
    print("\nScore Table:")
    print("-" * 70)
    print(f"{'Action':10s} P0(Coll) P1(Legal) P2(Block) P3(Prog) P4(Time)")
    print("-" * 70)
    
    for action in actions:
        p4_str = "∞" if action.efficiency == float('inf') else f"{action.efficiency:.0f}s"
        print(f"{action.name:10s}    {action.collision:.0f}        "
              f"{action.legal:.0f}         {action.blocking:.0f}      "
              f"{action.progress:5.0f}m   {p4_str:>5s}")
    print("-" * 70)


def explain_comparison(actions: List[Action]):
    """Explain the lexicographic comparison step-by-step."""
    print("\nLexicographic Comparison:")
    print("=" * 70)
    
    # Sort actions
    sorted_actions = sorted(actions)
    winner = sorted_actions[0]
    
    print(f"\nWinner: {winner.name}")
    print("\nStep-by-step reasoning:")
    
    # Compare winner with others
    for i, action in enumerate(sorted_actions[1:], 1):
        print(f"\n{winner.name} vs {action.name}:")
        
        # Compare each priority
        priorities = ['P0 (Collision)', 'P1 (Legal)', 'P2 (Blocking)', 
                     'P3 (Progress)', 'P4 (Efficiency)']
        
        winner_scores = winner.score_vector
        action_scores = action.score_vector
        
        for j, priority_name in enumerate(priorities):
            w_score = winner_scores[j]
            a_score = action_scores[j]
            
            if w_score == a_score:
                print(f"  {priority_name}: {w_score} = {a_score} (tied, continue)")
            elif w_score < a_score:
                print(f"  {priority_name}: {w_score} < {a_score} → {winner.name} WINS at {priority_name}!")
                break
            else:
                print(f"  {priority_name}: {w_score} > {a_score} → {action.name} would win?!")
                print(f"  ERROR: This shouldn't happen!")
                break
    
    print("\n" + "=" * 70)
    print(f"FINAL DECISION: {winner.name}")
    print("=" * 70)
    
    return winner


def visualize_scenario(actions: List[Action], scenario_name: str):
    """Create bar chart visualization of scores."""
    fig, axes = plt.subplots(1, 5, figsize=(18, 4))
    fig.suptitle(f'{scenario_name} - Score Comparison', 
                fontsize=16, fontweight='bold')
    
    names = [a.name for a in actions]
    colors = ['green', 'blue', 'orange', 'red'][:len(actions)]
    
    # Get winner
    winner = min(actions)
    winner_idx = actions.index(winner)
    
    priority_names = ['P0: Collision', 'P1: Legal', 'P2: Blocking', 
                     'P3: Progress', 'P4: Efficiency']
    
    for i, (ax, priority_name) in enumerate(zip(axes, priority_names)):
        scores = [a.score_vector[i] for a in actions]
        
        # Handle infinity
        if any(s == float('inf') for s in scores):
            scores = [100 if s == float('inf') else s for s in scores]
        
        # For progress (negative), show absolute value
        if i == 3:
            scores = [-s for s in scores]
            ax.set_ylabel('Distance (m)', fontsize=10)
        else:
            ax.set_ylabel('Score', fontsize=10)
        
        bars = ax.bar(names, scores, color=colors, alpha=0.7, edgecolor='black')
        
        # Highlight winner
        bars[winner_idx].set_edgecolor('gold')
        bars[winner_idx].set_linewidth(4)
        
        ax.set_title(priority_name, fontsize=12, fontweight='bold')
        ax.set_xlabel('Action', fontsize=10)
        ax.grid(True, axis='y', alpha=0.3)
        
        # Add value labels
        for j, (bar, score) in enumerate(zip(bars, scores)):
            height = bar.get_height()
            label = '∞' if actions[j].score_vector[i] == float('inf') else f'{score:.0f}'
            ax.text(bar.get_x() + bar.get_width()/2., height,
                   label, ha='center', va='bottom', fontsize=9)
    
    plt.tight_layout()
    plt.show()


# ============================================================================
# INTERACTIVE SESSION
# ============================================================================

def run_scenario(scenario_func, scenario_name: str, visualize: bool = True):
    """Run a complete scenario analysis."""
    # Generate scenario
    actions, explanation = scenario_func()
    
    # Print score table
    print_score_table(actions)
    
    # Ask for prediction
    print("\n" + "="*70)
    print("YOUR TURN: Which action should win?")
    print("="*70)
    guess = input("Enter your guess (STOP/STRAIGHT/RIGHT/LEFT): ").strip().upper()
    
    # Show answer
    print("\n" + "="*70)
    print("ANALYSIS:")
    print("="*70)
    winner = explain_comparison(actions)
    
    # Check guess
    if guess == winner.name:
        print(f"\n✓ CORRECT! You predicted {winner.name}!")
    else:
        print(f"\n✗ Not quite. You guessed {guess}, but {winner.name} wins.")
    
    # Explanation
    print(f"\nWhy? {explanation}")
    
    # Visualize
    if visualize:
        visualize_scenario(actions, scenario_name)
    
    input("\nPress Enter to continue...")


def show_rulebook():
    """Display the rulebook."""
    print("\n" + "="*70)
    print("INTERSECTION RULEBOOK")
    print("="*70)
    print("\nPriority Order (Higher → Lower):")
    print("\nP0: Collision Avoidance (SAFETY)")
    print("    Measures: Will action cause collision?")
    print("    Scores: 0 = safe, 1 = collision")
    print("    Why P0: Safety is paramount, non-negotiable")
    
    print("\nP1: Traffic Law Compliance (LEGAL)")
    print("    Measures: Does action violate traffic laws?")
    print("    Scores: 0 = legal, 1+ = violations")
    print("    Why P1: Must follow laws (red lights, speed limits)")
    
    print("\nP2: Don't Block Intersection (COURTESY)")
    print("    Measures: Would action block cross-traffic?")
    print("    Scores: 0 = clear, 1 = blocking")
    print("    Why P2: Courtesy, traffic flow, avoid gridlock")
    
    print("\nP3: Forward Progress (PROGRESS)")
    print("    Measures: Distance traveled")
    print("    Scores: Negative distance (maximize)")
    print("    Why P3: Prefer moving over stopping if safe/legal")
    
    print("\nP4: Time Efficiency (EFFICIENCY)")
    print("    Measures: Duration of action")
    print("    Scores: Time in seconds")
    print("    Why P4: Prefer faster options, all else equal")
    
    print("\n" + "="*70)
    print("KEY PRINCIPLE: Higher priorities ALWAYS win!")
    print("P0 violation >> ANY amount of P1, P2, P3, P4 violations")
    print("="*70)
    
    input("\nPress Enter to continue...")


def main():
    """Main interactive session."""
    print("\n" + "="*70)
    print("INTERSECTION SCENARIOS - INTERACTIVE SESSION")
    print("="*70)
    print("\nToday we'll apply rule-based planning to real driving scenarios!")
    print("\nYou'll:")
    print("  1. Learn the rulebook (5 priorities)")
    print("  2. Analyze 4 different intersection scenarios")
    print("  3. Predict the best action")
    print("  4. See lexicographic ordering in action")
    
    input("\nPress Enter to start...")
    
    # Show rulebook
    show_rulebook()
    
    # Run scenarios
    scenarios = [
        (scenario_a_green_clear, "Scenario A: Green Clear"),
        (scenario_b_yellow_clear, "Scenario B: Yellow Light"),
        (scenario_c_red_clear, "Scenario C: Red Light"),
        (scenario_d_green_blocked, "Scenario D: Green Blocked"),
    ]
    
    for scenario_func, name in scenarios:
        run_scenario(scenario_func, name, visualize=True)
    
    # Summary
    print("\n" + "="*70)
    print("SESSION COMPLETE!")
    print("="*70)
    print("\nKey Takeaways:")
    print("  ✓ One rulebook handles all scenarios correctly")
    print("  ✓ Lexicographic ordering ensures right priorities")
    print("  ✓ Safety (P0) always comes first")
    print("  ✓ Green light doesn't always mean GO (Scenario D)")
    print("  ✓ Legal compliance (P1) beats efficiency (P4)")
    print("\nThis is how autonomous vehicles make decisions!")
    print("="*70)


if __name__ == "__main__":
    main()
