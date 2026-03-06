# Rule Evaluation Tutorial - Complete Guide 📊

**Understanding Rule-Based Planning Through Visualization**

---

## 🎯 Learning Objectives

By the end of this tutorial, you'll understand:

1. **What rules are** - Functions that score trajectories
2. **How to evaluate** - Computing scores for single trajectories
3. **How to compare** - Ranking multiple options
4. **Lexicographic ordering** - Comparing with multiple rules
5. **Why priorities matter** - Higher priorities always win

---

## 📚 Core Concepts

### **Concept 1: Rules Are Scoring Functions**

A **rule** is just a function that takes a trajectory and returns a number (score).

```python
def rule_path_length(trajectory):
    """Returns the total length of the path."""
    return calculate_length(trajectory)
```

**Key insight:** Lower scores = better!

---

### **Concept 2: Single Rule Evaluation**

With one rule, comparison is simple:

```
Trajectory A: Score = 10.5
Trajectory B: Score = 8.2
Trajectory C: Score = 12.1

Winner: B (lowest score) ✅
```

**Example: Path Length**
```
Straight line:   10.0 m  ← Best!
Curved path:     12.5 m
Zigzag path:     15.3 m
```

---

### **Concept 3: Multiple Rules (Conflict!)**

With multiple rules, trajectories might be best at different things:

```
              Rule 1        Rule 2
              (Length)   (Smoothness)
Trajectory A:   10.0         50.0
Trajectory B:   12.0         5.0   ← Smoother but longer
Trajectory C:   15.0         100.0
```

**Question:** Which is better, A or B?
- A is shorter (better at Rule 1)
- B is smoother (better at Rule 2)

**We need priorities!**

---

### **Concept 4: Lexicographic Ordering**

**Definition:** Compare priorities one by one, like alphabetical order for words.

**How it works:**

```
Priority 0: Most important
Priority 1: Second most important
Priority 2: Third most important
...
```

**Comparison process:**
1. Compare Priority 0 scores
2. If tied, compare Priority 1 scores
3. If tied, compare Priority 2 scores
4. Continue until a winner emerges

**Like alphabetical order:**
```
"apple" vs "banana"
→ Compare first letter: a < b
→ "apple" wins! (don't need to check other letters)

"apple" vs "apply"
→ First letter: a = a (tied)
→ Second letter: p = p (tied)
→ Third letter: p = p (tied)
→ Fourth letter: l < l (tied)
→ Fifth letter: e < y
→ "apple" wins!
```

---

### **Concept 5: Priority Hierarchy**

**Example: Autonomous Driving**

```
Priority 0: Safety (avoid collision)
    ↓ If tied...
Priority 1: Legality (follow traffic laws)
    ↓ If tied...
Priority 2: Comfort (smooth driving)
    ↓ If tied...
Priority 3: Efficiency (minimize time)
```

**Why this order?**
- Safety is INFINITELY more important than speed
- Would you rather arrive 2 minutes late or crash? 🚗💥

---

## 🔍 Detailed Examples

### **Example 1: Single Rule**

**Rule:** Minimize Path Length

**Trajectories:**
```
Straight:  ━━━━━━━━━━━━  (10.0 m)
Curved:    ╭──────╮      (12.5 m)
Zigzag:    /\/\/\/\      (15.3 m)
```

**Scores:**
```
Straight: 10.0  ← BEST ✅
Curved:   12.5
Zigzag:   15.3
```

**Winner:** Straight (shortest path)

---

### **Example 2: Two Rules (Lexicographic)**

**Rules in order:**
1. **P0:** Minimize Smoothness (jerk)
2. **P1:** Minimize Length

**Trajectories and Scores:**
```
               P0 (Smooth)  P1 (Length)
Straight:          5.0         10.0
Curved:            8.0         12.5
Zigzag:          100.0         15.3
```

**Comparison:**
```
Round 1 - Compare P0 (Smoothness):
  Straight: 5.0   ← BEST at P0
  Curved:   8.0
  Zigzag:   100.0

Winner: Straight ✅ (wins at P0, don't even check P1!)
```

**Key Point:** Zigzag is shortest (15.3 m) but loses because it's so wiggly (100.0 jerk)!

---

### **Example 3: Priority Order Matters**

**Two trajectories:**
- **A:** Short (8m) but wiggly (jerk = 80)
- **B:** Long (12m) but smooth (jerk = 10)

**Scenario 1: Length First**
```
Priority 0: Length
Priority 1: Smoothness

        P0 (Length)  P1 (Smooth)
A:          8.0         80.0
B:         12.0         10.0

Compare P0: 8.0 < 12.0 → A wins! ✅
```

**Scenario 2: Smoothness First**
```
Priority 0: Smoothness
Priority 1: Length

        P0 (Smooth)  P1 (Length)
A:         80.0          8.0
B:         10.0         12.0

Compare P0: 10.0 < 80.0 → B wins! ✅
```

**Conclusion:** Priority order determines the winner!

---

## 🎓 Understanding Lexicographic Ordering

### **Formal Definition**

Given two trajectories with score vectors:
- Trajectory 1: `[r0_1, r1_1, r2_1, ...]`
- Trajectory 2: `[r0_2, r1_2, r2_2, ...]`

**Trajectory 1 is better than Trajectory 2 if:**

```python
if r0_1 < r0_2:
    return Trajectory1  # Wins at P0
elif r0_1 > r0_2:
    return Trajectory2  # Loses at P0
else:  # r0_1 == r0_2 (tied at P0)
    if r1_1 < r1_2:
        return Trajectory1  # Wins at P1
    elif r1_1 > r1_2:
        return Trajectory2  # Loses at P1
    else:  # Tied at P1, continue...
        # Keep comparing until tie is broken
```

**In Python (compact):**
```python
# Compare tuples directly!
score1 = (r0_1, r1_1, r2_1)
score2 = (r0_2, r1_2, r2_2)

if score1 < score2:
    winner = Trajectory1  # Python does lexicographic comparison!
```

---

## 📊 Visualization Guide

### **What Each Plot Shows**

#### **Plot 1: Trajectories**
- Shows all paths in 2D (x, y)
- Different colors for different trajectories
- Centerline at y=0 (dashed gray)

#### **Plot 2: Score Comparison**
- Horizontal bar chart
- Longer bar = higher (worse) score
- Gold border = winner (shortest bar)

#### **Plot 3: Score Matrix (Heatmap)**
- Rows = Rules (P0, P1, P2, ...)
- Columns = Trajectories
- Green = Low (good), Red = High (bad)

#### **Plot 4: Individual Rule Scores**
- One subplot per rule
- Shows which trajectory is best for each rule
- Helps understand trade-offs

---

## 🛠️ Using the Tutorial

### **Running the Interactive Tutorial**

```bash
python rule_evaluation_tutorial.py
```

**You'll see a menu:**
```
MENU
====================================
Demonstrations:
  1. Single Rule Evaluation
  2. Lexicographic Ordering
  3. Priority Order Matters

Exercises:
  4. Exercise 1: Define Your Own Rule
  5. Exercise 2: Compare Two Rules  
  6. Exercise 3: Design a Rulebook

  0. Exit
```

---

### **Recommended Learning Path**

**Step 1: Demo 1** (Single Rule)
- See how one rule works
- Understand scoring
- See visualization

**Step 2: Exercise 1** (Define Your Own Rule)
- Practice writing a rule function
- Test it on trajectories

**Step 3: Demo 2** (Lexicographic Ordering)
- See multiple rules in action
- Understand priority hierarchy
- Watch comparison process

**Step 4: Demo 3** (Priority Matters)
- See same trajectories, different orders
- Different winners!
- Understand importance of priorities

**Step 5: Exercise 3** (Design Rulebook)
- Apply knowledge to real scenario
- Think about what should come first

---

## 💡 Key Insights

### **Insight 1: Rules Are Just Math**

A rule is just:
```python
trajectory → number
```

Lower number = better trajectory

**Examples:**
- Path length: `trajectory.length`
- Smoothness: `sum of jerk`
- Safety: `number of violations`

---

### **Insight 2: Priorities Are Non-Negotiable**

Higher priority violations are **infinitely worse** than lower priority violations.

**Example:**
```
Trajectory A: P0=1, P1=0, P2=0    (collision!)
Trajectory B: P0=0, P1=100, P2=100 (legal but messy)

Winner: B

Why? P0 (safety) violation in A is worse than ANY amount of P1/P2 violations!
```

---

### **Insight 3: This Matches Human Reasoning**

**Real-world example:**
```
You're driving and see:
1. Red light ahead
2. Route that's 5 minutes faster
3. Smoother road on the left

Priority order:
P0: Stop for red light (LEGAL) ← MUST DO
P1: Take faster route (EFFICIENT) ← Nice to have
P2: Take smoother road (COMFORT) ← Nice to have

You stop for the red light, even though the other route is faster!
```

---

## 🎯 Common Questions

### **Q1: Why lower scores are better?**

**A:** Convention! We could use "higher is better" but then we'd say "maximize" instead of "minimize". Either works, just pick one and be consistent.

---

### **Q2: What if all priorities tie?**

**A:** Then the trajectories are considered **equivalent**. Pick either one (or randomly).

---

### **Q3: Can we have weighted priorities instead?**

**A:** Yes, but you lose the strict hierarchy! With weights:
```
score = w0*r0 + w1*r1 + w2*r2

Problem: Even huge r0 can be beaten by huge r2 if w2 is big!
```

With lexicographic ordering:
```
NO amount of r1, r2, r3, ... can overcome even a tiny r0 difference!
```

---

### **Q4: How many priorities should we have?**

**A:** As many as needed, but:
- 1-3 priorities: Simple, clear
- 4-6 priorities: Manageable
- 7+ priorities: Getting complex, rarely needed

**Most autonomous driving systems:** 3-5 priorities

---

## 🧪 Exercise Solutions

### **Exercise 1 Solution**

```python
def my_custom_rule(traj: Trajectory) -> float:
    """Penalize trajectories above y=1.0"""
    violations = 0
    for y_val in traj.y:
        if y_val > 1.0:
            violations += 1
    return violations

# Or more concisely:
def my_custom_rule(traj: Trajectory) -> float:
    return np.sum(traj.y > 1.0)
```

**Test results:**
- Straight (y=0 always): 0 violations ✅
- Small curve (max y≈0.5): 0 violations ✅
- Large curve (max y≈2.0): ~25 violations ❌

---

### **Exercise 2 Solution**

**Answer:** YES, priority order matters!

**Order 1 (Length first):** Short wiggly wins
**Order 2 (Smoothness first):** Long smooth wins

**Why?** First priority dominates the decision!

---

### **Exercise 3 Solution**

**Correct answer:** A. [Collision, Lane, Fuel]

**Why?**
- Safety (collision) MUST come first
- Lane keeping is important but not life-critical
- Fuel efficiency is nice but least important

**Wrong answers:**
- B: Fuel first? No! Would sacrifice safety for fuel savings! ❌
- C: Lane before collision? No! Better to leave lane than crash! ❌

---

## 📈 Advanced Topics

### **Continuous vs Discrete Scores**

**Discrete (counted):**
```python
violations = count_rule_violations(trajectory)
# Returns: 0, 1, 2, 3, ...
```

**Continuous (measured):**
```python
deviation = max_lateral_deviation(trajectory)
# Returns: 0.0, 1.5, 2.3, ...
```

**Both work!** Just be consistent.

---

### **Normalized vs Raw Scores**

**Raw scores:**
```python
length = 10.5  # meters
jerk = 123.7   # m/s³
```

**Normalized scores:**
```python
length_score = length / max_length  # 0.0 to 1.0
jerk_score = jerk / max_jerk       # 0.0 to 1.0
```

**When to normalize?**
- Different units (m vs m/s³)
- Vastly different scales (0.1 vs 10000)
- Easier to interpret

**But:** Not required for lexicographic ordering!

---

## ✅ Summary

**What you learned:**

1. ✅ Rules are scoring functions (trajectory → number)
2. ✅ Lower scores = better trajectories
3. ✅ Single rule = simple comparison
4. ✅ Multiple rules = need priorities
5. ✅ Lexicographic ordering = compare priority by priority
6. ✅ Higher priorities ALWAYS win
7. ✅ Priority order determines the winner
8. ✅ This matches human decision-making

**Key formula:**
```
Trajectory A beats Trajectory B if:
  A's score vector < B's score vector (lexicographically)
```

**Key principle:**
```
P0 violation >> ANY amount of P1, P2, P3, ... violations
```

---

## 🚀 Next Steps

1. ✅ Run all demos
2. ✅ Complete all exercises
3. ✅ Understand lexicographic ordering
4. ⏭️ **Next:** Apply to intersection scenario (4 traffic lights)
5. ⏭️ **Then:** Implement in CARLA simulator

---

**You now understand the mathematical foundation of rule-based planning!** 🎓📊

**This is how autonomous vehicles make principled decisions!** 🚗✨
