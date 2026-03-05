# Intersection Scenario Demo - Complete Guide 🚦

**Rule-Based Planning in Action**

---

## 🎯 What This Demo Does

This demo shows **rule-based planning** at a traffic intersection with **four different scenarios**:

- **Scenario A:** Green light, clear → Vehicle goes straight ✅
- **Scenario B:** Yellow light → Vehicle decides stop or go
- **Scenario C:** Red light → Vehicle turns right (only legal option) ✅
- **Scenario D:** Green light BUT blocked → Vehicle doesn't block intersection ✅

**Key Feature:** The rulebook automatically picks the best action based on **priorities**, not just speed!

---

## 🚀 Quick Start

### **Run the Demo:**

```bash
# Terminal 1: Start CARLA
cd ~/CARLA_0.9.13
./CarlaUE4.sh

# Terminal 2: Run intersection demo
cd ~/path/to/files
python intersection_scenario_demo.py

# Choose scenario when prompted:
# A, B, C, or D
```

---

## 📋 The Four Scenarios

### **Scenario A: Green Light, Clear** 🟢

```
        ↑
        │
    ────┼──── 🟢 Green light
        │
      • You (25m away)
```

**Context:**
- Traffic light: GREEN
- Intersection: CLEAR
- No obstacles

**Trajectories Generated:**
1. ⛔ STOP - Duration: 3s
2. ➡️ STRAIGHT (slow) - Duration: 5s
3. ➡️ STRAIGHT (normal) - Duration: 4.5s
4. ↪️ RIGHT - Duration: 6s
5. ↩️ LEFT - Duration: 7s

**Rulebook Evaluation:**

| Action | P0: Collision | P1: Legal | P2: Block | P3: Progress | P4: Time | Winner? |
|--------|---------------|-----------|-----------|--------------|----------|---------|
| STOP | 0 | 0 | 0 | 0m | ∞ | ❌ |
| STRAIGHT | 0 | 0 | 0 | 40m | 4.5s | ✅ **BEST** |
| RIGHT | 0 | 0 | 0 | 25m | 6s | ❌ |
| LEFT | 0 | 0 | 0 | 30m | 7s | ❌ |

**Decision:** GO STRAIGHT (fastest legal option) ✅

**Why:**
- P0-P2: All tied (all safe, legal, not blocking)
- P3: STRAIGHT has best progress (40m > 30m > 25m > 0m)
- Winner: STRAIGHT ✅

---

### **Scenario B: Yellow Light** 🟡

```
        ↑
        │
    ────┼──── 🟡 Yellow light
        │
      • You (25m away)
```

**Context:**
- Traffic light: YELLOW
- Distance: 25m
- Current speed: 6 m/s

**The Dilemma:**

```
Can we stop safely?
Stopping distance = v² / (2a) = 6² / (2×2) = 9m

9m < 25m → YES, we can stop safely!
```

**Trajectories Generated:**
1. ⛔ STOP
2. ➡️ STRAIGHT
3. ↪️ RIGHT
4. ↩️ LEFT

**Rulebook Evaluation:**

| Action | P0 | P1 | P2 | P3 | P4 |
|--------|----|----|----|----|----| 
| STOP | 0 | 0 | 0 | 23m | 3s |
| STRAIGHT | 0 | 0 | 0 | 40m | 4.5s |
| RIGHT | 0 | 0 | 0 | 25m | 6s |
| LEFT | 0 | 0 | 0 | 30m | 7s |

**Decision:** Usually STOP (conservative) ⚠️

**Why:**
- Both STOP and STRAIGHT are legal (P1 = 0)
- Since can stop safely, conservative approach is STOP
- Real autonomous vehicles would stop here

---

### **Scenario C: Red Light** 🔴

```
        ↑
        │
    ────┼──── 🔴 Red light
        │
      • You (25m away)
```

**Context:**
- Traffic light: RED
- Must obey traffic law!

**Trajectories Generated:**
1. ⛔ STOP
2. ➡️ STRAIGHT (will be illegal!)
3. ↪️ RIGHT (legal on red in most places)

**Rulebook Evaluation:**

| Action | P0 | P1: Legal? | P2 | P3 | P4 | Winner? |
|--------|----|----|----|----|----|---------| 
| STOP | 0 | 0 | 0 | 23m | ∞ | ❌ |
| STRAIGHT | 0 | **1 ❌** | 0 | 40m | 4.5s | ❌ **ILLEGAL!** |
| RIGHT | 0 | 0 ✅ | 0 | 25m | 6s | ✅ **BEST** |
| LEFT | 0 | **1 ❌** | 0 | 30m | 7s | ❌ **ILLEGAL!** |

**Decision:** TURN RIGHT (only legal moving option) ✅

**Why:**
- P1: STRAIGHT and LEFT are ILLEGAL (violation = 1)
- Between STOP and RIGHT (both legal):
  - P3: RIGHT has better progress (25m > 23m)
  - Winner: RIGHT ✅

**Key Insight:** Even though STRAIGHT is faster (P4 = 4.5s < 6s), it's **illegal** so it loses at P1!

---

### **Scenario D: Green Light, But BLOCKED** 🟢🚗🚗

```
        ↑
    🚗🚗 Traffic jam!
    ────┼──── 🟢 Green light
        │
      • You (25m away)
```

**Context:**
- Traffic light: GREEN (legal to go)
- Intersection: BLOCKED by traffic jam
- "Don't block the box" rule applies!

**Trajectories Generated:**
1. ⛔ STOP
2. ➡️ STRAIGHT (would block intersection!)
3. ↪️ RIGHT
4. ↩️ LEFT (would block too!)

**Rulebook Evaluation:**

| Action | P0 | P1 | P2: Block? | P3 | P4 | Winner? |
|--------|----|----|----|----|----|---------| 
| STOP | 0 | 0 | 0 ✅ | 23m | ∞ | ❌ |
| STRAIGHT | 0 | 0 | **1 ❌** | 40m | 4.5s | ❌ **BLOCKS!** |
| RIGHT | 0 | 0 | 0 ✅ | 25m | 6s | ✅ **BEST** |
| LEFT | 0 | 0 | **1 ❌** | 30m | 7s | ❌ **BLOCKS!** |

**Decision:** TURN RIGHT (doesn't block, makes progress) ✅

**Why:**
- P0-P1: All safe and legal (green light)
- P2: STRAIGHT and LEFT would block intersection (violation = 1)
- Between STOP and RIGHT (both don't block):
  - P3: RIGHT has better progress (25m > 23m)
  - Winner: RIGHT ✅

**Key Insight:** Even with green light, we DON'T go straight because it would block the box! Courtesy (P2) beats efficiency (P4)! 🚦

---

## 🎓 Understanding the Rulebook

### **The Hierarchy** (Most Important → Least Important)

```
P0: COLLISION (Safety)
    ↓ If tied...
P1: LEGAL (Traffic Laws)
    ↓ If tied...
P2: BLOCKING (Courtesy)
    ↓ If tied...
P3: PROGRESS (Distance)
    ↓ If tied...
P4: EFFICIENCY (Time)
```

### **How Comparison Works (Lexicographic Ordering)**

Think of it like alphabetical order for words:

```
"apple" vs "banana"
Compare first letter: a < b
Winner: "apple" (don't need to check other letters!)

Trajectory 1: [0, 1, 0, 25, 5]
Trajectory 2: [0, 0, 0, 30, 3]
             P0 P1 P2 P3 P4

Compare P0: 0 = 0 (tied, continue)
Compare P1: 1 > 0 (Trajectory 2 wins!)
Don't even look at P2, P3, P4!
```

**Rule:** Higher priority violations are infinitely worse than lower priority violations!

---

## 📊 What You'll See in CARLA

### **Traffic Light Visualization:**

```
🔴 Large red sphere = Red light
🟡 Large yellow sphere = Yellow light
🟢 Large green sphere = Green light

(Mounted on gray post)
```

### **Trajectory Visualization:**

```
Small colored dots (candidates):
  🔴 Red dots = STOP trajectory
  🟢 Green dots = STRAIGHT trajectory
  🔵 Blue dots = RIGHT trajectory
  🟡 Yellow dots = LEFT trajectory

Larger bright dots (selected):
  Whichever action was chosen
  
⚪ White arrow = Points to selected
```

### **Obstacle Markers (Scenario D):**

```
🔴 Red spheres = Blocking vehicles in intersection
```

---

## 📖 Example Output

### **Running Scenario D:**

```
==========================================================
INTERSECTION SCENARIO - RULE-BASED PLANNING
==========================================================

[STEP 1] Connecting to CARLA...
✓ Connected (Map: Town01)

[STEP 2] Spawning vehicle...
✓ Vehicle spawned

[STEP 3] Setting up chase camera...
✓ Camera positioned

[STEP 4] Select intersection scenario:
  A: Green light, clear
  B: Yellow light
  C: Red light
  D: Green light, but intersection BLOCKED

Enter scenario (A-D) [default=D]: D

✓ Scenario D: Green light, but intersection blocked!

[STEP 5] Creating traffic light...
✓ Traffic light: GREEN
✓ 2 obstacles blocking intersection

[STEP 6] Generating candidate trajectories...
  Generated: STOP
  Generated: STRAIGHT (v=5.4 m/s)
  Generated: STRAIGHT (v=6.0 m/s)
  Generated: RIGHT
  Generated: LEFT
✓ Generated 5 candidate trajectories

[STEP 7] Evaluating trajectories with rulebook...

==========================================================
RULEBOOK DECISION
==========================================================
Traffic Light: GREEN
Distance to Intersection: 25.0m
Blocked Ahead: True

Selected Action: RIGHT
Duration: 6.3s

Rulebook Evaluation:
  P0 (Collision):  0.0 ✓ Safe
  P1 (Legal):      0.0 ✓ Legal
  P2 (Blocking):   0.0 ✓ Clear
  P3 (Progress):   25.3m
  P4 (Efficiency): 6.3s

Other Options:
  STOP    : P0=0 P1=0 P2=0 P3=23m P4=∞s ❌
  STRAIGHT: P0=0 P1=0 P2=1 P3=40m P4=5s ❌
  LEFT    : P0=0 P1=0 P2=1 P3=30m P4=7s ❌
==========================================================

[STEP 8] Visualizing trajectories...
✓ Trajectories drawn

Look at CARLA:
  🔴 Red dots = STOP
  🟢 Green dots = STRAIGHT
  🔵 Blue dots = RIGHT
  🟡 Yellow dots = LEFT
  ⚪ White arrow = SELECTED action

==========================================================
SCENARIO SUMMARY
==========================================================
Scenario: D - Green light, but intersection blocked!
Traffic Light: GREEN
Selected Action: RIGHT

Why this decision?
  Green light BUT intersection blocked
  Don't block the box (P2: Courtesy rule)
  RIGHT turn or STOP are better options ✓

Visualization will remain for 30 seconds...
Press Ctrl+C to exit
==========================================================
```

---

## 🔧 How to Modify

### **Change Initial Speed:**

```python
# Line ~560 in main():
v0 = 6.0  # Current: 6 m/s (22 km/h)

# Slower (safer):
v0 = 4.0  # 4 m/s (14 km/h)

# Faster:
v0 = 8.0  # 8 m/s (29 km/h)
```

---

### **Change Distance to Intersection:**

```python
# Line ~470 in get_scenario_context():
distance_to_intersection = 25.0  # Current

# Closer (harder to stop on yellow):
distance_to_intersection = 15.0

# Further (easier to stop):
distance_to_intersection = 40.0
```

**Effect on Scenario B:** Changes whether we can safely stop!

---

### **Add More Trajectory Options:**

```python
# Line ~600 in main(), add:

# More straight options with different speeds
for v_mult in [0.7, 0.8, 0.9, 1.0, 1.1]:
    straight_traj = generate_straight_trajectory(
        s0, v0, context['distance_to_intersection'], v0 * v_mult
    )
    trajectories.append(straight_traj)
```

**Effect:** More options to choose from (better optimization)

---

### **Modify Rulebook Weights:**

Want to make blocking worse? Or progress more important?

```python
# In evaluate_trajectory() function:

# Current P2 (blocking):
r2 = check_blocking(trajectory, context)  # Returns 0 or 1

# Make blocking MUCH worse:
r2 = check_blocking(trajectory, context) * 10  # 0 or 10!

# Current P3 (progress):
r3 = -trajectory['final_s']

# Make progress more important:
r3 = -trajectory['final_s'] * 2
```

**Warning:** This changes the rulebook philosophy! Only do this if you understand the implications.

---

## 🎯 Learning Exercises

### **Exercise 1: Yellow Light Decision Point**

**Goal:** Find the distance where decision changes from GO to STOP

**Steps:**
1. Run Scenario B with distance = 25m → Should STOP
2. Change distance to 15m → Should GO
3. Find the exact transition point

**Question:** At what distance does it switch?

**Answer:** Around stopping_distance = v²/(2a) ≈ 18m

---

### **Exercise 2: Add "WAIT" Action**

**Goal:** Add a new action that waits for traffic to clear

**Steps:**
1. Create `generate_wait_trajectory()` function
2. Stays stopped for 10 seconds
3. Then proceeds straight

**Question:** When would this be selected in Scenario D?

**Answer:** If RIGHT turn wasn't an option!

---

### **Exercise 3: Multiple Obstacles**

**Goal:** Test with more blocking vehicles

**Steps:**
1. In Scenario D context, add more obstacles:
```python
'obstacles': [
    {'s': distance_to_intersection + 2, 'd': 0.0},
    {'s': distance_to_intersection + 7, 'd': 0.0},
    {'s': distance_to_intersection + 12, 'd': 0.0},  # New!
    {'s': distance_to_intersection + 17, 'd': 0.0},  # New!
]
```

**Question:** Does the decision change?

**Answer:** No! Still RIGHT, but P2 violations for STRAIGHT/LEFT are higher.

---

## 🐛 Troubleshooting

### **Issue 1: No Trajectories Visible**

**Cause:** Drawing at wrong height

**Fix:**
```python
# In visualize_trajectories(), change z values:
z=2.0  # Try z=1.0 or z=3.0
```

---

### **Issue 2: All Actions Seem Illegal**

**Cause:** Traffic light logic too strict

**Check:**
```python
# In check_traffic_law():
print(f"Action: {action}, Light: {traffic_light}")
```

**Fix:** Make sure light state is correctly passed

---

### **Issue 3: Crash When Generating Trajectories**

**Cause:** Division by zero (v0 = 0)

**Fix:**
```python
# Always start with some velocity:
v0 = max(v0, 0.1)  # At least 0.1 m/s
```

---

## 📚 Key Concepts Summary

### **1. Rule-Based Planning**
- Not just "pick fastest"
- Hierarchical priorities
- Safety > Legality > Courtesy > Progress > Efficiency

### **2. Lexicographic Ordering**
- Compare priority-by-priority
- Higher priority ALWAYS wins
- Like alphabetical order for lists

### **3. Context-Aware**
- Same intersection, different scenarios
- Different best actions
- Rulebook adapts automatically

### **4. Multi-Objective**
- Can't satisfy all goals
- Must trade off
- Rulebook provides clear ranking

---

## 🎓 What Nidhi Learns

After running all four scenarios, Nidhi understands:

✅ **Why rules matter** - Can't just optimize speed  
✅ **How priorities work** - Some constraints are more important  
✅ **Lexicographic ordering** - How to compare properly  
✅ **Real-world complexity** - Driving requires balancing many goals  
✅ **Formal methods** - Mathematical approach to decision-making  

---

## 🚀 Next Steps

1. ✅ Run all four scenarios
2. ✅ Observe how decisions change
3. ✅ Modify parameters and re-run
4. ✅ Try the exercises
5. ⏭️ **Next:** Add more maneuvers (U-turn, merge, etc.)

---

## 📊 Comparison with Highway Scenario

| Feature | Highway | Intersection |
|---------|---------|--------------|
| **Maneuvers** | Lane changes | Stop/straight/turn |
| **Selection** | Could use any | MUST use rulebook |
| **Scenarios** | One (normal driving) | Four (different traffic) |
| **Complexity** | Simple (2-3 rules) | Complex (5 rules) |
| **Learning** | How sampling works | Why rules matter |
| **Real-world** | Common | Challenging |

---

## ✅ Complete Checklist

Before moving on, make sure you can:

- [ ] Run all four scenarios successfully
- [ ] Understand why each decision was made
- [ ] Explain lexicographic ordering to someone
- [ ] Modify initial speed and see different results
- [ ] Read and understand the rulebook output
- [ ] Explain why green light doesn't always mean "go"

---

**You now have a complete rule-based planning system for intersections!** 🚦✅

**Run it, experiment with it, and see how formal methods make autonomous driving decisions!** 🚗🎓

---

## 🎯 Quick Reference

```bash
# Run demo
python intersection_scenario_demo.py

# Scenarios:
A = Green, clear → STRAIGHT
B = Yellow → STOP/GO (depends)
C = Red → RIGHT (only legal)
D = Green, blocked → RIGHT (don't block)

# Rulebook:
P0: Collision (safety)
P1: Legal (traffic law)
P2: Blocking (courtesy)
P3: Progress (distance)
P4: Efficiency (time)
```

**Perfect for teaching rule-based autonomous driving!** 🚗✨
