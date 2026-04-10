# Rule Evaluation - Exercise Workbook 📝

**For: Nidhi**  
**Hands-On Practice with Rule-Based Planning**

---

## 📚 How to Use This Workbook

1. Read each exercise carefully
2. Try to solve it yourself first
3. Check your answer against the solution
4. Run the code to see visualizations
5. Experiment with modifications

---

## 🎯 Exercise 1: Understanding Basic Rules

### **Part A: Calculate Scores by Hand**

Given this trajectory:
```
Points: [(0,0), (1,0), (2,1), (3,1), (4,0)]
```

**Calculate:**

1. **Path length**
   - Hint: Distance = √((x2-x1)² + (y2-y1)²)
   - Answer: ~4.82

2. **Maximum |y| value**
   - Hint: Look at all y values, take max absolute value
   - Answer: 1

3. **Number of points with y > 0.5**
   - Hint: Count how many points have y > 0.5
   - Answer: 2

<details>
<summary>Click for Solution</summary>

1. **Path length:**
   - (0,0)→(1,0): √1 = 1.0
   - (1,0)→(2,1): √2 = 1.41
   - (2,1)→(3,1): √1 = 1.0
   - (3,1)→(4,0): √2 = 1.41
   - **Total: 4.82**

2. **Maximum |y|:**
   - y values: [0, 0, 1, 1, 0]
   - **Max: 1**

3. **Points with y > 0.5:**
   - (2,1): yes
   - (3,1): yes
   - **Count: 2**
</details>

---

### **Part B: Write Your Own Rule**

Complete this function:

```python
def rule_count_turns(traj: Trajectory) -> float:
    """
    Count the number of direction changes in the trajectory.
    A turn is when the sign of dy changes.
    
    Returns:
        Number of turns (lower is smoother)
    """
    # YOUR CODE HERE
    turns = 0
    for i in range(len(dy)-1):
      if (dy[i] >= 0 and dy[i+1] =< 0) or (dy[i] =< 0 and dy[i+1] >= 0)
      turns += 1
   return turns
    
    # Hint: Compare consecutive dy values
    # If dy[i] and dy[i+1] have different signs, that's a turn!
    
    return turns
```

**Test cases:**
- Straight line (y always 0): Expected score = 0
- Single curve (y goes up then down): Expected score = 1
- Zigzag (up, down, up, down): Expected score = many!

<details>
<summary>Click for Solution</summary>

```python
def rule_count_turns(traj: Trajectory) -> float:
    turns = 0
    dy = np.diff(traj.y)  # Compute differences
    
    for i in range(len(dy) - 1):
        # Check if sign changes
        if dy[i] * dy[i+1] < 0:  # Different signs
            turns += 1
    
    return turns
```

**Alternative solution:**
```python
def rule_count_turns(traj: Trajectory) -> float:
    dy = np.diff(traj.y)
    sign_changes = np.sum(np.diff(np.sign(dy)) != 0)
    return sign_changes
```
</details>

---

## 🎯 Exercise 2: Comparing with One Rule

### **Scenario:**

You have three delivery routes:

**Route A (Highway):**
- Length: 15 km
- Smoothness score: 10 (very smooth)
- Time: 12 minutes

**Route B (City):**
- Length: 10 km
- Smoothness score: 50 (lots of stops)
- Time: 18 minutes

**Route C (Mixed):**
- Length: 12 km
- Smoothness score: 25 (moderate)
- Time: 15 minutes

### **Questions:**

1. **If you only care about LENGTH, which route wins?**
   - Answer: B

2. **If you only care about SMOOTHNESS, which route wins?**
   - Answer: A

3. **If you only care about TIME, which route wins?**
   - Answer: A

4. **Can one route be the "best" at everything?**
   - Answer: Not in this scenario because each route has different setbacks and there is no defining factor

<details>
<summary>Click for Solutions</summary>

1. **Length:** Route B (10 km) ✅
2. **Smoothness:** Route A (score 10) ✅
3. **Time:** Route A (12 min) ✅
4. **No!** Different routes are best at different things. This is why we need priorities!
</details>

---

## 🎯 Exercise 3: Lexicographic Comparison by Hand

### **Scenario:**

Three trajectories with two rules:

```
              P0 (Safety)  P1 (Length)
Trajectory A:     0            15
Trajectory B:     0            10
Trajectory C:     1            8
```

**Priority order:** Safety first, then Length

### **Questions:**

1. **Compare A and B at P0:**
   - Both have score 0 (tied)
   - Continue to P1?
   - Answer: Yes

2. **Compare A and B at P1:**
   - A: 15, B: 10
   - Who wins?
   - Answer: B

3. **Compare B and C:**
   - At P0: B has 0, C has 1
   - Who wins? (Check P1 or not?)
   - Answer: B

4. **Final ranking (best to worst):**
   - Answer: B, A, C (failed in terms of safety)

<details>
<summary>Click for Solutions</summary>

1. **A vs B at P0:** Tied (0 = 0), so yes, continue to P1 ✅

2. **A vs B at P1:** B wins (10 < 15) ✅

3. **B vs C:** B wins at P0 (0 < 1). Don't even check P1! Even though C is shorter (8 < 10), C loses because of safety violation. ✅

4. **Final ranking:**
   1. B (Safe and shortest) ✅
   2. A (Safe but longer)
   3. C (Unsafe, disqualified at P0)
</details>

---

## 🎯 Exercise 4: Does Priority Order Matter?

### **Setup:**

Two routes:
- **Route X:** Short (5 km) but bumpy (smoothness = 80)
- **Route Y:** Long (10 km) but smooth (smoothness = 10)

### **Scenario 1: Smoothness First**

Priority order: [Smoothness, Length]

**Question:** Which route wins? Route Y

<details>
<summary>Click for Solution</summary>

```
        P0 (Smooth)  P1 (Length)
Route X:    80           5
Route Y:    10          10

Compare P0: 10 < 80 → Route Y wins! ✅
```

Even though Y is twice as long, it wins because smoothness is P0!
</details>

---

### **Scenario 2: Length First**

Priority order: [Length, Smoothness]

**Question:** Which route wins? ROute X

<details>
<summary>Click for Solution</summary>

```
        P0 (Length)  P1 (Smooth)
Route X:     5          80
Route Y:    10          10

Compare P0: 5 < 10 → Route X wins! ✅
```

Now X wins because length is P0!
</details>

---

### **Conclusion Question:**

**Does priority order change the winner?**
- Answer: Yes, the winner was different as soon as priority order shifted

<details>
<summary>Click for Solution</summary>

**YES!** Priority order completely changes the winner:
- Smoothness first → Route Y wins
- Length first → Route X wins

**This is why defining priorities is so important!** 🎯
</details>

---

## 🎯 Exercise 5: Design a Rulebook

### **Scenario: Robot Vacuum Cleaner**

Your robot vacuum must:
- Avoid hitting furniture (CRITICAL)
- Cover all floor area (IMPORTANT)
- Minimize energy use (NICE TO HAVE)
- Minimize noise (NICE TO HAVE)

### **Questions:**

1. **Define 4 rules with appropriate names:**
   - R0 (highest priority): Avoide Hitting Furniture
   - R1: Cover all floor area
   - R2: Minimize energy use
   - R3: Minimize noise

2. **Why is collision avoidance P0?**
   - Answer: It is the most important thing to consider as a collison could mean breaking/damaging items

3. **Should energy or noise be P2?**
   - Answer: Probably energy as that impacts functionality more but also depends on user needs

<details>
<summary>Click for Solution</summary>

1. **Rules:**
   - **R0:** Collision avoidance (no furniture hits)
   - **R1:** Coverage (% of floor cleaned)
   - **R2:** Energy consumption (battery life)
   - **R3:** Noise level (quietness)

2. **Why collision is P0:**
   - Hitting furniture can damage the robot or furniture
   - Can knock over valuable items
   - Safety/damage prevention always comes first

3. **Energy vs Noise for P2:**
   - Either works! Depends on priorities:
   - Home with baby sleeping → Noise is P2
   - Normal home → Energy is P2 (longer runtime)
   - This is a design choice!
</details>

---

## 🎯 Exercise 6: Spot the Error

### **Setup:**

Someone implemented lexicographic ordering like this:

```python
def compare_trajectories_WRONG(traj1, traj2, rules):
    # Compute all scores
    scores1 = [rule(traj1) for rule in rules]
    scores2 = [rule(traj2) for rule in rules]
    
    # Sum all scores
    total1 = sum(scores1)
    total2 = sum(scores2)
    
    # Compare totals
    if total1 < total2:
        return traj1  # This is WRONG!
    else:
        return traj2
```

### **Questions:**

1. **What's wrong with this approach?**
   - Answer: It treats each rule as equal in priority

2. **Give an example where it fails:**
   ```
           P0  P1  P2
   Traj A:  10  0   0
   Traj B:   0  5   5
   
   Wrong code says: A=10, B=10 (tie)
   Correct answer: If it fails at p0, then the other trajectory should immediately win
   ```

3. **What's the correct way to compare?**
   - Answer: It should compare the scores at each point

<details>
<summary>Click for Solutions</summary>

1. **What's wrong:**
   - Summing scores treats all priorities equally!
   - P0 violations can be "canceled out" by good P1/P2 scores
   - This violates the whole point of priority hierarchy!

2. **Example failure:**
   ```
   Traj A: P0=10 → UNSAFE!
   Traj B: P0=0  → Safe
   
   Wrong code: 10 = 10 (tie)
   Correct: B wins (0 < 10 at P0)
   ```
   
   B should win immediately at P0, but wrong code says they're tied!

3. **Correct way:**
   ```python
   def compare_lexicographically(traj1, traj2, rules):
       scores1 = [rule(traj1) for rule in rules]
       scores2 = [rule(traj2) for rule in rules]
       
       # Compare priority by priority
       for s1, s2 in zip(scores1, scores2):
           if s1 < s2:
               return traj1  # Wins at this priority
           elif s1 > s2:
               return traj2  # Loses at this priority
           # If s1 == s2, continue to next priority
       
       return traj1  # All tied, arbitrary choice
   ```
   
   Or simply:
   ```python
   # Python tuples compare lexicographically!
   if tuple(scores1) < tuple(scores2):
       return traj1
   ```
</details>

---

## 🎯 Exercise 7: Real-World Application

### **Scenario: Intersection Decision**

Your car approaches an intersection. Three options:

**Option 1: Go Straight**
- P0 (Collision): 0 (safe)
- P1 (Legal): 1 (red light violation!)
- P2 (Time): 5 seconds

**Option 2: Turn Right**
- P0 (Collision): 0 (safe)
- P1 (Legal): 0 (legal)
- P2 (Time): 8 seconds

**Option 3: Stop**
- P0 (Collision): 0 (safe)
- P1 (Legal): 0 (legal)
- P2 (Time): ∞ (wait forever)

### **Questions:**

1. **Compare Options 1 and 2:**
   - P0: Tied (both 0)
   - P1: Option 2 wins (0 < 1)
   - Check P2? No because 2 wins at P1

2. **Compare Options 2 and 3:**
   - P0: Tied (both 0)
   - P1: Tied (both 0)
   - P2: Option 2 wins (8 < ∞)
   - Winner? Option 2 wins

3. **Final ranking:**
   - 1st: 1
   - 2nd: 3
   - 3rd: 2

4. **Why doesn't Option 1 win even though it's fastest?**
   - Answer: _______

<details>
<summary>Click for Solutions</summary>

1. **Options 1 vs 2:**
   - P0: Tied
   - P1: 2 wins (0 < 1) ✅
   - Don't check P2! Winner decided at P1.

2. **Options 2 vs 3:**
   - P0: Tied
   - P1: Tied
   - P2: 2 wins (8 < ∞) ✅

3. **Final ranking:**
   1. Option 2 (Turn Right) ✅
   2. Option 3 (Stop)
   3. Option 1 (Go Straight) ❌ ILLEGAL

4. **Why Option 1 loses:**
   - Even though it's fastest (5 seconds), it's ILLEGAL (P1 violation)
   - Legal compliance (P1) beats efficiency (P2)
   - Would you run a red light to save 3 seconds? NO! 🚦
</details>

---

## 🎯 Exercise 8: Create Your Own Scenario

### **Your Task:**

Design a scenario with:
- 3 trajectories/options
- 3 rules in priority order
- Interesting trade-offs (no option is best at everything)
- Clear winner using lexicographic ordering

### **Template:**

```
Scenario: School Hallway Delivery Robot 

Options:
1. Main Hallway: [P0=1, P1=0, P2= 4 min]
2. Quad Route: [P0=0, P1=1, P2= 6 min]
3. Music Hallway: [P0=0, P1=0, P2= 5 min]

Rules (in order):
P0: Avoid collision with students
P1: Avoid uneven surfaces
P2: Minimize time

Winner: Music Hallway
Why: The robot will have a lower chance of collision. routes are eliminated in p1 and 2
```

**Example solution:**

```
Scenario: Drone Delivery

Options:
1. Fast route: [P0=1 (near airport!), P1=0, P2=5min]
2. Safe route: [P0=0, P1=0, P2=10min]
3. Scenic route: [P0=0, P1=1 (over houses), P2=8min]

Rules:
P0: Avoid restricted airspace (collision/legal)
P1: Avoid flying over buildings
P2: Minimize time

Winner: Safe route
Why: 
- Fast route fails at P0 (restricted airspace)
- Scenic fails at P1 (over buildings)
- Safe route wins (safe and legal, even if slower)
```

---

## ✅ Summary Checklist

After completing these exercises, you should be able to:

- [ ] Calculate rule scores by hand
- [ ] Write your own rule functions
- [ ] Compare trajectories with single rules
- [ ] Perform lexicographic comparison
- [ ] Understand why priority order matters
- [ ] Design appropriate rulebooks
- [ ] Spot errors in implementations
- [ ] Apply to real-world scenarios

---

## 🚀 Next Steps

1. Complete all exercises above
2. Run the interactive tutorial (`rule_evaluation_tutorial.py`)
3. Experiment with your own rules and scenarios
4. Apply to intersection scenario (CARLA)

**You're ready to build rule-based planners!** 🎓✨
