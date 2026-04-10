# Today's Lesson: Rule Evaluation & Visualization 📊

**For: Ahmad & Nidhi**  
**Focus: Understanding rules through Python visualizations**

---

## 🎯 Today's Goals

By the end of today's session, Nidhi will:
1. ✅ Understand what rules are (scoring functions)
2. ✅ See how to evaluate trajectories with single rules
3. ✅ Understand lexicographic ordering (total order)
4. ✅ Visualize everything with Python figures
5. ✅ Complete hands-on exercises

**No CARLA today!** Just Python + matplotlib visualizations ✨

---

## 📁 Files for Today

**Main interactive tutorial:**
- `rule_evaluation_tutorial.py` - Interactive menu-driven program

**Reference guides:**
- `RULE_EVALUATION_GUIDE.md` - Complete concept guide
- `RULE_EVALUATION_EXERCISES.md` - Exercise workbook

---

## ⏰ Session Plan (60-90 minutes)

### **Part 1: Introduction (10 min)**

**Explain the concept:**
```
Rule = Function that scores a trajectory
trajectory → number (score)
Lower score = better trajectory
```

**Example:**
```python
def rule_path_length(trajectory):
    return total_length(trajectory)

# Trajectory A: 10.0 m
# Trajectory B: 12.5 m
# Winner: A (shorter)
```

**Run the tutorial:**
```bash
python rule_evaluation_tutorial.py
# Choose: 1 (Single Rule Evaluation)
```

**Show Nidhi:**
- Three trajectories (straight, curved, zigzag)
- Scores for path length
- Bar chart comparison
- Winner highlighted in gold

**Key point:** "See how we can compare trajectories by just computing a number?"

---

### **Part 2: Single Rule Practice (15 min)**

**Exercise 1 (from workbook):**

Have Nidhi calculate scores by hand:
```
Trajectory: [(0,0), (1,0), (2,1), (3,1), (4,0)]

Calculate:
1. Path length = ?
2. Max |y| = ?
3. Points with y > 0.5 = ?
```

**Then verify in Python:**
```python
import numpy as np

x = np.array([0, 1, 2, 3, 4])
y = np.array([0, 0, 1, 1, 0])

# Path length
dx = np.diff(x)
dy = np.diff(y)
length = np.sum(np.sqrt(dx**2 + dy**2))
print(f"Length: {length:.2f}")

# Max |y|
max_y = np.max(np.abs(y))
print(f"Max |y|: {max_y}")

# Count y > 0.5
count = np.sum(y > 0.5)
print(f"Points with y > 0.5: {count}")
```

**Key point:** "Rules are just math! We can compute them by hand or with code."

---

### **Part 3: Writing Custom Rules (15 min)**

**Challenge Nidhi to write a rule:**

```python
def rule_lateral_deviation(traj: Trajectory) -> float:
    """
    Penalize trajectories that deviate from centerline.
    
    Goal: Minimize maximum distance from y=0
    """
    # Nidhi writes this!
    return np.max(np.abs(traj.y))
```

**Test it:**
```bash
# In tutorial menu, choose 1 again
# See how different trajectories score
```

**Key point:** "You can define ANY rule! It's just a function that returns a number."

---

### **Part 4: Multiple Rules - The Conflict! (15 min)**

**Show the problem:**

Run Demo 3 from tutorial:
```bash
python rule_evaluation_tutorial.py
# Choose: 3 (Priority Order Matters)
```

**Observe:**
- Short wiggly trajectory
- Long smooth trajectory

**Ask Nidhi:** "Which is better?"
- Short wiggly: Best at LENGTH
- Long smooth: Best at SMOOTHNESS
- **Can't both be best!** Need to decide priorities!

**Key point:** "When rules conflict, we need priorities!"

---

### **Part 5: Lexicographic Ordering (20 min)**

**Explain like alphabetical order:**

```
"apple" vs "banana"
→ Compare 1st letter: a < b
→ "apple" wins!

"apple" vs "apply"  
→ 1st letter: a = a (tied)
→ 2nd letter: p = p (tied)
→ 3rd letter: p = p (tied)
→ 4th letter: l = l (tied)
→ 5th letter: e < y
→ "apple" wins!
```

**Same for trajectories:**

```
Traj A: [5.0, 10.0, 15.0]  (P0, P1, P2)
Traj B: [5.0, 8.0, 20.0]

Compare P0: 5.0 = 5.0 (tied, continue)
Compare P1: 10.0 > 8.0 (B wins!)
Don't even check P2!
```

**Run Demo 2:**
```bash
# Choose: 2 (Lexicographic Ordering)
```

**Observe:**
- Score matrix (heatmap)
- Priority-by-priority comparison
- Explanation of why winner wins

**Key point:** "Higher priorities ALWAYS win! P0 violation beats ANY amount of P1/P2 violations!"

---

### **Part 6: Hands-On Exercises (20 min)**

**Exercise from workbook:**

```
Three routes:
            P0 (Safety)  P1 (Length)
Route A:        0            15
Route B:        0            10
Route C:        1             8

Question: Who wins?
```

**Walk through together:**
1. Compare A and B at P0: Tied (both 0)
2. Compare A and B at P1: B wins (10 < 15)
3. Compare B and C at P0: B wins (0 < 1)
4. Don't check P1 for B vs C!
5. Winner: B ✅

**Ask Nidhi:** "What if C had length 1 instead of 8? Would it win?"
**Answer:** No! Still loses at P0 (safety).

**Key point:** "Safety violation at P0 is worse than ANY length difference at P1!"

---

### **Part 7: Real-World Application (10 min)**

**Intersection scenario (from exercises):**

```
Approaching red light. Three options:

1. Go straight: [P0=0 (safe), P1=1 (illegal!), P2=5s]
2. Turn right:  [P0=0 (safe), P1=0 (legal),   P2=8s]
3. Stop:        [P0=0 (safe), P1=0 (legal),   P2=∞]

Winner: Turn right (legal and makes progress)
```

**Discuss:** "Why doesn't 'go straight' win even though it's fastest?"

**Answer:** Illegal at P1! Legal compliance beats efficiency.

**Key point:** "This is how autonomous cars make decisions! Safety first, then legality, then efficiency."

---

## 💡 Key Concepts to Emphasize

### **Concept 1: Rules Are Just Functions**
```python
trajectory → number
lower = better
```

### **Concept 2: Single Rule = Simple**
```python
best = min(trajectories, key=rule_function)
```

### **Concept 3: Multiple Rules = Priorities**
```python
Can't just sum scores!
Need lexicographic ordering!
```

### **Concept 4: Higher Priority ALWAYS Wins**
```python
P0 violation >> ANY amount of P1, P2, P3 violations
```

### **Concept 5: Priority Order Matters**
```python
[Length, Smoothness] → Short wiggly wins
[Smoothness, Length] → Long smooth wins
```

---

## 🎨 Visualization Highlights

**What Nidhi will see:**

1. **Trajectory plots:**
   - 2D paths (x, y)
   - Different colors
   - Centerline reference

2. **Bar charts:**
   - Score comparison
   - Winner highlighted in gold
   - Clear rankings

3. **Heatmaps:**
   - All scores at once
   - Green = good, Red = bad
   - Easy to spot patterns

4. **Individual rule plots:**
   - One subplot per rule
   - See trade-offs clearly

---

## 📝 Assessment Questions

**Check understanding at the end:**

1. **"What is a rule?"**
   - Answer: A function that scores trajectories

2. **"Why are lower scores better?"**
   - Answer: Convention (we minimize violations/costs)

3. **"What if two trajectories tie at P0?"**
   - Answer: Compare at P1, then P2, etc.

4. **"Can a P2 violation beat a P0 violation?"**
   - Answer: NO! Never! Higher priority always wins.

5. **"Does priority order matter?"**
   - Answer: YES! Changes the winner.

---

## 🎯 Success Criteria

**Nidhi understands if she can:**

- [ ] Explain what a rule is
- [ ] Calculate a simple rule score by hand
- [ ] Compare two trajectories with one rule
- [ ] Explain lexicographic ordering
- [ ] Compare two trajectories with multiple rules
- [ ] Understand why priority order matters
- [ ] Apply to real-world scenario (intersection)

---

## 🚀 Next Session Preview

**After mastering rules:**

1. ✅ Today: Rules and evaluation (Python only)
2. ⏭️ Next: Apply rules to intersection scenarios
3. ⏭️ Then: Implement in CARLA simulator
4. ⏭️ Final: Complete autonomous driving demo

---

## 🔧 Quick Start Commands

```bash
# Run interactive tutorial
python rule_evaluation_tutorial.py

# Menu options:
1 - Single rule demo
2 - Lexicographic ordering demo  
3 - Priority order matters demo
4-6 - Exercises

# Start with 1, then 3, then 2
```

---

## 📊 Expected Session Flow

```
0:00 - 0:10  Introduction & Demo 1
0:10 - 0:25  Single rule practice
0:25 - 0:40  Custom rule writing
0:40 - 0:55  Multiple rules (Demo 3)
0:55 - 1:15  Lexicographic ordering (Demo 2)
1:15 - 1:30  Exercises & discussion
```

---

## 💬 Discussion Prompts

**Throughout the session, ask:**

- "What do you think this trajectory will score?"
- "Which one will win? Why?"
- "What happens if we swap the priority order?"
- "Can you think of a real-world example?"
- "What rule would you add?"

**Keep it interactive!** Have Nidhi predict before revealing.

---

## ✅ Session Checklist

**Before starting:**
- [ ] Files ready (tutorial.py, guides)
- [ ] Python + matplotlib installed
- [ ] Workspace clear

**During session:**
- [ ] Explain rules as functions
- [ ] Show single rule demo
- [ ] Practice calculating by hand
- [ ] Show priority conflict (Demo 3)
- [ ] Explain lexicographic ordering
- [ ] Show complete demo (Demo 2)
- [ ] Work through exercises

**After session:**
- [ ] Nidhi can explain rules
- [ ] Nidhi understands lexicographic
- [ ] Completed at least 2 exercises
- [ ] Ready for next step (intersection)

---

## 🎓 Learning Outcomes

**After today, Nidhi will:**

✅ Understand rule-based planning foundations  
✅ Know how to define and evaluate rules  
✅ Understand lexicographic ordering  
✅ See how it applies to autonomous driving  
✅ Be ready to implement in CARLA  

---

**You're ready for today's session! Focus on understanding through visualization, not implementation. Keep it visual and interactive!** 🎨📊✨
