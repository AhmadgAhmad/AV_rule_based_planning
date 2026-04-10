# Today's Session: Intersection Scenarios with Rule-Based Planning 🚦

**Session 2: From Theory to Real Scenarios**

**Date:** Follow-up session  
**Duration:** 90 minutes  
**For:** Ahmad & Nidhi

---

## 🎯 Today's Objectives

By the end of today's session, Nidhi will:
1. ✅ Review homework and solidify understanding
2. ✅ Apply rules to **real intersection scenarios**
3. ✅ Design rulebooks for traffic light decisions
4. ✅ Evaluate 4 different intersection scenarios
5. ✅ See it working in CARLA (if time permits)

**Focus:** Bridge from abstract rules → real autonomous driving decisions

---

## 📚 What We're Building On

### **Last Session:**
- Rules are scoring functions (trajectory → number)
- Lexicographic ordering (compare priority by priority)
- Single rule vs multiple rules
- Priority order matters

### **Homework:**
- Implemented custom rules (safety margin, speed consistency, lane keeping)
- Designed school bus rulebook
- (Optional) Analyzed yellow light scenario

### **Today:**
- Apply these concepts to **intersection scenarios**
- Four traffic light situations (A, B, C, D)
- Build complete rulebook for autonomous driving
- See it in action!

---

## ⏰ Session Timeline (90 minutes)

```
0:00 - 0:15  Part 1: Homework Review & Discussion
0:15 - 0:30  Part 2: Intersection Scenario Setup
0:30 - 0:50  Part 3: Design Intersection Rulebook
0:50 - 1:10  Part 4: Evaluate 4 Scenarios
1:10 - 1:25  Part 5: CARLA Demo (if time)
1:25 - 1:30  Part 6: Wrap-up & Next Steps
```

---

## 📋 PART 1: Homework Review (15 minutes)

### **Quick Discussion Questions:**

**Question 1:** "Did you complete the homework? Which part was most interesting?"

**Question 2:** "Let's talk about your school bus rulebook. What did you put as P0 (highest priority)?"

**Expected answer:** Safety/Collision Avoidance
- If correct: "Great! Why is that always P0?"
- If incorrect: "Let's think about this... would you ever sacrifice safety for speed? Why not?"

**Question 3:** "In the route comparison (A, B, C), which route won?"

**Expected answer:** Route C
- If correct: "Excellent! Walk me through why."
- If incorrect: "Let's work through it together step by step..."

---

### **Code Review (if she completed Part 1):**

**Quick test of one rule:**

```python
# Test Nidhi's safety margin rule
from nidhi_custom_rules import rule_safety_margin

# Create simple test trajectory
test_traj = Trajectory(
    x=np.linspace(0, 10, 50),
    y=np.ones(50) * 1.9,  # Close to obstacle at y=2.0
    name="Test"
)

score = rule_safety_margin(test_traj, obstacle_positions=[2.0])
print(f"Score: {score}")  # Should be > 0 (violations detected)
```

**Ask:** "What does this score mean? Is the trajectory safe?"

---

### **Key Takeaways to Reinforce:**

✅ **Rules measure violations/costs**  
✅ **Lower scores are better**  
✅ **Lexicographic ordering = priority by priority**  
✅ **Higher priorities ALWAYS win**  
✅ **Safety should always be P0**

**Transition:** "Now let's apply these concepts to a real autonomous driving problem: intersections with traffic lights!"

---

## 📋 PART 2: Intersection Scenario Setup (15 minutes)

### **The Problem:**

**Present the scenario:**

```
You're designing the decision-making system for a self-driving car.
The car is approaching an intersection with a traffic light.

Question: What should the car do?

The answer depends on:
- Traffic light color (green, yellow, red)
- What's blocking the intersection?
- How far away are we?
- How fast are we going?
```

---

### **Show Visual Diagram:**

```
        ↑
        │ North
        │
    ────┼──── 🚦 Traffic Light
        │
        •  You (25m away, driving 10 m/s)
        │
        ↓
```

---

### **Available Actions:**

**List the options:**

1. **STOP** - Brake to a stop before intersection
2. **STRAIGHT** - Continue straight through intersection
3. **RIGHT** - Turn right at intersection
4. **LEFT** - Turn left at intersection (if light is green)

**Ask Nidhi:** "Can you always do all of these? What constraints exist?"

**Expected answers:**
- Red light → Can't go straight or left
- Blocked intersection → Shouldn't enter even if green
- Safety → Must avoid collisions always

---

### **The Four Scenarios We'll Analyze:**

**Preview:**

```
Scenario A: 🟢 Green light, clear intersection
           → What should we do? (Hint: probably go straight)

Scenario B: 🟡 Yellow light, clear intersection
           → Tricky! Can we stop safely? Should we?

Scenario C: 🔴 Red light, clear intersection
           → Limited options! What's legal?

Scenario D: 🟢 Green light, BUT intersection blocked!
           → Green doesn't mean go! Why not?
```

**Build anticipation:** "We'll design a rulebook that handles ALL four scenarios correctly!"

---

## 📋 PART 3: Design Intersection Rulebook (20 minutes)

### **Activity: Build the Rulebook Together**

**Brainstorm rules with Nidhi:**

**Prompt:** "What rules should our autonomous car follow at intersections?"

**Guide her toward these (but let her suggest first):**

---

### **Rule 1: Collision Avoidance (P0)**

**Discussion:**
- "What's the most important thing?"
- "Would we ever run into another car to save time?"

```python
def rule_collision(action, obstacles):
    """
    Check if action would cause collision.
    
    Returns:
        0 = safe, 1 = collision
    """
    # If action would hit obstacle → return 1
    # Otherwise → return 0
```

**Key point:** This is ALWAYS P0. Non-negotiable.

---

### **Rule 2: Traffic Law Compliance (P1)**

**Discussion:**
- "After safety, what's next most important?"
- "Can we run red lights if no one is around?"

```python
def rule_legal_compliance(action, traffic_light):
    """
    Check if action violates traffic law.
    
    Returns:
        0 = legal, 1+ = violations
        
    Examples:
    - Go straight on red → 1 violation
    - Turn left on red → 1 violation
    - Turn right on red → 0 violations (usually legal)
    - Any action on green → 0 violations
    """
```

**Key point:** Even when safe, we must follow laws.

---

### **Rule 3: Courtesy / Don't Block Box (P2)**

**Discussion:**
- "What if the light is green but there's a traffic jam ahead?"
- "Should we enter the intersection?"

```python
def rule_blocking(action, blocked_ahead):
    """
    Check if action would block intersection.
    
    Returns:
        0 = won't block, 1 = would block
        
    Example:
    - Go straight into blocked intersection → 1
    - Turn right (clears intersection) → 0
    - Stop before intersection → 0
    """
```

**Key point:** "Don't block the box" - courtesy to cross-traffic.

---

### **Rule 4: Progress (P3)**

**Discussion:**
- "If all else is equal, what do we prefer?"
- "Sitting still or moving forward?"

```python
def rule_progress(action):
    """
    Measure forward progress.
    
    Returns:
        Negative distance traveled (we want to maximize)
        
    Example:
    - Stop → 0 progress
    - Straight → 30m progress → score = -30
    - Right → 20m progress → score = -20
    """
```

**Key point:** Only matters if higher priorities tie.

---

### **Rule 5: Efficiency (P4)**

**Discussion:**
- "Among safe, legal, non-blocking options, what's best?"
- "Fast or slow?"

```python
def rule_efficiency(action):
    """
    Measure time to complete action.
    
    Returns:
        Duration in seconds
        
    Example:
    - Stop → ∞ (waiting forever)
    - Straight → 5s
    - Right turn → 8s
    """
```

**Key point:** Lowest priority - only tie-breaker.

---

### **Complete Rulebook:**

**Write it on board/screen:**

```
INTERSECTION RULEBOOK
=====================

P0: Collision Avoidance (Safety)
    → 0 = safe, 1 = collision
    → MUST be 0, non-negotiable

P1: Traffic Law Compliance (Legal)
    → 0 = legal, 1+ = violations
    → Red light, speed limits, etc.

P2: Don't Block Intersection (Courtesy)
    → 0 = clear, 1 = blocking
    → Even if green, don't block box

P3: Forward Progress (Progress)
    → Negative distance (maximize)
    → Prefer moving over stopping

P4: Time Efficiency (Efficiency)
    → Duration in seconds
    → Prefer faster options
```

**Ask Nidhi:** "Does this priority order make sense? Why is P0 always collision?"

---

## 📋 PART 4: Evaluate Four Scenarios (20 minutes)

### **Now apply the rulebook to each scenario!**

---

### **Scenario A: Green Light, Clear** 🟢

**Context:**
```
Traffic light: GREEN
Intersection: CLEAR (no obstacles)
Distance: 25m
Speed: 10 m/s (36 km/h)
```

**Available actions:**
- STOP
- STRAIGHT
- RIGHT
- LEFT

---

**Work through together:**

**Step 1: Generate scores**

```
Action      P0(Collision) P1(Legal) P2(Block) P3(Progress) P4(Time)
------      ------------- --------- --------- ------------ --------
STOP              0           0         0          0           ∞
STRAIGHT          0           0         0        -40m          5s
RIGHT             0           0         0        -25m          8s
LEFT              0           0         0        -35m          7s
```

**Ask Nidhi:** "Fill in these scores. What's the collision score for STRAIGHT?"

---

**Step 2: Compare lexicographically**

```
P0: All tied (0 = 0 = 0 = 0) → continue
P1: All tied (0 = 0 = 0 = 0) → continue
P2: All tied (0 = 0 = 0 = 0) → continue
P3: STRAIGHT best (-40 > -35 > -25 > 0) → STRAIGHT WINS!
```

**Ask Nidhi:** "Which action wins? Why?"

**Answer:** STRAIGHT (best progress, all else equal)

---

### **Scenario B: Yellow Light** 🟡

**Context:**
```
Traffic light: YELLOW
Distance: 25m
Speed: 10 m/s
Can stop? Just barely (stopping dist = 25m)
```

**Actions:**
- STOP
- STRAIGHT (risky - might catch red)
- RIGHT

---

**Work through:**

```
Action      P0  P1  P2  P3      P4
------      --  --  --  ------  ----
STOP         0   0   0   -23m    3s
STRAIGHT     0   0   0   -40m    5s
RIGHT        0   0   0   -25m    8s
```

**Discussion:**
- "All are safe and legal on yellow"
- "Yellow means 'stop if you can safely'"
- "Since we CAN stop (just barely), what's the conservative choice?"

**Ask Nidhi:** "Which would a real autonomous car choose? Why?"

**Expected answer:** STOP (conservative, even though STRAIGHT is legal)

**Teaching point:** "Real AVs prefer stopping on yellow if possible. Safety margin!"

---

### **Scenario C: Red Light** 🔴

**Context:**
```
Traffic light: RED
Intersection: CLEAR
```

**Actions:**
- STOP
- STRAIGHT (illegal!)
- RIGHT (legal in most places)
- LEFT (illegal!)

---

**Work through:**

```
Action      P0  P1(Legal!) P2  P3      P4
------      --  ---------- --  ------  ----
STOP         0       0      0    -23m   ∞
STRAIGHT     0       1      0    -40m   5s  ← ILLEGAL!
RIGHT        0       0      0    -25m   8s
LEFT         0       1      0    -35m   7s  ← ILLEGAL!
```

**Ask Nidhi:** "What's different here? Look at P1."

**Key insight:**
```
Compare STRAIGHT vs RIGHT:
  P0: 0 = 0 (tied)
  P1: 1 > 0 (RIGHT wins immediately!)
  
Don't even check P2, P3, P4!
```

**Winner:** RIGHT (only legal moving option)

**Ask:** "Why doesn't STRAIGHT win even though it's faster? It has better P3 and P4!"

**Answer:** "P1 violation! Legal (P1) beats efficiency (P4)!"

---

### **Scenario D: Green Light, BUT Blocked** 🟢🚗

**Context:**
```
Traffic light: GREEN (legal to go)
Intersection: BLOCKED by traffic jam
```

**Actions:**
- STOP
- STRAIGHT (would block intersection!)
- RIGHT (clears intersection)

---

**Work through:**

```
Action      P0  P1  P2(Block!) P3      P4
------      --  --  ---------- ------  ----
STOP         0   0      0       -23m    ∞
STRAIGHT     0   0      1       -40m    5s  ← BLOCKS!
RIGHT        0   0      0       -25m    8s
LEFT         0   0      1       -35m    7s  ← BLOCKS!
```

**Key insight:**
```
Compare STRAIGHT vs RIGHT:
  P0: 0 = 0 (tied)
  P1: 0 = 0 (tied, both legal - it's green!)
  P2: 1 > 0 (RIGHT wins!)
  
STRAIGHT loses at P2 even though it's legal and faster!
```

**Winner:** RIGHT (doesn't block, makes progress)

**Ask:** "What's the lesson here? Can we always go on green?"

**Answer:** "NO! Green light doesn't always mean GO. Don't block the box!"

---

### **Summary Table:**

**Write this on board:**

```
Scenario  Light  Blocked?  Winner      Why?
--------  -----  --------  ----------  ---------------------------
A         Green    No      STRAIGHT    Best progress (P3)
B         Yellow   No      STOP        Conservative choice
C         Red      No      RIGHT       Only legal option (P1)
D         Green    Yes     RIGHT       Doesn't block (P2)
```

**Ask Nidhi:** "See how the same rulebook handles all four situations correctly?"

---

## 📋 PART 5: CARLA Demo (15 minutes, if time)

### **Show It Working in Real Simulator**

**Run the demo:**

```bash
# Terminal 1: CARLA
cd ~/CARLA_0.9.13
./CarlaUE4.sh

# Terminal 2: Intersection demo
python intersection_scenario_demo.py
# Choose scenario D (most interesting)
```

---

### **What Nidhi Will See:**

**Visual elements:**
- 🚗 Car approaching intersection
- 🟢 Green traffic light (sphere on post)
- 🚗🚗 Red obstacles blocking intersection
- Colored dots showing trajectory options

**Console output:**
```
RULEBOOK DECISION
=================
Traffic Light: GREEN
Blocked Ahead: True

Selected Action: RIGHT

Rulebook Evaluation:
  P0 (Collision):  0.0 ✓ Safe
  P1 (Legal):      0.0 ✓ Legal
  P2 (Blocking):   0.0 ✓ Clear
  P3 (Progress):   25.3m
  P4 (Efficiency): 6.3s

Other Options:
  STRAIGHT: P0=0 P1=0 P2=1 ❌ BLOCKS
```

---

### **Discussion Points:**

**Ask:**
- "See the green light? Should we go straight?"
- "Look at the red obstacles. What would happen if we went straight?"
- "Why does the car choose RIGHT even though STRAIGHT is faster?"

**Emphasize:**
- "This is exactly what we worked out on paper!"
- "The rulebook made the right decision!"
- "P2 (blocking) beat P4 (efficiency)!"

---

**If time permits, show another scenario:**

```bash
# Run Scenario C (red light)
python intersection_scenario_demo.py
# Choose: C
```

**Point out:**
- "Red light means limited options"
- "STRAIGHT gets P1=1 (illegal)"
- "RIGHT is chosen (only legal moving option)"
- "Same rulebook, different scenario, correct decision!"

---

## 📋 PART 6: Wrap-up (5 minutes)

### **Key Takeaways:**

**Ask Nidhi to summarize:**

"What did you learn today?"

**Expected answers:**
- Rules apply to real scenarios
- Same rulebook handles different situations
- Priority order ensures correct decisions
- Green doesn't always mean go!

---

### **Reinforce Concepts:**

**The Power of Lexicographic Ordering:**

```
One rulebook → Four scenarios → Correct decisions

How?
- P0 (collision) eliminates unsafe options
- P1 (legal) eliminates illegal options
- P2 (blocking) eliminates discourteous options
- P3 (progress) picks best among remaining
- P4 (efficiency) breaks final ties
```

---

### **Real-World Connection:**

**Explain:**

"This is EXACTLY how autonomous vehicles make decisions!

Companies like Waymo, Cruise, Tesla use rule-based systems:
- Safety is always P0
- Legal compliance is always high priority
- Comfort, efficiency are lower priorities
- Priorities ensure right decisions"

**Show video/image (if available):**
- Waymo car at intersection
- Tesla autopilot visualization
- "They're using the same concepts you just learned!"

---

### **Next Steps:**

**Preview next session:**

```
Today:  Intersection scenarios (4 situations)
Next:   More complex scenarios
        - Multiple obstacles
        - Pedestrians
        - Uncertain predictions
        - Dynamic replanning
```

**Homework for next time (optional):**

"Think about these questions:
1. What if a pedestrian suddenly appears?
2. What if we're not sure the intersection is clear?
3. What if other cars don't follow rules?

How would you modify the rulebook?"

---

## 📝 Assessment Questions

**Check understanding before ending:**

**Q1:** "What is P0 in our intersection rulebook?"
- Answer: Collision avoidance

**Q2:** "In Scenario D, why doesn't STRAIGHT win even though it's faster?"
- Answer: P2 violation (blocking) beats P4 (efficiency)

**Q3:** "Can we go straight on a green light?"
- Answer: Not always! Depends on if intersection is clear (P2)

**Q4:** "What makes lexicographic ordering better than summing scores?"
- Answer: Ensures higher priorities always win, no matter what

**If she answers all correctly:** "Excellent! You really understand this!" ✅

---

## 🎯 Success Criteria

**Nidhi succeeds if she can:**

- ✅ Explain the rulebook (5 priorities)
- ✅ Apply it to all 4 scenarios
- ✅ Correctly identify winners
- ✅ Explain why using lexicographic ordering
- ✅ Understand "green ≠ always go"
- ✅ Connect to real autonomous vehicles

---

## 💡 Teaching Tips

### **If She Struggles:**

**Concept: Lexicographic comparison**
- Go back to alphabet analogy
- Work through one scenario step-by-step
- Draw decision tree on board

**Concept: Priority order**
- Ask "Would you rather crash or be late?"
- Make it personal and relatable
- Use extreme examples

**Concept: Blocking rule**
- Draw the intersection
- Show what happens if you block it
- Explain "Don't block the box" traffic law

---

### **If She's Doing Great:**

**Challenge her:**
- "What if there's a pedestrian?"
- "What if we can't tell if it's blocked?"
- "Design a 6th rule for pedestrian crossings"

**Advanced topics:**
- Uncertainty in perception
- Predictions of other vehicles
- Multi-step planning
- Replanning every 0.5 seconds

---

### **Keep It Interactive:**

**Throughout session:**
- Ask prediction questions before revealing
- Have her fill in score tables
- Let her explain reasoning
- Encourage questions
- Draw diagrams together

---

## 🛠️ Materials Needed

### **Before Session:**

- [ ] Review Nidhi's homework
- [ ] Prepare feedback (use answer key)
- [ ] Test CARLA demo (make sure it runs)
- [ ] Print or prepare scenario diagrams
- [ ] Have whiteboard/paper for drawing

### **During Session:**

- [ ] Laptop with Python + matplotlib
- [ ] CARLA running (if doing demo)
- [ ] Whiteboard markers / digital whiteboard
- [ ] Homework submissions (if she sent them)

---

## 📊 Session Flow Summary

```
┌─────────────────────────────────────┐
│  Homework Review (15 min)           │
│  - Quick discussion                 │
│  - Code review if completed         │
│  - Reinforce key concepts           │
└─────────────────────────────────────┘
              ↓
┌─────────────────────────────────────┐
│  Intersection Setup (15 min)        │
│  - Present problem                  │
│  - Show 4 scenarios                 │
│  - Build anticipation               │
└─────────────────────────────────────┘
              ↓
┌─────────────────────────────────────┐
│  Design Rulebook (20 min)           │
│  - Brainstorm 5 rules together      │
│  - Justify priorities               │
│  - Write complete rulebook          │
└─────────────────────────────────────┘
              ↓
┌─────────────────────────────────────┐
│  Evaluate Scenarios (20 min)        │
│  - Work through A, B, C, D          │
│  - Fill score tables                │
│  - Explain winners                  │
└─────────────────────────────────────┘
              ↓
┌─────────────────────────────────────┐
│  CARLA Demo (15 min, optional)      │
│  - Run intersection_scenario_demo   │
│  - Show scenario D                  │
│  - Connect to theory                │
└─────────────────────────────────────┘
              ↓
┌─────────────────────────────────────┐
│  Wrap-up (5 min)                    │
│  - Summarize learnings              │
│  - Preview next session             │
│  - Assessment questions             │
└─────────────────────────────────────┘
```

---

## ✅ Post-Session Checklist

**After session:**

- [ ] Assess Nidhi's understanding
- [ ] Note areas of strength
- [ ] Identify concepts needing reinforcement
- [ ] Plan next session accordingly
- [ ] Send follow-up resources if needed

---

**You're ready for today's session! This builds directly on the homework and brings rule-based planning to life with real intersection scenarios!** 🚦🎓✨
