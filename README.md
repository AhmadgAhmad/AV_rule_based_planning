# Autonomous Driving with Rule-Based Planning 🚗

**Educational repository for learning motion planning and autonomous driving concepts**

Created by Ahmad Ahmad for Curious Cardinals Mentorship Program

---

## 📚 What's Inside

This repository contains complete implementations of autonomous driving scenarios in CARLA simulator, demonstrating:

- **Sampling-based motion planning** (generate multiple trajectory options)
- **Rule-based decision making** (lexicographic ordering with rulebooks)
- **Path following controllers** (Pure Pursuit algorithm)
- **Multi-objective planning** (safety, legality, courtesy, efficiency)

Perfect for students learning robotics, autonomous vehicles, and formal methods!

---

## 🎯 Project Demos

### 1. **Path Following Demo** (`path_following_demo.py`)
- Single trajectory generation
- Pure Pursuit controller
- Lane change maneuvers
- Chase camera following vehicle

**Concepts:** Quintic polynomials, Frenet frame, path tracking

---

### 2. **Sampling-Based Planner** (`sampling_planner_carla_demo.py`)
- Generate 36 candidate trajectories
- Sample across lanes, speeds, and durations
- Visualize all options in CARLA
- User selects best maneuver

**Concepts:** Sampling-based planning, trajectory diversity, visualization

---

### 3. **Overtaking Scenario** (`overtaking_planner_demo.py`)
- Multiple overtaking strategies (FOLLOW, QUICK, NORMAL, EARLY, CAUTIOUS)
- Multi-phase maneuvers (lane departure → passing → lane return)
- Collision detection and safety analysis
- Standalone visualization + CARLA integration

**Concepts:** Multi-phase planning, relative motion, safety constraints

---

### 4. **Intersection Scenario** (`intersection_scenario_demo.py`) ⭐
- Four different traffic scenarios (green/yellow/red/blocked)
- Multiple maneuver types (stop, straight, left, right)
- **Rule-based planning with lexicographic ordering**
- Automatic best action selection via rulebook

**Concepts:** Rule hierarchies, multi-objective optimization, traffic law compliance

---

## 🚀 Quick Start

### Prerequisites

```bash
# CARLA Simulator 0.9.13+
# Download from: https://github.com/carla-simulator/carla/releases

# Python 3.7+
# Required packages:
pip install numpy matplotlib
```

### Running a Demo

```bash
# Terminal 1: Start CARLA
cd ~/CARLA_0.9.13
./CarlaUE4.sh

# Terminal 2: Run demo
python intersection_scenario_demo.py
```

---

## 📖 Documentation

Each demo has a comprehensive guide:

- `BEGINNERS_GUIDE_SAMPLING_PLANNER.md` - Understanding sampling-based planning
- `INTERSECTION_DEMO_GUIDE.md` - Rule-based planning tutorial
- `INTERSECTION_SCENARIO_GUIDE.md` - Intersection planning concepts
- `CAMERA_VIEWS_GUIDE.md` - CARLA camera control
- `WAYPOINT_MARKERS_GUIDE.md` - Visualization techniques

Technical documentation:
- `overtaking_technical_document.pdf` - Complete LaTeX paper on overtaking

---

## 🎓 Learning Path

**Recommended order for students:**

1. **Start:** `BEGINNERS_GUIDE_SAMPLING_PLANNER.md` (understand basics)
2. **Run:** `sampling_planner_carla_demo.py` (see it in action)
3. **Experiment:** Modify speeds and trajectories
4. **Advance:** `INTERSECTION_DEMO_GUIDE.md` (learn rule-based planning)
5. **Run:** `intersection_scenario_demo.py` (see all 4 scenarios)
6. **Master:** Modify rulebook priorities, add new maneuvers

---

## 🔧 Key Concepts

### Sampling-Based Planning
```
Instead of optimizing (slow):
    trajectory = solve_optimization(constraints)

We sample many options (fast):
    trajectories = []
    for lane in [left, center, right]:
        for speed in [slow, medium, fast]:
            trajectories.append(generate(lane, speed))
    
    best = select_best(trajectories)
```

### Rule-Based Selection (Lexicographic Ordering)
```
Priority 0: Safety (collision-free)
    ↓ If tied...
Priority 1: Legality (follow traffic laws)
    ↓ If tied...
Priority 2: Courtesy (don't block others)
    ↓ If tied...
Priority 3: Progress (distance traveled)
    ↓ If tied...
Priority 4: Efficiency (minimize time)
```

**Higher priorities ALWAYS win!**

---

## 🎯 Intersection Scenarios

### Scenario A: Green Light, Clear
**Decision:** GO STRAIGHT (fastest legal option) ✅

### Scenario B: Yellow Light
**Decision:** STOP or GO (depends on distance and speed) ⚠️

### Scenario C: Red Light
**Decision:** TURN RIGHT (only legal moving option) ✅

### Scenario D: Green Light, Blocked
**Decision:** TURN RIGHT or STOP (don't block the box!) ✅

---

## 📊 Example Output

```
RULEBOOK DECISION
===========================================
Traffic Light: RED
Selected Action: RIGHT

Rulebook Evaluation:
  P0 (Collision):  0.0 ✓ Safe
  P1 (Legal):      0.0 ✓ Legal
  P2 (Blocking):   0.0 ✓ Clear
  P3 (Progress):   25.3m
  P4 (Efficiency): 6.3s

Other Options:
  STRAIGHT: P0=0 P1=1 ❌ ILLEGAL (red light!)
  LEFT:     P0=0 P1=1 ❌ ILLEGAL (red light!)
===========================================
```

---

## 🛠️ Customization

### Change Speeds (Safer/Slower)
```python
# In sampling_planner_carla_demo.py:
v_samples = [5.0, 7.0, 9.0]  # Instead of [8, 12, 15, 18]
```

### Change Rulebook Priorities
```python
# In intersection_scenario_demo.py:
# Make blocking MUCH worse:
r2 = check_blocking(trajectory, context) * 10
```

### Add New Maneuvers
```python
def generate_uturn_trajectory(s0, v0, ...):
    # Your implementation
    pass
```

---

## 📁 File Structure

```
RuleBookDriving/
├── demos/
│   ├── path_following_demo.py
│   ├── sampling_planner_carla_demo.py
│   ├── overtaking_planner_demo.py
│   └── intersection_scenario_demo.py
├── guides/
│   ├── BEGINNERS_GUIDE_SAMPLING_PLANNER.md
│   ├── INTERSECTION_DEMO_GUIDE.md
│   ├── INTERSECTION_SCENARIO_GUIDE.md
│   └── CAMERA_VIEWS_GUIDE.md
├── docs/
│   └── overtaking_technical_document.pdf
└── README.md
```

---

## 🎥 What You'll See in CARLA

### Trajectory Visualization
- 🔴 Red dots = STOP trajectory
- 🟢 Green dots = STRAIGHT trajectory
- 🔵 Blue dots = RIGHT trajectory
- 🟡 Yellow dots = LEFT trajectory
- ⚪ White arrow = Selected action

### Traffic Elements
- 🟢 Green sphere = Green light
- 🟡 Yellow sphere = Yellow light
- 🔴 Red sphere = Red light
- 🚗 Red markers = Blocking vehicles

---

## 🐛 Troubleshooting

### Car goes too fast
```python
# Reduce speeds in sampling_planner_carla_demo.py:
v_samples = [5.0, 7.0, 9.0]      # Target speeds (was [8, 12, 15, 18])
s_dot0 = 3.0                      # Initial speed (was 8.0)
Kp_throttle = 0.2                 # Acceleration (was 0.5)
```

### Can't see trajectories
```python
# Adjust height in visualization:
z=2.0  # Try z=1.0 or z=3.0
```

### CARLA connection error
```bash
# Make sure CARLA is running:
cd ~/CARLA_0.9.13
./CarlaUE4.sh
```

---

## 📚 References

### Papers & Concepts
- Werling et al. (2010) - Optimal trajectory generation in Frenet frame
- Frazzoli et al. (2019) - Rulebooks for autonomous driving
- Coulter (1992) - Pure Pursuit path tracking

### Tools
- CARLA Simulator: https://carla.org/
- Frenet Frame Planning: Lateral (d) + Longitudinal (s) decomposition
- Quintic Polynomials: Smooth trajectory generation

---

## 🎓 Educational Objectives

After working through these demos, students will understand:

✅ **Sampling-based planning** - Generate many options vs. optimization  
✅ **Frenet frame** - Separate lateral and longitudinal motion  
✅ **Pure Pursuit** - Simple but effective path tracking  
✅ **Rule-based planning** - Hierarchical decision making  
✅ **Lexicographic ordering** - How to compare priorities  
✅ **Multi-objective** - Balancing safety, legality, efficiency  
✅ **Formal methods** - Mathematical approach to autonomy  

---

## 🤝 Contributing

This is an educational repository. Suggestions and improvements welcome!

**Ideas for extension:**
- Add more intersection scenarios
- Implement MPC controller
- Add obstacle prediction
- Multi-vehicle scenarios
- Parking maneuvers
- Highway merging

---

## 📧 Contact

**Mentor:** Ahmad Ahmad  
**Program:** Curious Cardinals Mentorship  
**Focus:** Motion Planning, Temporal Logic, Reinforcement Learning

**For questions or collaboration:** Create an issue or reach out!

---

## 📄 License

Educational use - feel free to learn, modify, and share!

---

## 🌟 Acknowledgments

- **Nidhi** - Student mentee working through these concepts
- **Curious Cardinals** - Mentorship platform connecting students and mentors
- **CARLA Team** - Open-source autonomous driving simulator
- **Boston University** - Supporting robotics research

---

**Ready to learn autonomous driving? Start with the beginner's guide and run your first demo!** 🚗🎓✨
