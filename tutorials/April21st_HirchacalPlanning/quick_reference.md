# Hierarchical Planning - Quick Reference Card
**For Nidhi - Keep this handy while coding!**

---

## 🎯 The Big Picture

```
Partial Order:
    P0 (Safety)
         ↓
    P1 (Legal)
         ↓
    ┌────┼────┐
    ↓    ↓    ↓
   P2   P3   P4
(Com) (Fuel)(Time)
Incomparable!

Hierarchical Stages:
1. Filter P0 (Safety)
2. Filter P1 (Legal)  
3. Context → Linearize
4. Select winner
```

---

## 📊 Algorithm Overview

```python
def plan(candidates):
    # Stage 1: Filter (O(n))
    safe = filter_safety(candidates)      # 200 → 127
    legal = filter_legal(safe)            # 127 → 82
    
    # Stage 2: Context (O(1))
    context = detect_context()            # HIGHWAY/CITY/PARKING
    
    # Stage 3: Select (O(m log m))
    order = get_total_order(context)
    winner = min(legal, key=order)        # 1 winner
    
    return winner
```

---

## 🔧 Key Functions

### **1. Trajectory Class**
```python
@dataclass
class Trajectory:
    id: int
    min_obstacle_distance: float  # meters
    max_speed: float             # m/s
    violates_red_light: bool
    comfort: float               # lower = better
    fuel: float                  # liters
    time: float                  # seconds
```

### **2. Safety Filter (P0)**
```python
def filter_safety(trajectories):
    safe = []
    for traj in trajectories:
        if traj.min_obstacle_distance >= SAFETY_MARGIN:
            safe.append(traj)
    return safe
```

### **3. Legal Filter (P1)**
```python
def filter_legal(trajectories):
    legal = []
    for traj in trajectories:
        if not traj.violates_red_light and \
           traj.max_speed <= SPEED_LIMIT:
            legal.append(traj)
    return legal
```

### **4. Context Detection**
```python
def detect_context(speed, has_traffic_lights):
    if speed > 25 and not has_traffic_lights:
        return HIGHWAY
    elif speed < 3:
        return PARKING
    else:
        return CITY
```

### **5. Total Order Linearization**
```python
def get_total_order(context):
    if context == HIGHWAY:
        return lambda t: (t.time, t.fuel, t.comfort)
    elif context == CITY:
        return lambda t: (t.comfort, t.time, t.fuel)
    else:  # PARKING
        return lambda t: (t.comfort, t.time, t.fuel)
```

---

## 💡 Key Concepts

### **Filtering Funnel**
```
200 candidates
    ↓ (P0: Safety check)
127 safe
    ↓ (P1: Legal check)
82 feasible
    ↓ (Context + Linearize)
1 winner
```

### **Context-Specific Winners**
```
Same candidates:
├─ HIGHWAY → Fastest trajectory wins
├─ CITY → Most comfortable wins
└─ PARKING → Most careful wins
```

### **Why It's Fast**
```
Exact Pareto:
├─ Compare all pairs: 200 × 200 = 40,000 ops
└─ O(n²) = too slow!

Hierarchical:
├─ Filter: 200 ops
├─ Context: 1 op
├─ Sort: 82 × 5 = 410 ops
├─ Total: ~611 ops
└─ 65× faster! ⚡
```

---

## 🐛 Common Issues & Fixes

### **Problem: No trajectories pass filter**
```python
if not safe:
    print("⚠️ NO SAFE OPTIONS!")
    return None
```

### **Problem: Need to see what's happening**
```python
print(f"Filter: {len(before)} → {len(after)}")
```

### **Problem: Wrong winner selected**
```python
# Check: Did you use the right context?
# Check: Is your lambda ordering correct?
# Remember: min() picks LOWEST values
```

---

## 🧪 Test Cases

### **Simple Test (3 trajectories)**
```python
routes = [
    Trajectory(1, 2.0, 12, False, 5, 10, 20),   # A
    Trajectory(2, 2.0, 12, False, 8, 5, 25),    # B
    Trajectory(3, 2.0, 12, False, 3, 15, 15),   # C
]

# Highway (Time matters) → Winner: 3 (fastest)
# City (Comfort matters) → Winner: 2 (smoothest)
```

### **Edge Cases**
```python
# All unsafe
trajs = [Trajectory(i, 0.5, 10, False, 5, 8, 15) for i in range(3)]
# Result: Should return None

# All violate red light
trajs = [Trajectory(i, 2.0, 10, True, 5, 8, 15) for i in range(3)]
# Result: Should return None
```

---

## 📈 Performance Metrics

### **Typical Results (200 candidates)**
```
Initial:         200 trajectories
After P0:        120-130 (60-65% pass)
After P1:        80-100 (40-50% pass)
Winner:          1 trajectory

Reduction:       50-60% eliminated
Speedup:         65-100× vs Pareto
Time:            < 1ms
```

---

## 🎓 Understanding Checks

**Can you answer these?**

✅ Why filter before optimizing?
✅ What makes hard constraints "hard"?
✅ How does context resolve incomparability?
✅ Why is this O(n) instead of O(n²)?
✅ What would happen with 1000 trajectories?

---

## 🚀 Extensions to Try

### **1. Add emergency mode**
```python
if emergency_mode:
    return lambda t: (t.time, t.comfort, t.fuel)
    # Time is MOST important!
```

### **2. Track statistics**
```python
stats = {
    'initial': len(candidates),
    'after_safety': len(safe),
    'after_legal': len(legal)
}
```

### **3. Visualize filtering**
```python
import matplotlib.pyplot as plt
plt.bar(['Initial', 'Safe', 'Legal'], 
        [200, 127, 82])
plt.savefig('funnel.png')
```

---

## 🔑 Key Python Syntax

### **Dataclass**
```python
@dataclass
class Trajectory:
    id: int
    distance: float
    # Auto-generates __init__, __repr__, etc!
```

### **Lambda Functions**
```python
# Creates anonymous function
order = lambda t: (t.time, t.fuel)

# Same as:
def order(t):
    return (t.time, t.fuel)
```

### **min() with key**
```python
winner = min(trajectories, key=lambda t: t.time)
# Finds trajectory with SMALLEST time
```

### **List Comprehension**
```python
safe = [t for t in trajs if t.distance >= 1.5]
# Same as:
safe = []
for t in trajs:
    if t.distance >= 1.5:
        safe.append(t)
```

---

## 📝 Debugging Checklist

When code doesn't work:

□ Check syntax (colons, indentation)
□ Print intermediate values
□ Test functions individually
□ Verify filter thresholds
□ Check context logic
□ Ensure lambda is correct
□ Try with simple test case first

---

## ✅ Session Checklist

**Built so far:**
□ Trajectory class
□ Safety filter
□ Legal filter  
□ Context detection
□ Total order function
□ Complete planner
□ Test with 200 trajectories
□ Visualization (optional)

**When done, you'll have:**
✅ Production-ready AV planning code
✅ Understanding of hierarchical planning
✅ Knowledge of partial order exploitation
✅ Code that's 65× faster than naive approach

---

## 🎯 Final Output Should Look Like

```
==================================================
HIERARCHICAL PLANNING
==================================================

Stage 1: FILTERING
Safety: 200 → 127 (63.5% passed)
Legal: 127 → 82 (64.6% passed)

Total reduction: 59.0% eliminated

Stage 2: CONTEXT
Detected: HIGHWAY
Total order: Time > Fuel > Comfort

Stage 3: SELECTION
Selected from 82 candidates
Winner: Trajectory 198

============================================================
WINNER
============================================================
ID:      198
Time:    10.4s
Fuel:    6.2L
Comfort: 4.8

Speedup vs Pareto: ~66×
```

---

**Remember: Build incrementally, test frequently, have fun! 🚀**
