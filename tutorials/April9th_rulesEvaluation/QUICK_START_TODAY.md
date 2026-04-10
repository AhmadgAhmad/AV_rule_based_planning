# Today's Session - Quick Start Guide ⚡

**Session 2: Intersection Scenarios with Rule-Based Planning**

---

## 🎯 **What You're Teaching Today:**

Apply the rule evaluation concepts from last session to **real intersection scenarios** with traffic lights.

---

## 📦 **Files You Have:**

### **1. TODAYS_SESSION_PLAN.md** 📋 **← YOUR GUIDE**
- Complete 90-minute lesson plan
- Minute-by-minute schedule
- Discussion questions
- Teaching tips
- Assessment questions

### **2. todays_session_interactive.py** 🐍 **← RUN THIS**
- Interactive Python demo
- 4 intersection scenarios
- Automatic score calculation
- Visualizations with matplotlib
- Asks Nidhi to predict before revealing answers

### **3. todays_session_worksheet.md** 📝 **← FOR NIDHI**
- Fill-in-the-blank worksheet
- Score tables to complete
- Reflection questions
- Self-assessment

---

## ⚡ **Quick Start (5 Steps):**

### **Step 1: Review Homework (5 min)**
- "Did you complete it? Which part was interesting?"
- "What did you put as P0 in school bus rulebook?"
- Quick code review if she submitted Part 1

### **Step 2: Present Problem (5 min)**
- "Today: Intersection with traffic light"
- Show diagram of car approaching intersection
- Preview 4 scenarios (A, B, C, D)

### **Step 3: Build Rulebook Together (15 min)**
- Brainstorm 5 rules with Nidhi
- P0: Collision, P1: Legal, P2: Blocking, P3: Progress, P4: Efficiency
- Justify priority order

### **Step 4: Work Through Scenarios (40 min)**
Run the interactive script:
```bash
python todays_session_interactive.py
```

For each scenario:
1. Show context
2. Ask Nidhi to predict winner
3. Fill score table together
4. Reveal answer with lexicographic comparison
5. Visualize with bar charts

### **Step 5: CARLA Demo (15 min, if time)**
```bash
# Terminal 1
cd ~/CARLA_0.9.13
./CarlaUE4.sh

# Terminal 2
python intersection_scenario_demo.py
# Choose: D (green but blocked)
```

---

## 📊 **The 4 Scenarios:**

### **Scenario A: Green + Clear** 🟢
- **Winner:** STRAIGHT
- **Why:** All safe/legal, best progress (P3)
- **Key lesson:** Basic case

### **Scenario B: Yellow Light** 🟡
- **Winner:** STOP
- **Why:** Can stop safely, conservative choice
- **Key lesson:** Yellow = stop if you can

### **Scenario C: Red Light** 🔴
- **Winner:** RIGHT
- **Why:** STRAIGHT illegal (P1), RIGHT is only legal moving option
- **Key lesson:** Legal (P1) beats efficiency (P4)

### **Scenario D: Green BUT Blocked** 🟢🚗
- **Winner:** RIGHT
- **Why:** STRAIGHT would block (P2), RIGHT doesn't
- **Key lesson:** Green ≠ always go! Don't block the box!

---

## 🎯 **Key Teaching Points:**

### **Point 1: The Rulebook**
```
P0: Collision (safety) - ALWAYS highest
P1: Legal (traffic law) - Must follow rules
P2: Blocking (courtesy) - Don't block others
P3: Progress (distance) - Prefer moving
P4: Efficiency (time) - Prefer faster
```

### **Point 2: Lexicographic Comparison**
```
Compare priority by priority:
- P0 different? → Lower P0 wins, STOP
- P0 tied? → Check P1
- P1 different? → Lower P1 wins, STOP
- Continue until tie breaks...
```

### **Point 3: Green ≠ Always Go**
```
Scenario D teaches:
- Green light is LEGAL
- But might still be wrong to go
- Check P2 (blocking)!
```

---

## 💬 **Key Questions to Ask:**

Throughout session:

- "Which action do you predict will win?"
- "Why is safety always P0?"
- "Can we go on green?" (Scenario D)
- "Why doesn't STRAIGHT win in Scenario C even though it's fastest?"
- "How is this different from your school bus rulebook?"

---

## ✅ **Success Criteria:**

**Nidhi succeeds if she can:**

- ✓ Explain all 5 priorities
- ✓ Fill score tables correctly
- ✓ Predict winners (at least 2/4)
- ✓ Use lexicographic ordering
- ✓ Understand "green ≠ always go"

---

## ⏰ **Timeline:**

```
0:00 - 0:15  Homework review + problem intro
0:15 - 0:30  Build rulebook together
0:30 - 0:50  Scenario A & B
0:50 - 1:10  Scenario C & D  
1:10 - 1:25  CARLA demo (optional)
1:25 - 1:30  Wrap-up
```

---

## 🛠️ **Running the Interactive Demo:**

```bash
python todays_session_interactive.py
```

**What happens:**
1. Shows rulebook
2. For each scenario:
   - Presents context
   - Prints score table
   - Asks for prediction
   - Shows lexicographic comparison
   - Reveals winner
   - Explains why
   - Shows bar chart visualizations
3. Summary at end

**Interactive parts:**
- Press Enter to continue between sections
- Type prediction when asked
- See if prediction was correct

---

## 📝 **Worksheet Usage:**

**Give Nidhi the worksheet to:**
- Fill in during session (as you go)
- Take notes on key insights
- Practice score tables
- Self-assess understanding

**Review together:**
- Check her answers
- Discuss any confusion
- Reinforce key concepts

---

## 🎓 **Learning Outcomes:**

**After today, Nidhi will:**

✅ Apply rules to real scenarios  
✅ Understand intersection decision-making  
✅ Know that green ≠ always go  
✅ See lexicographic ordering in action  
✅ Be ready for CARLA implementation  

---

## 🚀 **Next Session Preview:**

After today's success:

**Next session could cover:**
- More complex scenarios (pedestrians, uncertainty)
- Full CARLA implementation
- Multiple obstacles
- Dynamic replanning
- Multi-step planning

---

## 💡 **Pro Tips:**

### **If She Struggled with Homework:**
- Spend 20 min reviewing concepts
- Do Scenario A together step-by-step
- Skip CARLA demo, focus on understanding

### **If She Did Great on Homework:**
- Quick review (10 min)
- Let her lead on Scenario A
- Challenge her with "what if" questions
- Definitely do CARLA demo

### **Keep It Interactive:**
- Have her predict before revealing
- Ask "why" frequently
- Draw comparisons to school bus rulebook
- Make connections to real self-driving cars

---

## 📊 **Session Flow:**

```
┌──────────────────────┐
│ Homework Review      │
│ (What did you learn?)│
└──────────────────────┘
          ↓
┌──────────────────────┐
│ Build Rulebook       │
│ (5 priorities)       │
└──────────────────────┘
          ↓
┌──────────────────────┐
│ Scenario A (Green)   │
│ Predict → Calculate  │
└──────────────────────┘
          ↓
┌──────────────────────┐
│ Scenario B (Yellow)  │
│ Conservative choice  │
└──────────────────────┘
          ↓
┌──────────────────────┐
│ Scenario C (Red)     │
│ Legal beats efficient│
└──────────────────────┘
          ↓
┌──────────────────────┐
│ Scenario D (Blocked) │
│ Green ≠ always go!   │
└──────────────────────┘
          ↓
┌──────────────────────┐
│ CARLA Demo (optional)│
│ See it in action!    │
└──────────────────────┘
```

---

## ✅ **Pre-Session Checklist:**

- [ ] Review Nidhi's homework (if submitted)
- [ ] Test interactive script (runs without errors)
- [ ] Print or have worksheet ready
- [ ] CARLA ready (if doing demo)
- [ ] Whiteboard/paper for diagrams

---

## 🎯 **Expected Outputs:**

**By end of session:**

- ✓ Completed worksheet
- ✓ Understanding of 4 scenarios
- ✓ Can explain lexicographic ordering
- ✓ Knows green ≠ always go
- ✓ Ready for next level (CARLA)

---

## 💬 **Closing Questions:**

**Ask Nidhi:**

1. "What was most surprising today?"
2. "Why is safety always P0?"
3. "Can you design a 6th rule?"
4. "What should we cover next session?"

---

**You're ready! This session builds perfectly on the homework and prepares for CARLA implementation next time!** 🚦🎓✨

---

## 🔧 **Troubleshooting:**

**If interactive script has issues:**
```bash
pip install numpy matplotlib
python todays_session_interactive.py
```

**If visualization doesn't show:**
- Make sure matplotlib backend is working
- Try adding `plt.ion()` at start of script

**If she's confused:**
- Go back to alphabet analogy
- Work through one scenario step-by-step on whiteboard
- Skip CARLA, focus on understanding

---

**Have a great session! The materials are ready, just follow the plan!** 🚀
