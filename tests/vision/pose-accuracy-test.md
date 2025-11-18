# Vision Pose Accuracy Test

## Purpose
Validate PhotonVision + AprilTag pose estimation accuracy to ensure autonomous routines can reliably locate the robot for reef alignment and barge launching.

## Test Type
Static position validation using only vision system (PhotonVision + AprilTags)

## Success Criteria

### Target Performance (Elite FRC Teams)
- **Position error:** < 5cm
- **Angle error:** < 2°
- **Camera agreement:** All cameras within ±10cm of each other
- **Ambiguity score:** < 0.3 for multi-tag detections

### Pass/Fail Thresholds

| Metric | Pass | Needs Work | Fail |
|--------|------|------------|------|
| Position accuracy | < 5cm | 5-10cm | > 10cm |
| Angle accuracy | < 2° | 2-5° | > 5° |
| Camera spread | < 10cm | 10-15cm | > 15cm |
| Tag ambiguity | < 0.3 | 0.3-0.8 | > 0.8 |

---

## Equipment Required

- [ ] Fully charged robot with PhotonVision running
- [ ] Field with regulation AprilTags properly mounted
- [ ] 5m measuring tape (±1mm accuracy)
- [ ] Digital protractor or angle finder
- [ ] Laptop with AdvantageScope or PhotonVision dashboard
- [ ] Clipboard with data recording sheet (print section below)
- [ ] Camera/phone for documenting test setup

---

## Test Positions

Select 5 representative field positions where vision is critical:

### Recommended Positions (Edit based on your strategy):
1. **Reef approach** - Where you align to score on reef
2. **Barge launch position** - Where you shoot into barge
3. **Starting zone** - Autonomous starting position
4. **Mid-field** - Pathfinding transition area
5. **Algae collection zone** - Where you pick up game pieces

### Requirements for Each Position:
- 2-3 AprilTags visible to at least one camera
- Representative of actual match usage
- Tags at various distances (near and far)

---

## Procedure

### Pre-Test Setup
1. [ ] Verify robot battery voltage > 12.0V
2. [ ] Verify all 3 PhotonVision cameras are streaming (check dashboard)
3. [ ] Verify AdvantageKit logging is enabled
4. [ ] Print data recording sheet (see below)
5. [ ] Mark test positions on field with tape

### For Each Test Position:

#### Step 1: Position Robot
- Place robot at marked test position
- Align robot to desired angle using protractor
- Ensure robot is completely stationary
- **DO NOT move robot once positioned**

#### Step 2: Measure Ground Truth
Using measuring tape and protractor, record:
- **X position** (meters from field origin)
- **Y position** (meters from field origin)  
- **Angle θ** (degrees, 0° = facing red alliance wall)

*Tip: Measure from field corners to robot center for accuracy*

#### Step 3: Wait for Vision Stabilization
- Wait 10 seconds with robot stationary
- Observe vision pose on dashboard for stability
- Note if pose is jumping or stable

#### Step 4: Record Vision Data
From PhotonVision dashboard or AdvantageScope, record:
- **Camera 1 pose:** (X, Y, θ)
- **Camera 2 pose:** (X, Y, θ)
- **Camera 3 pose:** (X, Y, θ)
- **Fused pose:** (X, Y, θ)
- **Ambiguity scores** for each camera
- **Number of tags visible** per camera
- **Average tag distance** (if available)

#### Step 5: Calculate Errors
- Position error = distance between vision pose and ground truth
- Angle error = absolute difference in rotation
- Camera spread = max difference between any two cameras

#### Step 6: Document
- Take photo of robot position
- Note any observations (lighting, tag visibility, etc.)

---

## Data Recording Sheet

```
TEST RUN: Pose Accuracy Test
Date: ___________  Battery: _____V  Tester: ___________

Position 1: _______________________ (e.g., "Reef approach")
  Ground Truth:  X = _____ m,  Y = _____ m,  θ = _____ °
  
  Camera 1:      X = _____ m,  Y = _____ m,  θ = _____ °  
                 Tags visible: ___  Ambiguity: _____
                 
  Camera 2:      X = _____ m,  Y = _____ m,  θ = _____ °
                 Tags visible: ___  Ambiguity: _____
                 
  Camera 3:      X = _____ m,  Y = _____ m,  θ = _____ °
                 Tags visible: ___  Ambiguity: _____
                 
  Fused Pose:    X = _____ m,  Y = _____ m,  θ = _____ °
  
  ERRORS:
    Position error: _____ cm
    Angle error:    _____ °
    Camera spread:  _____ cm
    
  Pass/Fail: _______
  Notes: _________________________________

---

Position 2: _______________________
  [Repeat above format]

---

[Continue for positions 3-5]
```

---

## Analysis

After completing all test positions:

### 1. Calculate Statistics
- Average position error across all positions
- Maximum position error (worst case)
- Average angle error
- Identify which camera has highest error

### 2. Check for Patterns
- Does error increase with distance to tags?
- Is one camera consistently worse than others?
- Does error increase at certain angles?
- Are ambiguity scores correlated with error?

### 3. Determine Root Causes
If position error > 5cm, check:
- **Camera transform constants** in `Constants.java` - Are X, Y, Z, roll, pitch, yaw correct?
- **Camera mounting** - Did camera physically move/shift?
- **Ambiguity scores** - Are we rejecting bad detections properly?
- **Tag calibration** - Are AprilTags mounted at correct field positions?

### 4. Tuning Actions
Based on results:
- Update camera transform constants in `src/main/java/frc/robot/Constants.java` (lines 271-320)
- Adjust standard deviations for unreliable cameras
- Modify ambiguity thresholds if needed
- Physically re-mount cameras if necessary

---

## Expected Results

### First Run (Baseline)
Don't expect perfection - this establishes baseline

### After Tuning
Target: Fused pose within ±5cm, ±2° at all test positions

### Red Flags
- Error > 30cm = Camera transform is very wrong, re-measure physical mounts
- Pose jumping ±20cm = Ambiguity issues or multi-tag conflicts
- One camera 15cm+ worse than others = That camera's calibration is off

---

## Reference: Camera Transforms

Current values in `Constants.java`:

```java
// Camera 1 (front)
X = 0.307m, Y = 0.180m, Z = 0.750m
Roll = 0°, Pitch = 7.45°, Yaw = 0°

// Camera 2 (rear)  
X = -0.3327m, Y = 0m, Z = 0.3708m
Roll = 0°, Pitch = 0°, Yaw = 180°

// Camera 3 (side)
X = 0.238m, Y = -0.294m, Z = 0.625m
Roll = 0°, Pitch = 0°, Yaw = 40.5°
```

---

## Benchmark Teams

- **254 (Cheesy Poofs):** Target ±2cm pose accuracy
- **1678 (Citrus Circuits):** Publish ±3-5cm as competition standard
- **6328 (Mechanical Advantage):** Document ±5cm reliability threshold

---

## After Testing

1. Save results to `tests/vision/results-YYYY-MM-DD.md`
2. Log summary in engineering notebook with link to detailed results
3. If changes made, re-run test to validate improvements
4. Update this procedure based on learnings

---

## Questions or Issues?

Document in results file and discuss with vision/software leads.

