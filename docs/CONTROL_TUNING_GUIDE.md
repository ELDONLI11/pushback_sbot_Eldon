# Control Tuning Guide

## Overview
This guide explains the control system improvements made to prevent tipping and provide better driver control.

---

## Driver Control Improvements

### 1. Squared Response Curve
**Location:** `src/drivetrain.cpp` - `applyCurve()` function

**What it does:**
- Applies a squared curve to joystick input
- Preserves the sign (direction) of the input
- Gives fine control at low stick deflections
- Provides aggressive acceleration at high stick deflections

**Response Table:**
| Joystick | Linear Power | Squared Power | Benefit |
|----------|--------------|---------------|---------|
| 25%      | 25%          | 6.25%         | 4x more control |
| 50%      | 50%          | 25%           | 2x more control |
| 75%      | 75%          | 56%           | Smooth ramp-up |
| 100%     | 100%         | 100%          | Full power maintained |

**To adjust:**
- No tuning needed - this is a fixed mathematical curve
- If you want even MORE fine control, use a cubic curve instead (change `normalized * std::abs(normalized)` to `normalized * normalized * std::abs(normalized)`)

---

### 2. Adaptive Slew Rate Limiting with Direction Reversal Protection
**Location:** `src/drivetrain.cpp` - `applySlewRate()` function  
**Configuration:** `include/config_sbot.h` - Multiple constants

**What it does:**
- **Adaptive behavior:** Uses different slew rates for normal vs direction reversals
- **Direction reversal detection:** Automatically detects when you're reversing from forward to backward (or vice versa)
- **Coast-through-zero:** Forces robot to stop briefly before reversing direction (prevents tipping)
- **Smart acceleration:** Fast response for normal driving, slow/safe response for reversals

**Current Settings:**
```cpp
SBOT_SLEW_RATE_NORMAL = 12        // Normal acceleration (same direction)
SBOT_SLEW_RATE_REVERSAL = 6       // Reversal rate (50% of normal)
SBOT_FORCE_STOP_ON_REVERSAL = true // Coast to zero before reversing
SBOT_REVERSAL_DEADBAND = 3        // How close to zero = "stopped"
```

**How It Works:**

| Scenario | Behavior | Why |
|----------|----------|-----|
| 0 → 100 (forward accel) | Uses NORMAL rate (12) | Fast response |
| 100 → 50 (slowing down) | Uses NORMAL rate (12) | Responsive braking |
| 100 → -100 (reversal) | Forces stop, then REVERSAL rate (6) | **Prevents tipping!** |
| 50 → -50 (direction flip) | Same as above | Smooth transition |

**The Magic: Coast-Through-Zero**

When you reverse direction (e.g., joystick from +100 to -100):
1. Robot gradually slows to 0 using REVERSAL rate (6 units/cycle)
2. Once within DEADBAND (±3), it's considered "stopped"
3. Robot then accelerates in new direction using REVERSAL rate
4. Result: Smooth, predictable, no tipping!

**Tuning Guide:**

**SBOT_SLEW_RATE_NORMAL** (Normal Driving)
```
Value | Feel               | Use When
------|--------------------|-----------------------
8     | Smooth/gentle      | Very tippy robot
12    | Balanced (default) | Normal robot
15    | Responsive         | Stable robot
20    | Very aggressive    | Heavy/stable robot
```

**SBOT_SLEW_RATE_REVERSAL** (Direction Changes)
```
Value | Time to Reverse | Safety | Use When
------|-----------------|--------|------------------
3     | ~2 seconds      | Safest | Robot tips easily
6     | ~1 second       | Safe   | Default (recommended)
9     | ~0.7 seconds    | Medium | Stable robot
12    | ~0.5 seconds    | Low    | Testing only
```

**Recommended Ratio:** REVERSAL should be 40-60% of NORMAL
- Too low: Reversals feel sluggish
- Too high: Defeats the purpose, robot may still tip

**SBOT_FORCE_STOP_ON_REVERSAL** (Coast Behavior)
```
Setting | Behavior | Pros | Cons
--------|----------|------|------
true    | Always stops before reversing | Safest, most predictable | Slight delay
false   | Gradually reverses | Faster | Might tip if REVERSAL rate too high
```

**SBOT_REVERSAL_DEADBAND** (How close to zero = stopped)
```
Value | Effect
------|--------
1-2   | Very strict (must be nearly zero) - more stopping time
3-5   | Balanced (default) - natural feeling
8-10  | Loose (counts as stopped early) - faster but less smooth
```

**Symptoms and Solutions:**
- **Robot tips when reversing** → Already using coast-through-zero! Check:
  - Decrease `SBOT_SLEW_RATE_REVERSAL` to 3-4
  - Increase `SBOT_REVERSAL_DEADBAND` to 5
  - Ensure `SBOT_FORCE_STOP_ON_REVERSAL = true`
- **Reversals feel too slow** → 
  - Increase `SBOT_SLEW_RATE_REVERSAL` to 8-9
  - Decrease `SBOT_REVERSAL_DEADBAND` to 2
  - Try `SBOT_FORCE_STOP_ON_REVERSAL = false` (test carefully!)
- **Normal driving feels sluggish** → Increase `SBOT_SLEW_RATE_NORMAL` to 15-18
- **Still responsive but reversals are safe** → Perfect! You found the sweet spot

---

## Autonomous Control Improvements

### 3. LemLib Motion Profiling
**Location:** `src/lemlib_config_sbot.cpp` - Controller settings

**What it does:**
- Limits acceleration during autonomous movements
- Smooths out starts and stops
- Automatically slows down as robot approaches target (built into LemLib)
- Prevents tipping during path following

**Current Settings:**
```cpp
Linear Controller:
  max acceleration = 50 units/s²

Angular Controller:
  max acceleration = 40 deg/s²
```

**Tuning Guide:**

| Setting | Effect on Movement | Effect on Tipping | Recommend When |
|---------|-------------------|-------------------|----------------|
| 0 (unlimited) | Fastest possible | High tipping risk | Never (unless desperate for speed) |
| 20-30 | Very conservative | Minimal tipping | Heavy robot or slippery field |
| 40-60 | Balanced (current) | Good compromise | Default starting point |
| 80-100 | Aggressive | Some tipping risk | Stable robot, need speed |
| 150+ | Nearly unlimited | High risk | Testing only |

**Symptoms and Solutions:**
- **Robot tips during autonomous movements** → Decrease both to 30
- **Autonomous is too slow** → Increase both to 70-80
- **Tips only during turns** → Decrease angular_controller only to 30
- **Tips only during straight moves** → Decrease linear_controller only to 30

**Testing Process:**
1. Start with conservative values (30/30)
2. Run your autonomous routine
3. If no tipping, increase by 10
4. Repeat until you find the sweet spot
5. Back off by 10 for safety margin

---

## Complete Control Flow Diagram

```
Driver Control (opcontrol):
  Raw Joystick Input (-127 to 127)
       ↓
  [1] Apply Deadzone (removes jitter near center)
       ↓
  [2] Apply Squared Curve (fine control at low speeds)
       ↓
  [3] Scale by Sensitivity (global power scaling)
       ↓
  [4] Adaptive Slew Rate Limiting:
       │
       ├─→ Same direction? Use SBOT_SLEW_RATE_NORMAL (fast)
       │
       └─→ Direction reversal?
            ├─→ If FORCE_STOP: Coast to zero, then reverse slowly
            └─→ If not: Use SBOT_SLEW_RATE_REVERSAL (slow)
       ↓
  Motor Command Sent

Visual Example of Direction Reversal (Joystick +100 → -100):

With FORCE_STOP_ON_REVERSAL = true:
Time: 0ms    50ms   100ms  150ms  200ms  250ms  300ms
Motor: 100 → 94 → 88 → 82 → 76 → ... → 0 → -6 → -12 → ... → -100
               ↑ Slowing down (REVERSAL rate)    ↑ Speeding up (REVERSAL rate)
                            Coast through zero!

Without FORCE_STOP (REVERSAL rate only):
Time: 0ms    50ms   100ms  150ms  200ms  250ms
Motor: 100 → 94 → 88 → 82 → ... → 0 → ... → -88 → -94 → -100
               ↑ Continuous slow transition (more tipping risk)

Autonomous Control:
  Target Position/Heading
       ↓
  LemLib PID Controller (calculates required speed)
       ↓
  Motion Profiling (limits acceleration)
       ↓
  Automatic Deceleration (slows near target)
       ↓
  Motor Command Sent
```

---

## Quick Tuning Checklist

### If robot tips during driver control:
- [ ] Decrease `SBOT_SLEW_RATE_LIMIT` to 7 or 5
- [ ] Test with gentle joystick movements first
- [ ] Gradually increase slew limit until comfortable

### If robot tips during autonomous:
- [ ] Decrease `linear_controller` max acceleration to 30
- [ ] Decrease `angular_controller` max acceleration to 30
- [ ] Test with simple movements first (single straight line)
- [ ] Check that robot isn't top-heavy (mechanical issue)

### If robot feels too slow/sluggish:
- [ ] Increase `SBOT_SLEW_RATE_LIMIT` in steps of 5
- [ ] Increase autonomous max acceleration in steps of 10
- [ ] Verify motors are not overheating
- [ ] Check battery voltage (low battery = slow response)

### If you want even MORE fine control at low speeds:
- [ ] Change squared curve to cubic in `applyCurve()`
- [ ] Or add exponential scaling: `return (int)(127 * std::exp(std::abs(normalized) * 2) - 127) * sign`

---

## Theory: Why This Works

### Squared Curve
Human perception is logarithmic, not linear. A squared curve matches how we naturally expect controls to feel. Racing games and flight simulators use the same technique.

### Slew Rate Limiting
Physics: F = ma. Limiting acceleration limits force, which limits tipping torque. By controlling the rate of change (jerk), we prevent the momentum shift that causes tipping.

### LemLib Motion Profiling
Uses trapezoidal velocity profiles:
- Constant acceleration phase (limited by max_acceleration)
- Constant velocity phase (cruising)
- Constant deceleration phase (automatically calculated)

This ensures smooth starts and stops without sudden jerks.

---

## Advanced: Combining with PTO/Mechanism Control

If you have a PTO (power take-off) that shifts weight during movement:
1. Activate slew limiting BEFORE engaging PTO
2. Add extra delay after PTO shift to let robot stabilize
3. Consider different slew rates for PTO-engaged vs disengaged states

Example:
```cpp
// In your PTO code
void enablePTO() {
    // Store current slew rate
    int normal_slew = SBOT_SLEW_RATE_LIMIT;
    
    // Reduce slew temporarily
    SBOT_SLEW_RATE_LIMIT = 5;
    
    // Engage PTO
    pto_solenoid.extend();
    pros::delay(200); // Let weight settle
    
    // Restore slew rate
    SBOT_SLEW_RATE_LIMIT = normal_slew;
}
```

---

## Troubleshooting

**Q: Robot still tips even with low slew rate**  
A: This might be a mechanical issue. Check:
- Center of gravity too high
- Wheels not properly tightened
- Floor surface too slippery
- Battery mounted too high

**Q: Slew limiting causes robot to not reach full speed**  
A: This shouldn't happen. The slew rate only limits how FAST speed changes, not the final speed. Check:
- Is your control loop running fast enough? (should be every 10-50ms)
- Are you sending commands continuously, or only once?

**Q: Squared curve feels weird**  
A: It takes practice! The first few drives feel strange because you're used to linear. Give it 5-10 minutes and it becomes natural. If you hate it, change back by removing the `applyCurve()` call.

**Q: Autonomous moves smoothly but driver control doesn't**  
A: Increase `SBOT_SLEW_RATE_LIMIT`. Driver control needs faster response than autonomous.

---

## Testing Procedure

### Test 1: Adaptive Slew Rate & Direction Reversal
**Purpose:** Verify anti-tipping behavior during reversals

1. Place robot in open area
2. **Test A: Normal Acceleration**
   - Push joystick smoothly from 0 to +100
   - Should ramp up at NORMAL rate (fairly quick)
   - Time it: should reach full speed in ~1 second
3. **Test B: Direction Reversal**
   - At full forward (+100), quickly flip joystick to full reverse (-100)
   - 🔍 **WATCH**: Robot should visibly slow down, stop briefly, then reverse
   - Should take ~1-2 seconds total
   - **SUCCESS:** No tipping, smooth transition
   - **FAILURE:** Robot tips or jerks → Decrease SBOT_SLEW_RATE_REVERSAL
4. **Test C: Multiple Quick Reversals**
   - Rapidly alternate joystick: +100 → -100 → +100 → -100
   - Robot should handle this gracefully (even if slow)
   - **SUCCESS:** Smooth, controlled, no tipping
   - **FAILURE:** Chaotic/uncontrolled → Increase SBOT_REVERSAL_DEADBAND
5. **Test D: Partial Reversals**
   - Go from +80 to -80 (not full range)
   - Should still coast through zero
   - Verify REVERSAL_DEADBAND is working (±3)

### Test 2: Squared Curve Feel
1. Slowly increase joystick from 0 to 25%
2. Observe: Robot should barely move
3. Increase to 50%
4. Observe: Still relatively slow
5. Increase to 100%
6. Observe: Robot accelerates more aggressively

### Test 3: Autonomous Tipping
1. Create simple test: move forward 48", turn 90°, move back
2. Run at current settings - does it tip?
3. If yes, reduce max acceleration by 10
4. Repeat until stable

---

## Recommended Final Settings

**For heavy/stable robots:**
```cpp
SBOT_SLEW_RATE_NORMAL = 18
SBOT_SLEW_RATE_REVERSAL = 10
SBOT_FORCE_STOP_ON_REVERSAL = false  // Can skip the coast
linear max_acceleration = 60
angular max_acceleration = 50
```

**For light/tippy robots (RECOMMENDED START HERE):**
```cpp
SBOT_SLEW_RATE_NORMAL = 10
SBOT_SLEW_RATE_REVERSAL = 4
SBOT_FORCE_STOP_ON_REVERSAL = true   // Must coast to zero
SBOT_REVERSAL_DEADBAND = 5           // Extra safety margin
linear max_acceleration = 30
angular max_acceleration = 30
```

**For competition (after testing):**
```cpp
SBOT_SLEW_RATE_NORMAL = 12
SBOT_SLEW_RATE_REVERSAL = 6
SBOT_FORCE_STOP_ON_REVERSAL = true   // Safe default
SBOT_REVERSAL_DEADBAND = 3
linear max_acceleration = 45-55
angular max_acceleration = 35-45
```

**For maximum speed (test CAREFULLY):**
```cpp
SBOT_SLEW_RATE_NORMAL = 20
SBOT_SLEW_RATE_REVERSAL = 12
SBOT_FORCE_STOP_ON_REVERSAL = false
SBOT_REVERSAL_DEADBAND = 2
linear max_acceleration = 70
angular max_acceleration = 60
```

---

## Summary

✅ **Squared curve** - Done, no tuning needed  
✅ **Slew rate limiting** - Tune `SBOT_SLEW_RATE_LIMIT` based on feel  
✅ **Autonomous smoothing** - Tune `max_acceleration` in both controllers

All changes are **non-intrusive** and can be easily reverted if needed. Start conservative, then increase aggressiveness as you gain confidence.
