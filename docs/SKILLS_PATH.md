# Skills Autonomous - Complete Turn-by-Turn Path

## Overview
The Skills autonomous visits **all 8 corner tubes** (4 loader tubes + 4 cluster tubes) and scores at all available goals. The path is symmetric: complete left side (Red), then mirror on right side (Blue), then loop.

**Total tubes per cycle:** 8  
**Expected cycle time:** ~50-55 seconds  
**Expected points per cycle:** ~80 points (48 from tubes + 32 from scoring)  
**Loops:** Continuous until 30 seconds remain, then parks

---

## Starting Position
- **Jerry coordinates:** (-46.5, 13, 0°) - Same as Red Left match autonomous
- **Physical location:** Back bumper touching park zone strip at left alliance goal
- **Robot state:** Intake storage mode, loader retracted
- **Heading:** 0° faces into field (away from goal)

---

## LEFT SIDE (Red Alliance Side)

### Phase L1: Top-Left Loader Tube + Long Goal
| Step | Action | Target (Jerry) | Heading | Duration | Notes |
|------|--------|----------------|---------|----------|-------|
| 1 | Intake storage mode | - | 0° | - | Start collecting |
| 2 | Turn left | - | 90° | - | Clear goal area |
| 3 | Drive forward | (-48, 48) | 90° | - | Retreat to top-left corner |
| 4 | Turn left | - | 180° | - | Face loader tube |
| 5 | Deploy loader | - | 180° | 250ms | Extend pneumatic |
| 6 | Drive forward | (-73, 48) | 180° | - | Contact loader tube |
| 7 | **Collect balls** | (-73, 48) | 180° | **1800ms** | All 6 balls from tube |
| 8 | Stop intake, open flap | - | 180° | 100ms | Prepare for scoring |
| 9 | Drive backward | (-31, 48) | 180° | - | Left long goal (top) |
| 10 | **Score top goal** | (-31, 48) | 180° | 1200ms | Flap open, balls fall |
| 11 | Stop all mechanisms | - | 180° | - | Safety |

**Subtotal: ~6 balls collected, ~5-6 points scored**

---

### Phase L2: Top-Left Cluster Tube + Center Lower Goal
| Step | Action | Target (Jerry) | Heading | Duration | Notes |
|------|--------|----------------|---------|----------|-------|
| 12 | Turn | - | 270° | - | Face down |
| 13 | Intake storage mode | - | 270° | - | Start collecting |
| 14 | Drive forward | (-21, 21) | 270° | - | Top-left cluster |
| 15 | **Collect balls** | (-21, 21) | 270° | **1800ms** | All 6 balls from tube |
| 16 | Turn | - | 315° | - | Face center |
| 17 | Drive forward | (-8.3, 9.7) | 315° | - | Center lower left |
| 18 | **Score low goal** | (-8.3, 9.7) | 315° | 1300ms | Intake reverse |
| 19 | Stop all mechanisms | - | 315° | - | Safety |

**Subtotal: +6 balls, +3 points scored**

---

### Phase L3: Bottom-Left Cluster Tube + Center Middle Goal
| Step | Action | Target (Jerry) | Heading | Duration | Notes |
|------|--------|----------------|---------|----------|-------|
| 20 | Delay | - | 315° | 100ms | Settling |
| 21 | Turn | - | 225° | - | Face waypoint |
| 22 | Intake storage mode | - | 225° | - | Start collecting |
| 23 | Drive forward | (-24, 0) | 225° | - | Waypoint |
| 24 | Turn | - | 270° | - | Face down |
| 25 | Drive forward | (-24, -24) | 270° | - | Bottom-left cluster |
| 26 | **Collect balls** | (-24, -24) | 270° | **1800ms** | All 6 balls from tube |
| 27 | Turn | - | 315° | - | Face center |
| 28 | Drive backward | (-9, -9) | 315° | - | Center middle left |
| 29 | **Score mid goal** | (-9, -9) | 315° | 1000ms | Indexer reverse |
| 30 | Stop all mechanisms | - | 315° | - | Safety |

**Subtotal: +6 balls, +5 points scored**

---

### Phase L4: Bottom-Left Loader Tube + Long Goal
| Step | Action | Target (Jerry) | Heading | Duration | Notes |
|------|--------|----------------|---------|----------|-------|
| 31 | Delay | - | 315° | 100ms | Settling |
| 32 | Turn | - | 225° | - | Face corner |
| 33 | Intake storage mode | - | 225° | - | Prepare |
| 34 | Drive forward | (-48, -48) | 225° | - | Retreat bottom-left |
| 35 | Turn | - | 180° | - | Face loader |
| 36 | Deploy loader | - | 180° | 250ms | Extend pneumatic |
| 37 | Drive forward | (-73, -48) | 180° | - | Contact loader tube |
| 38 | **Collect balls** | (-73, -48) | 180° | **1800ms** | All 6 balls from tube |
| 39 | Stop intake, open flap | - | 180° | 100ms | Prepare for scoring |
| 40 | Drive backward | (-31, -48) | 180° | - | Left long goal (bottom) |
| 41 | **Score top goal** | (-31, -48) | 180° | 1200ms | Flap open, balls fall |
| 42 | Stop all mechanisms | - | 180° | - | Safety |

**Subtotal: +6 balls, +5-6 points scored**

**LEFT SIDE COMPLETE: 24 balls, ~19 points, ~25 seconds**

---

## RIGHT SIDE (Blue Alliance Side - Mirror of Left)

### Phase R1: Bottom-Right Loader Tube + Long Goal
| Step | Action | Target (Jerry) | Heading | Duration | Notes |
|------|--------|----------------|---------|----------|-------|
| 43 | Turn | - | 45° | - | Clear left goal |
| 44 | Intake storage mode | - | 45° | - | Start collecting |
| 45 | Drive forward | (-24, -36) | 45° | - | Waypoint 1: clear goal |
| 46 | Turn | - | 0° | - | Face right |
| 47 | Drive forward | (24, -36) | 0° | - | Waypoint 2: cross field |
| 48 | Drive forward | (48, -48) | 0° | - | Retreat bottom-right |
| 49 | Turn (none) | - | 0° | - | Already facing loader |
| 50 | Deploy loader | - | 0° | 250ms | Extend pneumatic |
| 51 | Drive forward | (73, -48) | 0° | - | Contact loader tube |
| 52 | **Collect balls** | (73, -48) | 0° | **1800ms** | All 6 balls from tube |
| 53 | Stop intake, open flap | - | 0° | 100ms | Prepare for scoring |
| 54 | Drive backward | (31, -48) | 0° | - | Right long goal (bottom) |
| 55 | **Score top goal** | (31, -48) | 0° | 1200ms | Flap open, balls fall |
| 56 | Stop all mechanisms | - | 0° | - | Safety |

**Subtotal: +6 balls, +5-6 points scored**

---

### Phase R2: Bottom-Right Cluster Tube + Center Middle Goal
| Step | Action | Target (Jerry) | Heading | Duration | Notes |
|------|--------|----------------|---------|----------|-------|
| 57 | Turn | - | 90° | - | Face up |
| 58 | Intake storage mode | - | 90° | - | Start collecting |
| 59 | Drive forward | (24, -24) | 90° | - | Bottom-right cluster |
| 60 | **Collect balls** | (24, -24) | 90° | **1800ms** | All 6 balls from tube |
| 61 | Turn | - | 45° | - | Face center |
| 62 | Drive backward | (9, -9) | 45° | - | Center middle right |
| 63 | **Score mid goal** | (9, -9) | 45° | 1000ms | Indexer reverse |
| 64 | Stop all mechanisms | - | 45° | - | Safety |

**Subtotal: +6 balls, +5 points scored**

---

### Phase R3: Top-Right Cluster Tube + Center Lower Goal
| Step | Action | Target (Jerry) | Heading | Duration | Notes |
|------|--------|----------------|---------|----------|-------|
| 65 | Delay | - | 45° | 100ms | Settling |
| 66 | Turn | - | 135° | - | Face waypoint |
| 67 | Intake storage mode | - | 135° | - | Start collecting |
| 68 | Drive forward | (24, 0) | 135° | - | Waypoint |
| 69 | Turn | - | 90° | - | Face up |
| 70 | Drive forward | (21, 21) | 90° | - | Top-right cluster |
| 71 | **Collect balls** | (21, 21) | 90° | **1800ms** | All 6 balls from tube |
| 72 | Turn | - | 45° | - | Face center |
| 73 | Drive forward | (8.3, 9.7) | 45° | - | Center lower right |
| 74 | **Score low goal** | (8.3, 9.7) | 45° | 1300ms | Intake reverse |
| 75 | Stop all mechanisms | - | 45° | - | Safety |

**Subtotal: +6 balls, +3 points scored**

---

### Phase R4: Top-Right Loader Tube + Long Goal
| Step | Action | Target (Jerry) | Heading | Duration | Notes |
|------|--------|----------------|---------|----------|-------|
| 76 | Turn | - | 90° | - | Face up |
| 77 | Intake storage mode | - | 90° | - | Prepare |
| 78 | Drive forward | (48, 48) | 90° | - | Retreat top-right |
| 79 | Turn | - | 0° | - | Face loader |
| 80 | Deploy loader | - | 0° | 250ms | Extend pneumatic |
| 81 | Drive forward | (73, 48) | 0° | - | Contact loader tube |
| 82 | **Collect balls** | (73, 48) | 0° | **1800ms** | All 6 balls from tube |
| 83 | Stop intake, open flap | - | 0° | 100ms | Prepare for scoring |
| 84 | Drive backward | (31, 48) | 0° | - | Right long goal (top) |
| 85 | **Score top goal** | (31, 48) | 0° | 1200ms | Flap open, balls fall |
| 86 | Stop all mechanisms | - | 0° | - | Safety |

**Subtotal: +6 balls, +5-6 points scored**

**RIGHT SIDE COMPLETE: 24 balls, ~19 points, ~25 seconds**

---

## Cycle Transition

| Step | Action | Target (Jerry) | Heading | Duration | Notes |
|------|--------|----------------|---------|----------|-------|
| 87 | Turn | - | 135° | - | Clear right goal |
| 88 | Intake storage mode | - | 135° | - | Prepare next cycle |
| 89 | Drive forward | (24, 36) | 135° | - | Waypoint 1: clear goal |
| 90 | Turn | - | 180° | - | Face left |
| 91 | Drive forward | (-24, 36) | 180° | - | Waypoint 2: cross field |
| 92 | Drive forward | (-48, 48) | 180° | - | Return to retreat point |
| 93 | Check time | - | - | - | If >30s remain, loop to step 4 |

---

## Parking (Final Stage)

When less than 30 seconds remain:

| Step | Action | Target (Jerry) | Heading | Duration | Notes |
|------|--------|----------------|---------|----------|-------|
| 94 | Turn | - | 180° | - | Face parking zone |
| 95 | Drive forward | (-48, -48) | 180° | - | Enter red negative zone |
| 96 | Stop | (-48, -48) | 180° | - | **+8 parking bonus** |

---

## Scoring Summary (Per Cycle)

| Category | Count | Points Each | Total |
|----------|-------|-------------|-------|
| Loader tubes (4) | 24 balls | 1 pt | 24 pts |
| Cluster tubes (4) | 24 balls | 1 pt | 24 pts |
| Long goals (4) | 4 scores | 5-6 pts | 20-24 pts |
| Center mid goals (2) | 2 scores | 5 pts | 10 pts |
| Center low goals (2) | 2 scores | 3 pts | 6 pts |
| **Cycle Total** | - | - | **84-88 pts** |
| Parking bonus | - | - | +8 pts |
| **Grand Total** | - | - | **92-96 pts** |

---

## Timing Breakdown

| Phase | Steps | Est. Time |
|-------|-------|-----------|
| L1: Top-left loader | 1-8 | ~6s |
| L2: Top-left cluster | 9-16 | ~5s |
| L3: Bottom-left cluster | 17-27 | ~6s |
| L4: Bottom-left loader | 28-39 | ~7s |
| **Left side subtotal** | | **~24s** |
| R1: Bottom-right loader | 40-50 | ~6s |
| R2: Bottom-right cluster | 51-58 | ~5s |
| R3: Top-right cluster | 59-69 | ~6s |
| R4: Top-right loader | 70-80 | ~7s |
| **Right side subtotal** | | **~24s** |
| Transition | 81-84 | ~3s |
| **Full cycle** | | **~51s** |
| Parking | 85-87 | ~4s |

---

## Configuration

**Timing constants:**
```cpp
skills_cluster_collect_ms = 1800  // Wait for all 6 balls
skills_tube_pull_ms = 1800        // Wait for all 6 balls
skills_loader_deploy_ms = 250
skills_low_score_ms = 1300
skills_mid_score_ms = 1000
skills_top_score_ms = 1200
```

**Speed settings:**
```cpp
turn_maxSpeed = 100
turn_minSpeed = 10
drive_maxSpeed = 100
```

---

## Notes

- **Starting position:** Same as Red Left match autonomous (Jerry -46.5, 13)
- **First cycle start:** Turn left 90°, drive to retreat point, turn left 90° to face loader
- Path is symmetric: left side (red) mirrors to right side (blue)
- Each tube contains 6 balls that fall over ~1.8 seconds
- Robot collects while balls are falling (red + blue both count)
- All 8 corner tubes are visited per cycle
- All 4 center goals are scored from both sides (8 scoring opportunities)
- Loop continues until 30 seconds remain for parking
- Expected 1 full cycle + parking in 60 seconds = 92-96 points
