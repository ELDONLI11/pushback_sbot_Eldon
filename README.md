# Pushback S-bot – Controls, Autonomous, and Design Overview

## Quick Driver Cheat Sheet (Match Use)
- Drive: Left stick Y = left side, Right stick Y = right side
- Storage/Intake: R1 (toggle) – intake + helper + indexer forward (flap forced closed)
- Top Goal: R2 (toggle) – intake + helper + indexer forward (flap forced open)
- Mid Goal: L1 (toggle) – continuous reverse indexer (intake off)
- Low Goal: L2 (toggle) – continuous reverse intake (indexer off)
- Sorting: Y – toggle auto color sort on/off
- Alliance Color: D-pad UP = RED, D-pad DOWN = BLUE
- Flap/Descore Piston: A – toggle open/closed (latched; R2 still opens flap while held)
- Match Loader Piston: B – toggle extend/retract (latched)

## Robot Overview
- Project: pushback_sbot (current / active robot)
- Drive: 6‑motor tank drive (3 per side)
- Mechanisms:
  - Dual‑motor intake (main + helper)
  - Single indexer feeding a vertical tower
  - Batch loader piston (stages balls)
  - Goal flap piston (opens path to high goal)
  - Single color sensor for alliance‑aware sorting

All new work and documentation should target this project; the original `pushback` project is legacy and should not be changed further.

---

## Controller Controls (Driver)

### Driving
- Left stick Y: Left side drive (tank)
- Right stick Y: Right side drive (tank)

### Ball Collection & Scoring
- R1 – **Storage / Intake Mode** (toggle)
  - Intake main + helper motors run forward (pull balls in)
  - Indexer feeds forward toward the top of the tower
  - Goal flap is **forced closed** (pistons down) so balls stage inside the robot
- R2 – **Top Goal Mode** (toggle)
  - Intake main + helper motors run forward
  - Indexer feeds forward
  - Goal flap is **forced open** to send balls up into the high goal
- L1 – **Middle Goal Mode** (toggle)
  - Continuous reverse indexer to drop/eject balls into the middle goal
  - Intake is **off** so only the indexer moves
- L2 – **Low Goal Mode** (toggle)
  - Continuous reverse intake (low‑goal spit)
  - Indexer is **off** and only the intake motors reverse

Note: these scoring/collection modes are mutually exclusive; pressing one turns the others off. Press the same button again to stop that mode.

### Alliance Color & Sorting
- Y – **Toggle Color Sorting On/Off**
  - When **enabled**, the color sensor watches balls at the indexer and ejects “wrong‑color” balls automatically.
  - When **disabled**, indexer runs normally (no auto eject).
- D-pad UP – **Set Alliance Color = RED**
- D-pad DOWN – **Set Alliance Color = BLUE**

Color sorting behavior:
- Uses optical sensor hue:
  - “Red”: hue ≈ < 30° or > 330°
  - “Blue”: hue ≈ 180–260° (tunable)
- If sorting is enabled and an **opponent‑colored** ball is seen while indexer is feeding forward:
  - Indexer is forced into a short **reverse‑eject** window (about `SBOT_COLOR_EJECT_TIME_MS`), then returns to normal.

### Pneumatics
- Batch loader piston
  - Default at start: **retracted** (staging mechanism pulled in)
  - **Driver control**: Button **B** toggles between extended and retracted.
- Goal flap piston
  - Default at start: **closed**.
  - **Driver control**: Button **A** toggles a latched open/closed state.
  - When **Top Goal Mode** (R2 toggle) is active, the flap is forced **open** regardless of the A-toggle.
  - When **Storage / Intake Mode** (R1 toggle) is active, the flap is forced **closed** regardless of the A-toggle.

---

## Autonomous & Test Selections

Autonomous and test modes for S‑bot are selected **through the same controller‑based selector**, just like the earlier robot: there is **one unified autonomous selector**, and some entries are “test” modes.

Development workflow (matches the old project behavior):
- If the robot is **not connected** to a competition switch/FMS, S-bot opens a short selection window at startup and (if confirmed) will **run the selected mode immediately** for quick testing.
- During driver control you can **hold R1+R2** to temporarily enter autonomous selection (then use D-pad LEFT/RIGHT + A). Release R1+R2 to return to driving.

### Selector Controls (while disabled)
- D‑pad RIGHT: Next mode (`SBOT_AUTO_NEXT_BTN`)
- D‑pad LEFT: Previous mode (`SBOT_AUTO_PREV_BTN`)
- A: Confirm / toggle confirm (`SBOT_AUTO_CONFIRM_BTN`)

Behavior:
- While **not confirmed**:
  - LEFT/RIGHT cycle through numeric modes 0–9.
  - A confirms the currently shown mode.
- When **confirmed**:
  - Display shows `READY: <mode name>`.
  - Pressing A again drops back into selection so you can change it.

### Mode List (SbotAutoMode)
`enum class SbotAutoMode {`  
`  DISABLED = 0,`  
`  RED_LEFT,`  
`  RED_RIGHT,`  
`  BLUE_LEFT,`  
`  BLUE_RIGHT,`  
`  SKILLS,`  
`  TEST_DRIVE,`  
`  TEST_TURN,`  
`  TEST_INTAKE,`  
`  TEST_INDEXER`  
`};`

### What Each Mode Does (Current State)
- **0 – DISABLED**
  - Robot does nothing in autonomous.
- **1 – RED_LEFT**
  - Stub for a full Red‑left match autonomous (currently just prints to the V5 screen).
- **2 – RED_RIGHT**
  - Stub for a full Red‑right match autonomous.
- **3 – BLUE_LEFT**
  - Stub for a full Blue‑left match autonomous.
- **4 – BLUE_RIGHT**
  - Stub for a full Blue‑right match autonomous.
- **5 – SKILLS**
  - Stub for a Programming Skills routine.
- **6 – TEST_DRIVE** (autonomous test)
  - Uses LemLib to:
    - Set pose to (0,0,0)
    - Drive forward to a point (e.g., 24 in)
    - Drive back to the origin
  - Purpose: verify drive motors, odometry, and LemLib motion tuning.
- **7 – TEST_TURN** (autonomous test)
  - Uses LemLib to:
    - Turn to 90°
    - Return to 0°
  - Purpose: verify turning PID and gyro.
- **8 – TEST_INTAKE** (autonomous test)
  - Stub for an intake‑only test routine (currently only logs text).
- **9 – TEST_INDEXER** (autonomous test)
  - Stub for an indexer‑only test routine (currently only logs text).

### Answer to Your Question
Yes – for **pushback_sbot** we are still using the **same general approach as before**:
- A **single autonomous selector** on the controller.
- The selector list includes both **match autonomous routes** and **test modes** (drive/turn/intake/indexer).
- Test selections are picked the same way as normal autonomous modes (via D‑pad + A while disabled).

---

## Current Design Approach (High Level)

- **Single responsibility subsystems**
  - `SbotDrivetrain` handles all tank‑drive motion and joystick processing.
  - `SbotIntake` abstracts main + helper intake motors behind an `IntakeMode` enum.
  - `SbotIndexer` abstracts the indexer motor with an `IndexerMode` enum and shared speed constants.
  - `BatchLoaderPiston` and `GoalFlapPiston` wrap individual pistons and hide ADI wiring details.
  - `SbotColorSensorSystem` is the only component that reads the color sensor and enforces sorting behavior.
  - `SbotAutonomousSystem` owns the mode selector and routes.

- **Configuration‑driven wiring**
  - All ports, gearsets, and button mappings live in `include/config_sbot.h`.
  - Logic code (`src/*.cpp`) relies on these defines so wiring / control changes only need edits in one place.

- **Simple driver loop**
  - `src/main.cpp` creates all subsystems once in `initialize()` and keeps a tight `opcontrol()` loop:
    - Read controller
    - Set subsystem modes (intake/indexer/pistons/color)
    - Call each subsystem’s `update()`

- **Autonomous test integration**
  - Instead of a separate test harness, `SbotAutonomousSystem` includes simple test entries in the selector.
  - This keeps test selection flow identical to match autonomous selection.

---

## Maintenance Rule
Whenever you change **controls, ports, or autonomous/test modes** in `pushback_sbot`:
- Update `include/config_sbot.h` first.
- Then update this `README.md` so driver and testing docs **stay in sync**.

Suggested checklist before committing sbot changes:
- [ ] Controls / buttons still match this README
- [ ] Any new autonomous or test mode is added to the **Mode List** and **What Each Mode Does** sections
- [ ] Any new mechanism or piston is described in **Controller Controls** or **Pneumatics**

---

## Autonomous Implementation Plan (S-bot)

Goal: implement **5 autonomous routines** in `SbotAutoMode`:
- 4 match autos: `RED_LEFT`, `RED_RIGHT`, `BLUE_LEFT`, `BLUE_RIGHT`
- 1 skills auto: `SKILLS`

Constraint (intentional): keep the **existing selector UX** (modes 0–9). We will evolve behavior inside these modes rather than adding new modes.

### Start Alignment (Match Autos)
All 4 match autos assume a *repeatable physical start*:
- Place the robot with its **back bumper touching the black park-zone strip** at the **far end of your alliance goal** (the “furthest part of the goal” you called out).
- Robot starts **square** to that surface (no intentional angle).
- In code, this physical placement is defined as LemLib pose **(0, 0, 0°)**.
  - **0°** means the robot faces *away from the goal* into the field.

Left vs Right starts:
- `RED_RIGHT` / `BLUE_RIGHT` are the “right-side” start using the above pose directly.
- `RED_LEFT` / `BLUE_LEFT` reuse the same routine but **mirror Y** (so it’s the same path on the other side).

### Phase 0 — Setup + Safety (same day)
- Ensure LemLib initializes and calibrates once (`initializeSbotLemLib()` is called in `SbotAutonomousSystem::initialize()`).
- Confirm odom pod and IMU are wired correctly (ports in `include/config_sbot.h`).
- Add “safe start” defaults at the beginning of every routine:
  - stop intake/indexer
  - retract batch loader, close goal flap
  - disable color sorting (optional, but recommended during autonomous)

Acceptance:
- Running any autonomous mode never leaves motors stuck on after completion.

### Phase 1 — Tiny Calibration Autos (1–2 sessions)
Implement and repeatedly run the built-in test modes:
- **TEST_DRIVE**: forward/back to a known distance and print pose checkpoints.
- **TEST_TURN**: turn to headings and print pose checkpoints.
- **TEST_INTAKE**: run intake forward for a short window, then stop.
- **TEST_INDEXER**: feed forward, then reverse briefly, then stop.

Acceptance:
- Drive test returns close to the origin consistently.
- Turn test returns close to the starting heading consistently.
- Intake/indexer tests are direction-correct and stop reliably.

### Phase 2 — Movement Tuning (2–4 sessions)
Tune these in `include/lemlib_config_sbot.h` / `src/lemlib_config_sbot.cpp`:
- `SBOT_DRIVE_TRACK_WIDTH`
- `SBOT_TRACKING_WHEEL_DISTANCE` (if the tracking wheel is not exactly at the rotation center)
- `sbot_linear_controller` and `sbot_angular_controller` gains

Suggested tuning loop:
1. Run **TEST_TURN** until 90°/0° is repeatable.
2. Run **TEST_DRIVE** until forward/back is repeatable.
3. Re-run both after any mechanical change.

Acceptance:
- 5 consecutive runs: heading error is small and does not drift badly.
- 5 consecutive runs: forward/back error is small and consistent.

### Phase 3 — Implement Common Autonomous Primitives (same as Phase 2)
Inside `src/autonomous_sbot.cpp`, create internal helpers so all autos share the same behavior:
- `driveTo(x, y, timeout)` wrapper + `waitUntilDone()`
- `turnTo(heading, timeout)` wrapper
- timed actions: `intakeFor(ms)`, `indexerFor(ms)`
- scoring helpers:
  - “mid goal score” (reverse indexer for `SBOT_MID_GOAL_SCORE_TIME_MS`)
  - “top goal feed” (forward indexer + open flap for a short window)
- coordinate transform helper so BLUE routines can reuse RED logic (rotate 180° around the start pose)

Acceptance:
- The 4 match autos differ only by a small set of parameters (side/alliance), not copy/pasted logic.

### Phase 4 — Implement the 4 Match Autos (4–8 sessions)
Use the “route phases” from the legacy docs as the skeleton:
1. Start intake and collect the first blocks.
2. Turn/position for mid-goal score and execute a short score window.
3. Do a small bump/retreat (field control + clearance).
4. Reposition toward match-load / next pickup.
5. Final approach + top-goal feed window.

Implementation detail (current approach):
- Match autos are implemented as **one shared routine** based on **Red Right (steps 1→5)**.
- `RED_LEFT` is the mirrored version.
- `BLUE_LEFT/BLUE_RIGHT` are treated as a **180° rotated** version of the same routine.
- This means tuning the step distances/headings in one place updates all 4 match autos.

Acceptance:
- Each routine finishes within 15 seconds with timeouts.
- Routines are robust to small placement error (don’t require perfect alignment).

### Phase 5 — Skills Auto (4–10 sessions)
Implement `SKILLS` as a repeatable “cycle” loop:
- collect → score → reposition → repeat
- always include timeouts and a hard stop near the end

Acceptance:
- Runs end-to-end without operator interaction.
- No single motion can hang the routine (timeouts + cancel/stop).

### Logging & On-Robot Checklist
During development, prefer simple terminal prints so it’s obvious what phase you’re in:
- print start + end markers per routine
- print key checkpoints (after each move/turn/score)

Before competition:
- [ ] Confirm the correct match auto is selected and shows `READY` while disabled
- [ ] IMU has finished calibrating before the match starts
- [ ] Odom wheel spins freely and reads correctly
- [ ] Intake/indexer directions are correct

