# AWP Autonomous (Our-Side) – Path Table (SBOT)

## Path Diagram (generated)

- Planned path diagram (from current tuning points): [docs/diagrams/awp_auton_paths.svg](docs/diagrams/awp_auton_paths.svg)

This document describes the intended **step-by-step autonomous** for achieving “our share” of the AWP requirements, across all four start positions.

## Coordinate / Starting Assumptions

- The auton uses a **start-relative** coordinate system.
- At auton start, the robot pose is set to `(x=0, y=0, θ=0°)`.
- **0°** means the robot is pointed **straight into the field**.
- Place the robot consistently (square to field), and start with the robot **not binding on the Park Zone barrier**. The auton immediately drives forward to clear the barrier.

Implementation note:

- The auton uses **two separate canonical tunings** (Red Left and Red Right). This avoids incorrect “mirror” assumptions because goal types/locations differ between left and right halves.
- Blue-side routines are derived by a **180° rotation** of the corresponding Red routine.

## Scoring Direction Assumptions

- **Front intake** is used for collection.
- **Center Goal – Lower (front score)** is implemented as **intake reverse only** (matches driver control).
- **Center Goal – Middle (back score)** is implemented as the **middle-goal routine** (intake assists, indexer runs reverse-middle). The approach is **backing into** the goal so the robot’s rear is against the goal.

Waypoint note:

- The **Center Goal – Lower approach point** (used for **Red Left / Blue Right**) is currently derived from the cluster as: `center_lower_approach = cluster1 + (18", 18")` (forward-right, 0.75 tile diagonal).
- The **Center Goal – Middle approach point** (used for **Red Right / Blue Left**) is a **separate tuning** and should not be assumed identical to Center-Lower.

## Behavior Matrix (4 start positions)

The field is asymmetric for “our half” in *approach*, even though the target Goal is the same:

- **Red Left + Blue Right**: first score is **Center Goal – Lower** via **front score**
- **Red Right + Blue Left**: first score is **Center Goal – Middle** via **back score**

| Start Position | Step 0 | Step 1 | Step 2 (First Score) | Step 3 | Step 4 | Step 5 (Final Score) | End |
|---|---|---|---|---|---|---|---|
| Red Left | Clear barrier | Collect center cluster | **Center Goal – Lower**: turn + drive in **forward** + **front score** | Retreat: **forced absolute point** | Face loader + load | Drive **back 2 tiles** into Long Goal end (no X change) + score | Drive to end-safe point |
| Red Right | Clear barrier | Collect center cluster | **Center Goal – Middle**: turn + drive in **backwards** + **back score** | Retreat straight back-out | Turn + load loader | Score near end of Long Goal (adjacent to loader) | Drive to end-safe point |
| Blue Left | Clear barrier | Collect center cluster | **Center Goal – Middle**: turn + drive in **backwards** + **back score** | Retreat straight back-out | Turn + load loader | Score near end of Long Goal (adjacent to loader) | Drive to end-safe point |
| Blue Right | Clear barrier | Collect center cluster | **Center Goal – Lower**: turn + drive in **forward** + **front score** | Retreat: **forced absolute point** | Face loader + load | Drive **back 2 tiles** into Long Goal end (no X change) + score | Drive to end-safe point |

Notes:

- “Collect center cluster” means driving through the cluster using intake/storage.
- “Tube load” in our earlier wording means **Loader** (manual term).
- **Solo AWP modes** perform **Tube 2** immediately after Tube 1, then score the **nearest High/Top goal**.
- On **Red Left / Blue Right**, the final goal is the **near end of the Long Goal adjacent to Loader 1**. The intended finish is: get onto the tube/long-goal X line, then **drive straight (no X change)** between tube and long-goal end.

## Where this is implemented

- Match auto entry point: [src/autonomous_sbot.cpp](src/autonomous_sbot.cpp)
- Logic branch for LOW vs MIDDLE: `low_goal_case` inside the match routine.
- Low-goal scoring helper: `sbot_score_low_for(ms)`
- Middle-goal scoring helper: `sbot_score_mid_for(ms)`

## Tuning Checklist (what to adjust on-field)

All of these are “first-pass placeholders” and should be tuned while watching robot behavior:

- `cluster1`, `cluster1_sweep` (cluster collection path)
- Center Goal – Lower approach offset (currently `cluster1 + (18", 18")`)
- `low_goal_approach`, `low_goal_heading_deg`
- `mid_goal_approach`, `mid_goal_heading_deg`
- Post-score retreat: for RL/BR we use `post_score_retreat_point` (forced absolute point). Other starts use `post_score_retreat_back_dist_in`.
- `tube1` and `tube1_pulloff`
- `end_safe`
