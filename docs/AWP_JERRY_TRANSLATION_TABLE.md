# AWP Half Auton — Jerry Translation Table (Pushback SBOT)

This file summarizes the **Red Left (canonical)** AWP-half autonomous steps, listing the **Jerry field points (inches)** that were provided and the **internal LemLib start-relative targets** the code uses.

## Coordinate conversion used (as implemented in code)

Jerry inputs were provided with the **gyro location** as the start reference:
- Jerry start (gyro): `(-51, 15)`

Axes (confirmed):
- Into-field: Jerry **X increases** (less negative)
- Robot-right: Jerry **Y decreases**

Conversion (gyro-origin internal frame):
- `ourX = (Jy_start - Jy)`
- `ourY_gyro = (Jx - Jx_start)`

Pose-origin correction:
- LemLib pose is the drivetrain rotation center.
- Gyro is **4.5" behind** the pose point.
- So we shift: `ourY_pose = ourY_gyro - 4.5`

## Red Left (canonical) step table

Legend:
- **Internal (pose-origin)** values are what the tuning stores/uses.
- Some steps use a **contact point** and then compute a pose target from it using bumper offsets.

| Stage | What the robot does | Jerry point(s) (in) | Internal (gyro-origin) | Internal (pose-origin) | Notes (what code targets) |
|---|---|---|---|---|---|
| Start | Start pose | Start gyro: `(-51, 15)` | Defined as `(0,0)` by odom reset | Defined as `(0,0)` by odom reset | `sbot_zero_pose_and_sensors(0,0,0)` sets this as origin. |
| 0 | Clear barrier | (not from Jerry) | — | — | Drives forward `clear_barrier_in = 10"`. |
| 1 | Collect cluster | Cluster: `(-24, 24)` | `ourX = 15-24=-9`, `ourY= -24 -(-51)=27` → `(-9, 27)` | `(-9, 22.5)` | Code uses `cluster1 = (-9, 27-4.5)` and a small sweep forward. |
| 2 (RL) | Score **Center Goal – Lower** (front) | **Contact**: `(-9, 9)` | `ourX=15-9=6`, `ourY= -9 -(-51)=42` → `(6, 42)` | `(6, 37.5)` | Code converts this **front contact point** to a **pose target** using heading `45°` and front bumper `7.5"`. Then it runs `sbot_score_low_for(...)` and **waits 0.5s** to let balls finish exiting. |
| 3 | Retreat to measured point | Retreat: `(-48, 48)` | `ourX=15-48=-33`, `ourY= -48 -(-51)=3` → `(-33, 3)` | `(-33, -1.5)` | Code now drives to this point with `moveToPoint(..., forwards=false)` (backs in), then turns to face loader. |
| 4 (RL) | Loader pull (tube) | **Contact**: `(-71, 48)` | `ourX=15-48=-33`, `ourY= -71 -(-51)=-20` → `(-33, -20)` | `(-33, -24.5)` | Code deploys loader first. Then converts this **front contact point** to a pose target using heading `180°` and **effective front reach** `(7.5 + 6.0) = 13.5"`. |
| 5 | Back into Long Goal end | (not provided yet) | — | — | Currently distance-based: backs `48"` from the loader-facing posture (contact-based long-goal end is disabled until a Jerry contact point is provided). |

## Red Right / Blue variants

- The code treats **Red Left** as canonical.
- **Red Right** is derived by mirroring X (and mirroring headings), then overriding Stage 2 to use **Center Goal – Middle** (back score).
- **Blue** variants rotate 180°.

Center Goal – Middle (used for Red Right / Blue Left):
- Jerry **back contact**: `(-9, -9)`
- Internal (gyro-origin): `ourX = 15 - (-9) = 24`, `ourY = -9 - (-51) = 42` → `(24, 42)`
- Internal (pose-origin): `(24, 37.5)`
- Code converts this **back contact point** into a **pose target** using heading `180°` and back bumper `7.5"`.

## What to provide next (optional)

If you want Stage 5 to be fully contact-based (instead of “back 48 inches”), provide:
- A Jerry point for the **Long Goal end contact** (where the robot’s **back bumper** should touch when scored).
