# Testing Autonomous Selector Without Competition Hardware

The autonomous selector uses **RoboDash** on the V5 Brain touchscreen.
You pick a route by tapping on the brain screen — no controller D-pad needed.

---

## Option 1: Development Mode (No Hardware Needed)

When no competition switch or field controller is connected, `opcontrol()` automatically
enters **dev mode** before normal driver control starts.

### How it works

1. Upload code normally (`pros upload`)
2. Power on robot (no competition switch plugged in)
3. The RoboDash selector appears on the **brain touchscreen**
4. Tap the autonomous route you want on the brain screen
5. Press **Y** on the controller to run the selected autonomous
6. Or press **DOWN** on the controller to skip straight to driver control
7. After autonomous completes (or is skipped), normal driver control begins

### What you'll see in the terminal

```
SBOT: development mode (no competition control)
SBOT: Select autonomous on brain touchscreen
SBOT: Press Y to run, DOWN to skip to driver control
```

Then either:
```
SBOT: DEV MODE - running selected autonomous
=== SBOT AUTONOMOUS START ===
...
SBOT: DEV MODE - autonomous complete
```
or:
```
SBOT: DEV MODE - skipping autonomous
```

### Controller screen shows

```
DEV: select on brain
Y=run  DOWN=skip
```

---

## Option 2: VEX Competition Switch (~$15-20)

**Product:** VEX V5 Competition Switch
**Part Number:** 276-4810
**URL:** https://www.vexrobotics.com/276-4810.html

Plugs into the V5 Brain "Competition Port" (smart port labeled "COMP").
Has 3 positions: Disabled / Autonomous / Driver Control.

### How to use

1. Plug switch into brain COMP port
2. Set to **Disabled** position
3. RoboDash selector appears on the brain touchscreen (via `disabled()`)
4. Tap the autonomous route you want on the brain screen
5. Flip switch to **Autonomous** → robot runs the selected autonomous
6. Flip to **Driver** → normal driver control

This most closely simulates match conditions.

---

## How it works (code flow)

### Competition mode (switch/field connected)

```
initialize() → disabled()          → autonomous()       → opcontrol()
                 ↓                      ↓
              selector.focus()      selector.run_auton()
              (pick on brain)       (runs selected route)
```

### Dev mode (no competition control)

```
initialize() → opcontrol()
                 ↓
              selector.focus()       // shows RoboDash on brain
              wait for Y or DOWN     // Y = run auton, DOWN = skip
              selector.run_auton()   // if Y pressed
              driver control loop    // normal driving
```

---

## Troubleshooting

**Brain screen is blank / no selector:**
- Check terminal for `SBOT: RoboDash selector focused on brain screen` (competition mode)
  or `SBOT: development mode (no competition control)` (dev mode)
- Make sure RoboDash library is installed (`pros conductor fetch robodash@2.3.1`)

**Controller screen stays blank:**
- Pair controller with brain (Settings → Controller on brain)
- Try power cycling the controller

**Y button does nothing in dev mode:**
- Make sure you're in dev mode (no competition switch plugged in)
- Check terminal output — if you see `opcontrol alive`, you're already past the selection step

**Autonomous runs but robot doesn't move:**
- Check terminal for motor/sensor errors
- Verify LemLib initialized: look for `SBOT: LemLib initialized` in terminal
