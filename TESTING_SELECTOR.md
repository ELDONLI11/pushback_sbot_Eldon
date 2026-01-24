# Testing Autonomous Selector Without Competition Hardware

## Option 1: FREE - Software Simulation (Recommended)

**No hardware needed!** Use PROS built-in competition control simulation.

### Method A: Code-based simulation

In [src/main.cpp](src/main.cpp), uncomment the competition control simulation block (around line 80):

```cpp
printf("\n=== ENABLING COMPETITION CONTROL SIMULATION ===\n");
pros::competition::initialize();  // Enable competition mode
return;
```

**What happens:**
1. Robot starts in disabled mode
2. `disabled()` runs → selector appears on controller
3. Use D-pad Left/Right + A to select autonomous
4. Press **Y button** on controller to transition to autonomous
5. `autonomous()` runs with your selected mode
6. Automatically transitions to driver control

### Method B: PROS Terminal Commands

Upload your code normally, then in PROS terminal:

```bash
pros terminal
# Then type in the terminal:
competition enable
```

This enables competition mode without code changes.

---

## Option 2: Cheap Hardware - VEX Competition Switch

**Cost: ~$15-20 USD**

**Product:** VEX V5 Competition Switch  
**Part Number:** 276-4810  
**URL:** https://www.vexrobotics.com/276-4810.html

**What it does:**
- Plugs into V5 Brain "Competition Port" (smart port labeled "COMP")
- Has 3 positions: Disabled / Autonomous / Driver Control
- Simulates official field controller behavior

**How to use:**
1. Plug switch into brain COMP port
2. Set to "Disabled" position
3. Controller selector appears
4. Select autonomous with D-pad + A
5. Flip switch to "Autonomous" 
6. Robot runs selected autonomous
7. Flip to "Driver" for driver control

---

## Option 3: Controller Button Test (No Competition Mode)

Your code already supports this in development mode (no competition control connected):

1. Upload code normally
2. Power on robot (no competition switch)
3. You have **10 seconds** to select autonomous using D-pad Left/Right + A
4. Selected autonomous runs immediately
5. Then transitions to driver control

Check terminal output for: `"SBOT: development mode (no competition control)"`

---

## Troubleshooting

**Controller screen stays blank:**
- Check terminal for `"SELECTOR: Displaying selection 0 - 'DISABLED'"`
- If you see `"controller not connected"` → pair controller with brain
- Try power cycling the controller

**Selector doesn't respond to buttons:**
- Terminal should show `"SELECTOR: LEFT/RIGHT button pressed"` when you press D-pad
- If no logs appear → controller not paired or button mapping issue

**"competition control simulation" doesn't work:**
- Make sure you uncommented the code AND added `return;` statement
- The return is critical - it exits initialize() and lets PROS handle the rest

---

## Recommendation

**Use Option 1 Method A** (competition control simulation) - it's free and most accurately simulates match conditions.
