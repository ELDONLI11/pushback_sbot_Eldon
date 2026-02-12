# Tournament Backup Slots - Quick Guide

This folder contains PowerShell scripts to create backup autonomous slots for tournament use.

## The Problem

Your default code uses a controller-based autonomous selector (D-pad Left/Right + A to choose modes). **If the selector fails during a tournament** (controller connection issues, timing problems, etc.), you need a backup plan.

## The Solution

Upload **5 program slots** to your V5 brain:

- **Slot 1**: Default with selector working (try this first!)
- **Slot 2**: leftmid - Red/Blue Left hardcoded (scores Middle Goal)
- **Slot 3**: rightlow - Red/Blue Right hardcoded (scores Low Goal)
- **Slot 4**: leftsolo - Solo AWP hardcoded
- **Slot 5**: skills - Skills autonomous hardcoded

## Before Tournament

### Upload All 5 Slots

Open PROS Terminal in VS Code and run:

```powershell
.\upload-backup-slots.ps1
```

This will:
1. Upload Slot 1 with normal selector (unmodified code)
2. Build and upload Slots 2-5 with hardcoded autonomous modes
3. Takes about 3-4 minutes total
4. **Automatically restores your original code after EVERY upload** (even if one fails)

**Optional**: If you want to specify a brain name:
```powershell
.\upload-backup-slots.ps1 -BrainName "SBOT"
```

## During Tournament

### Normal Match (Selector Works)

1. **Select Slot 1** (sbot-selector)
2. Robot boots → controller shows selector
3. Use **D-pad Left/Right + A** to choose:
   - Press **RIGHT** for Red Right / Blue Right (Low Goal)
   - Use **LEFT** (default) for Red Left / Blue Left (Middle Goal)
   - Keep pressing RIGHT for Solo AWP or Skills
4. Press **A** to confirm
5. Match starts → your selected autonomous runs

### Emergency Fallback (Selector Fails)

If the selector doesn't show or doesn't respond:

1. **Before the match**: Switch to the appropriate backup slot:
   - **Use Slot 2** (leftmid) for Red Left or Blue Left
   - **Use Slot 3** (rightlow) for Red Right or Blue Right
   - **Use Slot 4** (leftsolo) for Solo AWP
   - **Use Slot 5** (skills) for Skills
   
2. **No button input needed** - autonomous runs immediately when match starts

## How It Works

### The Script Process

For each slot, the script:

1. **Backs up original** → `src/main.cpp.backup`
2. **Modifies code** (or keeps original for Slot 1)
3. **Builds project** with `pros make`
4. **Uploads to slot** with `pros upload --slot X`
5. **Restores original** → Even if upload fails!

The restore happens in a `finally` block, which means **your code is ALWAYS restored**, even if:
- The build fails
- The upload fails  
- You cancel with Ctrl+C
- Any error occurs

**Your source code is never permanently changed.**

### Code Modification Example

For Slot 2 (leftmid), the script changes `src/main.cpp`:

```cpp
void autonomous() {
    printf("=== SBOT AUTONOMOUS START ===\n");
    selector.run_auton();  // RoboDash selector
}
```

To:

```cpp
void autonomous() {
    printf("=== SBOT AUTONOMOUS START ===\n");
    // HARDCODED FOR BACKUP SLOT: leftmid
    sbot_run_match_auto(SbotAutoSide::LEFT, SbotAutoAlliance::RED, false);
    return;
    // (selector code is skipped)
}
```

For Slot 1, no modification is made - the RoboDash selector works normally.

## Re-uploading After Code Changes

If you modify your autonomous code, just re-run:

```powershell
.\upload-backup-slots.ps1
```

This updates all 5 slots. **You don't need the `upload-default.ps1` script anymore** - everything is in one script now!

## Script Options

```powershell
# Standard usage (uploads all 5 slots)
.\upload-backup-slots.ps1

# Specify brain name
.\upload-backup-slots.ps1 -BrainName "SBOT"

# Skip building (use existing binary - faster, but be careful!)
.\upload-backup-slots.ps1 -SkipBuild
```

## Troubleshooting

**"Build failed"**: 
- Check that your code compiles normally with `pros make`
- Fix compile errors and try again
- **Your code is automatically restored**, so it's safe to fix and retry

**"Upload failed"**: 
- Make sure V5 brain is connected via USB
- Check brain is powered on
- Try specifying brain name with `-BrainName` parameter
- **Your code is still restored** even if upload fails

**Script won't run**:
```powershell
# Enable script execution (run once)
Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser
```

**Original file not restored**: 
- This should NEVER happen (it's in a `finally` block)
- But if it does, backup is at: `src\autonomous_sbot.cpp.backup`
- Restore manually: `copy src\autonomous_sbot.cpp.backup src\autonomous_sbot.cpp`

**One slot failed but others succeeded**:
- This is OK! The script continues even if one slot fails
- The successfully uploaded slots are still usable
- Fix the error and re-run to upload all 5 again

## What Gets Modified

The script **temporarily** modifies `src/main.cpp` during each build, then **ALWAYS restores** the original - even if errors occur.

Process for each slot:
1. Backs up original file → `.backup`
2. Modifies `autonomous()` function (or skips for Slot 1)
3. Builds project
4. Uploads to specific slot
5. **Restores original** (in loop - happens after each slot)
6. Final cleanup (in `finally` block - guaranteed to run)

**Double safety**: Restore happens both in the loop AND in the finally block, so your source is protected even if the script crashes.

## Competition Checklist

- [ ] Upload all 5 slots before competition day (run `.\upload-backup-slots.ps1`)
- [ ] Test Slot 1 (selector) on practice field
- [ ] Test each backup slot (2-5) on practice field
- [ ] Verify slot names show correctly on V5 brain
- [ ] Practice switching slots quickly between matches
- [ ] Know which slot to use for each starting position
- [ ] Verify autonomous runs correctly in each slot
- [ ] Battery fully charged
- [ ] Controller paired and charged

---

**Questions?** Check the main README.md or ask your programmer!
