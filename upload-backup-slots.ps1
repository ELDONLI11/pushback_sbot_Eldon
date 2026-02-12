#!/usr/bin/env pwsh
<#
.SYNOPSIS
    Build and upload ALL autonomous slots for sbot robot
    
.DESCRIPTION
    This script creates 5 program slots for tournament use:
    - Slot 1: "sbot-selector" - Normal version with controller selector (USE THIS FIRST!)
    - Slot 2: "leftmid"  - Hardcoded Red/Blue Left (Middle Goal)
    - Slot 3: "rightlow" - Hardcoded Red/Blue Right (Low Goal)
    - Slot 4: "leftsolo" - Hardcoded Solo AWP Left
    - Slot 5: "skills"   - Hardcoded Skills autonomous
    
    If the selector fails in competition, slots 2-5 run without any button input.
#>

param(
    [switch]$SkipBuild,
    [string]$BrainName = ""
)

$ErrorActionPreference = "Stop"

# Color output helpers
function Write-Step {
    param([string]$Message)
    Write-Host "`n=== $Message ===" -ForegroundColor Cyan
}

function Write-Success {
    param([string]$Message)
    Write-Host "✓ $Message" -ForegroundColor Green
}

function Write-Error {
    param([string]$Message)
    Write-Host "✗ $Message" -ForegroundColor Red
}

# Backup configurations: each entry is Slot#, Name, Mode to hardcode (empty Code = use default)
$backupSlots = @(
    @{ Slot = 1; Name = 'sbot-selector'; Code = '' },  # Default with RoboDash selector
    @{ Slot = 2; Name = 'leftmid';  Code = 'sbot_run_match_auto(SbotAutoSide::LEFT, SbotAutoAlliance::RED, false);' },
    @{ Slot = 3; Name = 'rightlow'; Code = 'sbot_run_match_auto(SbotAutoSide::RIGHT, SbotAutoAlliance::RED, false);' },
    @{ Slot = 4; Name = 'leftsolo'; Code = 'sbot_run_match_auto(SbotAutoSide::LEFT, SbotAutoAlliance::RED, true);' },
    @{ Slot = 5; Name = 'skills';   Code = 'sbot_run_skills_auto();' }
)

$sourceFile = "src/main.cpp"
$backupFile = "src/main.cpp.backup"

# Verify we're in the right directory
if (-not (Test-Path $sourceFile)) {
    Write-Error "Cannot find $sourceFile - are you in the project root?"
    exit 1
}

# Create backup of original file
Write-Step "Creating backup of $sourceFile"
Copy-Item $sourceFile $backupFile -Force
Write-Success "Backup created: $backupFile"

try {
    # Read original file content
    $originalContent = Get-Content $sourceFile -Raw

    foreach ($config in $backupSlots) {
        $slotNum = $config.Slot
        $slotName = $config.Name
        $hardcodedCode = $config.Code

        Write-Step "Building slot $slotNum`: $slotName"

        # If Code is empty, use default (no modification)
        if ($hardcodedCode) {
            # Modify the autonomous() function to hardcode the autonomous mode
            # Insert hardcoded call at the beginning of the function
            $searchPattern = 'void autonomous() {' + "`n" + '    printf("MARKER01\n");' + "`n" + '    printf("=== SBOT AUTONOMOUS() ENTER ===\n");' + "`n" + '    printf("=== SBOT AUTONOMOUS START ===\n");' + "`n" + '    printf("SBOT: Running RoboDash selector\n");' + "`n" + '    printf("SBOT: selector.run_auton()\n");' + "`n" + '    fflush(stdout);' + "`n" + "`n" + '    selector.run_auton();'
            $replacement = @"
void autonomous() {
    printf("MARKER01\n");
    printf("=== SBOT AUTONOMOUS() ENTER ===\n");
    printf("=== SBOT AUTONOMOUS START ===\n");
    printf("SBOT: HARDCODED BACKUP SLOT: $slotName\n");
    fflush(stdout);

    // HARDCODED FOR BACKUP SLOT: $slotName
    $hardcodedCode
    return;

    printf("SBOT: Running RoboDash selector\n");
    printf("SBOT: selector.run_auton()\n");
    fflush(stdout);

    selector.run_auton();
"@
            
            $modifiedContent = $originalContent.Replace($searchPattern, $replacement)

            # Write modified content
            Set-Content $sourceFile $modifiedContent -NoNewline

            Write-Host "  Modified run() to hardcode: $slotName"
        } else {
            Write-Host "  Using default code (selector enabled)"
        }

        if (-not $SkipBuild) {
            # Clean and build
            Write-Host "  Building project..."
            $buildOutput = & pros make 2>&1
            if ($LASTEXITCODE -ne 0) {
                Write-Error "Build failed for $slotName"
                Write-Host $buildOutput
                throw "Build failed"
            }
            Write-Success "Build successful"
        }

        # Upload to specific slot
        Write-Host "  Uploading to slot $slotNum as '$slotName'..."
        
        $uploadArgs = @("upload", "--slot", $slotNum, "--name", $slotName)
        if ($BrainName) {
            $uploadArgs += @("--target", $BrainName)
        }
        
        $uploadOutput = & pros @uploadArgs 2>&1
        if ($LASTEXITCODE -ne 0) {
            Write-Error "Upload failed for $slotName"
            Write-Host $uploadOutput
            throw "Upload failed"
        }
        Write-Success "Uploaded to slot $slotNum`: $slotName"

        # Restore original content for next iteration (ALWAYS runs, even if upload failed)
        Write-Host "  Restoring original code..."
        Set-Content $sourceFile $originalContent -NoNewline
    }

    Write-Step "All slots uploaded successfully!"
    Write-Host ""
    Write-Host "Tournament slots ready:" -ForegroundColor Green
    Write-Host "  Slot 1: sbot-selector - USE THIS FIRST (selector active)" -ForegroundColor Cyan
    Write-Host "  Slot 2: leftmid       - Backup: Red/Blue Left (Middle Goal)" -ForegroundColor Yellow
    Write-Host "  Slot 3: rightlow      - Backup: Red/Blue Right (Low Goal)" -ForegroundColor Yellow
    Write-Host "  Slot 4: leftsolo      - Backup: Solo AWP (Red Left)" -ForegroundColor Yellow
    Write-Host "  Slot 5: skills        - Backup: Skills Autonomous" -ForegroundColor Yellow
    Write-Host ""
    Write-Host "Tournament strategy:" -ForegroundColor Cyan
    Write-Host "  1. Always try Slot 1 first (use D-pad + A to select)" 
    Write-Host "  2. If selector fails, switch to appropriate backup slot (2-5)"
    Write-Host "  3. Backup slots run immediately without any button input"
    Write-Host ""
    Write-Host "NOTE: Original code is ALWAYS restored after each upload," -ForegroundColor Magenta
    Write-Host "      even if upload fails. Your source files are never permanently changed." -ForegroundColor Magenta

} catch {
    Write-Error "Error during slot creation: $_"
    Write-Host "Restoring original file from backup..." -ForegroundColor Yellow
    Copy-Item $backupFile $sourceFile -Force
    Write-Success "Original file restored after error"
    exit 1
} finally {
    # ALWAYS restore original - runs even if build or upload fails
    if (Test-Path $backupFile) {
        Write-Host "`nEnsuring original code is restored..." -ForegroundColor Cyan
        Copy-Item $backupFile $sourceFile -Force
        Remove-Item $backupFile -Force
        Write-Success "Original file restored and backup cleaned up"
    }
}

Write-Host "`nDone! Your robot has 5 slots ready for competition." -ForegroundColor Green
Write-Host "Slot 1 = Normal (try first), Slots 2-5 = Emergency backups" -ForegroundColor Green
