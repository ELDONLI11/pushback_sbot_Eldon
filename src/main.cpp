/**
 * main.cpp - Entry point for sbot robot.
 */

#include <cstdio>

#include "main.h"
#include "config_sbot.h"
#include "drivetrain.h"
#include "intake.h"
#include "indexer.h"
#include "pneumatics.h"
#include "color_sensor_system.h"
#include "autonomous_sbot.h"
#include "lemlib_config_sbot.h"

// Global subsystem pointers
pros::Controller* sbot_master = nullptr;
SbotDrivetrain* sbot_drive = nullptr;
SbotIntake* sbot_intake = nullptr;
SbotIndexer* sbot_indexer = nullptr;
BatchLoaderPiston* sbot_batch_loader = nullptr;
GoalFlapPiston* sbot_goal_flap = nullptr;
SbotColorSensorSystem* sbot_color_system = nullptr;
SbotAutonomousSystem* sbot_auton = nullptr;

// Simple helpers for timed scoring actions
static void run_middle_goal_score() {
    printf("SBOT: run_middle_goal_score()\n");
    if (!sbot_intake || !sbot_indexer) return;
    sbot_intake->setMode(IntakeMode::COLLECT_FORWARD);
    sbot_indexer->setMode(IndexerMode::FEED_BACKWARD_MIDDLE);
    uint32_t start = pros::millis();
    while (pros::millis() - start < SBOT_MID_GOAL_SCORE_TIME_MS) {
        sbot_intake->update();
        sbot_indexer->update();
        pros::delay(10);
    }
    sbot_indexer->setMode(IndexerMode::OFF);
}

static void run_low_goal_score() {
    printf("SBOT: run_low_goal_score()\n");
    if (!sbot_intake || !sbot_indexer) return;
    sbot_indexer->setMode(IndexerMode::OFF);
    sbot_intake->setMode(IntakeMode::REVERSE_LOW_GOAL);
    uint32_t start = pros::millis();
    while (pros::millis() - start < SBOT_LOW_GOAL_SCORE_TIME_MS) {
        sbot_intake->update();
        pros::delay(10);
    }
    sbot_intake->setMode(IntakeMode::OFF);
}

void initialize() {
    // Make terminal prints immediate (helps when diagnosing "no output")
    setvbuf(stdout, nullptr, _IONBF, 0);
    printf("=== SBOT INITIALIZE START ===\n");
    fflush(stdout);

    sbot_master = new pros::Controller(pros::E_CONTROLLER_MASTER);
    sbot_drive = new SbotDrivetrain();
    sbot_intake = new SbotIntake();
    sbot_indexer = new SbotIndexer();
    sbot_batch_loader = new BatchLoaderPiston();
    sbot_goal_flap = new GoalFlapPiston();
    sbot_color_system = new SbotColorSensorSystem();
    sbot_auton = new SbotAutonomousSystem();

    // Default states
    sbot_batch_loader->retract();
    sbot_goal_flap->close();
    sbot_color_system->setAllianceColor(AllianceColor::RED); // default
    sbot_color_system->setSortingEnabled(false);

    sbot_auton->initialize();

    printf("SBOT: subsystems created; defaults applied\n");
    fflush(stdout);

    // ========== AUTONOMOUS SELECTOR - ALWAYS AVAILABLE ON STARTUP ==========
    // Initialize and display selector immediately when program starts.
    // This ensures selection is ALWAYS available, regardless of competition state.
    if (sbot_master && sbot_master->is_connected()) {
        printf("\nSBOT: Initializing autonomous selector...\n");
        fflush(stdout);
        sbot_master->clear();
        pros::delay(50);
        
        // Force initial display
        if (sbot_auton) {
            sbot_auton->updateSelector();
            printf("SBOT: Selector ready - use D-pad L/R + A anytime to select/change\n");
            printf("SBOT: Current selection will be shown on controller screen\n");
            fflush(stdout);
        }
    } else {
        printf("\nSBOT: WARNING - Controller not connected, selector display unavailable\n");
        printf("SBOT: Selector logic still active, will display when controller connects\n");
        fflush(stdout);
    }
    // ========================================================================

    // ========== MANUAL COMPETITION TEST MODE (NO HARDWARE NEEDED) ==========
    // Uncomment this block to manually test selector and autonomous
    // using controller buttons (100% reliable, no competition switch needed)
    /*
    printf("\n=== MANUAL COMPETITION TEST MODE ENABLED ===\n");
    printf("Press X button on controller to start test sequence\n");
    fflush(stdout);
    
    // Wait for X button to start test
    while (!sbot_master->get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
        pros::delay(20);
    }
    
    printf("\n--- SIMULATING DISABLED PHASE (30 seconds) ---\n");
    printf("Use D-pad Left/Right + A to select autonomous\n");
    fflush(stdout);
    
    if (sbot_master) sbot_master->clear();
    pros::delay(50);
    
    // Run selector for 30 seconds (like real disabled period)
    uint32_t disabled_start = pros::millis();
    while (pros::millis() - disabled_start < 30000) {
        if (sbot_auton) sbot_auton->updateSelector();
        
        // Allow early exit with Y button
        if (sbot_master->get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            printf("Y pressed - ending disabled early\n");
            break;
        }
        pros::delay(20);
    }
    
    printf("\n--- SIMULATING AUTONOMOUS PHASE ---\n");
    const int mode = static_cast<int>(sbot_auton->getSelector().getMode());
    const bool confirmed = sbot_auton->getSelector().isConfirmed();
    printf("Selected mode: %d (confirmed: %s)\n", mode, confirmed ? "YES" : "NO");
    fflush(stdout);
    
    if (sbot_auton) {
        if (mode == 0 && !confirmed) {
            printf("WARNING: Using fallback - Red Right\n");
            sbot_auton->runRedRight();
        } else {
            sbot_auton->run();
        }
    }
    
    printf("\n--- TEST COMPLETE - Entering driver control ---\n");
    fflush(stdout);
    // Fall through to normal opcontrol
    */
    // =========================================================================

    // ========== SELECTOR TEST MODE (SIMULATED BUTTON PRESSES) ==========
    // Uncomment this block to test the selector with simulated button presses
    // without needing a physical controller connected.
    /*
    printf("\n=== SELECTOR TEST MODE: SIMULATED BUTTON PRESSES ===\n");
    fflush(stdout);
    
    if (sbot_auton) {
        printf("TEST: Simulating 3x RIGHT button presses...\n");
        fflush(stdout);
        pros::delay(500);
        sbot_auton->getSelector().simulateRightButton();  // Move to mode 1
        pros::delay(500);
        sbot_auton->getSelector().simulateRightButton();  // Move to mode 2
        pros::delay(500);
        sbot_auton->getSelector().simulateRightButton();  // Move to mode 3
        pros::delay(500);
        
        printf("TEST: Simulating 1x LEFT button press...\n");
        fflush(stdout);
        sbot_auton->getSelector().simulateLeftButton();   // Back to mode 2
        pros::delay(500);
        
        printf("TEST: Simulating A button (confirm)...\n");
        fflush(stdout);
        sbot_auton->getSelector().simulateConfirmButton(); // Confirm mode 2
        pros::delay(500);
        
        printf("TEST: Final selected mode = %d (confirmed: %s)\n",
               static_cast<int>(sbot_auton->getSelector().getMode()),
               sbot_auton->getSelector().isConfirmed() ? "YES" : "NO");
        fflush(stdout);
        
        printf("TEST: You should see mode 2 (Red Right) displayed on controller\n");
        printf("=== SELECTOR TEST MODE COMPLETE ===\n\n");
        fflush(stdout);
        pros::delay(2000);
    }
    */
    // ====================================================================

    // ========== AUTONOMOUS SELECTOR - ALWAYS AVAILABLE ON STARTUP ==========
    // Initialize and display selector immediately when program starts.
    // This ensures selection is ALWAYS available, regardless of competition state.
    printf("\nSBOT: Initializing autonomous selector on startup...\n");
    fflush(stdout);
    
    if (sbot_master && sbot_master->is_connected()) {
        sbot_master->clear();
        pros::delay(50);
        
        // Force initial display
        if (sbot_auton) {
            sbot_auton->updateSelector();
            printf("SBOT: Selector ready - use D-pad L/R + A anytime to select/change\n");
            printf("SBOT: Current selection will be shown on controller screen\n");
            fflush(stdout);
        }
    } else {
        printf("SBOT: WARNING - Controller not connected, selector display unavailable\n");
        printf("SBOT: Selector logic still active, will display when controller connects\n");
        fflush(stdout);
    }
    // ========================================================================

    // Autonomous selection window based on competition mode
    if (!pros::competition::is_connected()) {
        // Development-mode autonomous selection when not connected to competition control.
        // Mirrors the official project's workflow: allow a short selection window,
        // and (if confirmed) run the selected routine immediately.
        printf("SBOT: development mode (no competition control). Auto select open for 10s.\n");
        fflush(stdout);

        int countdown = 500; // 500 * 20ms = 10 seconds
        bool mode_confirmed = false;
        while (countdown > 0) {
            if (sbot_auton && sbot_auton->getSelector().update()) {
                // Mode confirmed, stop immediately
                printf("SBOT: auto mode confirmed\n");
                fflush(stdout);
                mode_confirmed = true;
                break;
            }

            // Occasional heartbeat while waiting
            if (countdown % 50 == 0) {
                printf("SBOT: auto select... %ds left\n", countdown / 50);
                fflush(stdout);
            }

            countdown--;
            pros::delay(20);
        }

        // If a mode was confirmed in dev mode, run it immediately.
        // This matches the "select then immediately test" workflow used in the old project.
        if (mode_confirmed || (sbot_auton && sbot_auton->getSelector().isConfirmed())) {
            const int selected = static_cast<int>(sbot_auton->getSelector().getMode());
            printf("SBOT: DEV MODE running selected autonomous mode: %d\n", selected);
            fflush(stdout);
            pros::delay(250);

            // Run the selected routine (match routes are stubs; test routes may move motors)
            sbot_auton->run();

            printf("SBOT: DEV MODE autonomous complete; entering opcontrol\n");
            fflush(stdout);
            pros::delay(250);
        }
    } else {
        // Competition mode - provide 30 second selection window
        printf("SBOT: competition control detected. Auto select open for 30s.\n");
        fflush(stdout);

        int countdown = 1500; // 1500 * 20ms = 30 seconds
        while (countdown > 0) {
            if (sbot_auton) {
                sbot_auton->updateSelector();
            }

            // Periodic heartbeat for tracking
            if (countdown % 50 == 0) {
                printf("SBOT: auto select... %ds left\n", countdown / 50);
                fflush(stdout);
            }

            countdown--;
            pros::delay(20);
        }

        // Display final selection
        if (sbot_auton) {
            const int mode = static_cast<int>(sbot_auton->getSelector().getMode());
            const bool confirmed = sbot_auton->getSelector().isConfirmed();
            printf("SBOT: Selection window complete. Mode: %d (%s)\n",
                   mode, confirmed ? "CONFIRMED" : "not confirmed");
            fflush(stdout);
        }
    }

    printf("=== SBOT INITIALIZE COMPLETE ===\n");
    fflush(stdout);
}

void disabled() {
    printf("=== SBOT DISABLED() ENTER ===\n");
    printf("=== SBOT DISABLED - AUTON SELECTOR ACTIVE ===\n");
    printf("SBOT: Selector was already initialized in initialize()\n");
    printf("SBOT: Use D-pad L/R + A to change selection if needed\n");
    fflush(stdout);
    
    // Display current selection status
    if (sbot_auton) {
        const int mode = static_cast<int>(sbot_auton->getSelector().getMode());
        const bool confirmed = sbot_auton->getSelector().isConfirmed();
        printf("SBOT: Current selection: mode %d (%s)\n",
               mode, confirmed ? "CONFIRMED" : "not confirmed");
        fflush(stdout);
    }
    
    // Continue updating selector while disabled (allows changes)
    int update_count = 0;
    int refresh_counter = 0;
    const int REFRESH_INTERVAL = 50; // Force screen refresh every 1 second
    
    while (pros::competition::is_disabled()) {
        if (sbot_auton) {
            sbot_auton->updateSelector();
            
            // Periodically force a display refresh
            refresh_counter++;
            if (refresh_counter >= REFRESH_INTERVAL) {
                sbot_auton->getSelector().forceDisplayRefresh();
                printf("SELECTOR: Forced screen refresh\n");
                fflush(stdout);
                refresh_counter = 0;
            }
        }
        
        update_count++;
        if (update_count % 25 == 0) {
            printf("SBOT: Selector active (updates: %d, time: ~%d sec)\n", 
                   update_count, update_count / 50);
            fflush(stdout);
        }
        pros::delay(20);
    }
    
    // Show final selection
    if (sbot_auton) {
        const int mode = static_cast<int>(sbot_auton->getSelector().getMode());
        const bool confirmed = sbot_auton->getSelector().isConfirmed();
        printf("SBOT: Final selection: mode %d (%s)\n",
               mode, confirmed ? "CONFIRMED" : "not confirmed");
        fflush(stdout);
    }
    
    printf("=== SBOT EXITING DISABLED (ran %d updates over ~%d sec) ===\n", 
           update_count, update_count / 50);
    fflush(stdout);
}

void competition_initialize() {
    printf("=== SBOT COMPETITION_INITIALIZE() ENTER ===\n");
    printf("=== SBOT COMPETITION INITIALIZE ===\n");
    printf("SBOT: Selector already initialized in initialize()\n");
    fflush(stdout);
    
    // Just refresh the display (don't clear - preserves any selection made during initialize)
    if (sbot_auton) {
        printf("SBOT: Refreshing selector display for competition mode\n");
        fflush(stdout);
        sbot_auton->updateSelector();
    }
}

void autonomous() {
    printf("=== SBOT AUTONOMOUS() ENTER ===\n");
    printf("=== SBOT AUTONOMOUS START ===\n");
    fflush(stdout);
    
    if (sbot_auton) {
        const int mode_num = static_cast<int>(sbot_auton->getSelector().getMode());
        const bool confirmed = sbot_auton->getSelector().isConfirmed();
        printf("SBOT: Running autonomous mode %d (confirmed: %s)\n", 
               mode_num, confirmed ? "YES" : "NO");
        fflush(stdout);
        
        // ONLY use fallback if mode is 0 (DISABLED) AND not confirmed
        // If they selected mode 0 intentionally, respect that choice
        if (mode_num == 0 && !confirmed) {
            printf("WARNING: No valid mode selected! Using EMERGENCY FALLBACK: RIGHT\n");
            printf("WARNING: Next time, select autonomous during DISABLED period!\n");
            fflush(stdout);
            // Change this to your preferred safe autonomous:
            sbot_auton->runRight();
        } else {
            // Run selected mode (even if mode 0 was intentionally selected)
            sbot_auton->run();
        }
    } else {
        printf("ERROR: sbot_auton not initialized\n");
    }
    
    printf("=== SBOT AUTONOMOUS COMPLETE ===\n");
    fflush(stdout);
}

void opcontrol() {
    printf("=== SBOT OPCONTROL() ENTER ===\n");
    if (!sbot_master || !sbot_drive || !sbot_intake || !sbot_indexer || !sbot_goal_flap || !sbot_batch_loader || !sbot_color_system) {
        printf("SBOT OPCONTROL: missing subsystem(s); returning early\n");
        fflush(stdout);
        return;
    }

    printf("=== SBOT DRIVER CONTROL START ===\n");
    fflush(stdout);

    uint32_t last_heartbeat_ms = pros::millis();
    uint32_t last_selector_update_ms = pros::millis();

    // Latched pneumatic states for toggle buttons
    bool goal_flap_latched_open = false;
    bool batch_loader_latched_extended = false;

    // Latched ball-handling modes (mutually exclusive)
    bool storage_mode_active = false;
    bool top_score_active = false;
    bool mid_score_active = false;
    bool low_score_active = false;

    while (true) {
        // Allow autonomous/test mode change during driver control, matching the old project.
        // Holding R1+R2 enters selection mode; release to return to driving.
        if (sbot_master->get_digital(pros::E_CONTROLLER_DIGITAL_R1) &&
            sbot_master->get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            printf("SBOT: entering AUTO CHANGE mode (hold R1+R2). Use D-pad L/R + A.\n");
            fflush(stdout);

            // Safe state while selecting
            sbot_intake->setMode(IntakeMode::OFF);
            sbot_indexer->setMode(IndexerMode::OFF);
            sbot_goal_flap->close();
            sbot_intake->update();
            sbot_indexer->update();

            while (sbot_master->get_digital(pros::E_CONTROLLER_DIGITAL_R1) ||
                   sbot_master->get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
                if (sbot_auton) sbot_auton->updateSelector();
                pros::delay(20);
            }

            printf("SBOT: exiting AUTO CHANGE mode\n");
            fflush(stdout);

            // Skip the rest of this loop iteration so we don't immediately act on R1/R2 state.
            pros::delay(20);
            continue;
        }

        // Periodic heartbeat and selector update
        uint32_t now = pros::millis();
        
        // Periodic selector update (every 2 seconds) to detect late controller connection
        if (now - last_selector_update_ms > 2000) {
            if (sbot_auton) sbot_auton->updateSelector();
            last_selector_update_ms = now;
        }

        // Heartbeat so you can confirm the program is alive in the terminal
        if (now - last_heartbeat_ms >= 2000) {
            last_heartbeat_ms = now;
            printf("SBOT: opcontrol alive (%lu ms)\n", static_cast<unsigned long>(now));
        }

        // Drivetrain tank control
        sbot_drive->arcadeTankControl(*sbot_master);

        // Alliance color and sorting toggles
        if (sbot_master->get_digital_new_press(SBOT_SET_RED_ALLIANCE_BTN)) {
            sbot_color_system->setAllianceColor(AllianceColor::RED);
        }
        if (sbot_master->get_digital_new_press(SBOT_SET_BLUE_ALLIANCE_BTN)) {
            sbot_color_system->setAllianceColor(AllianceColor::BLUE);
        }
        if (sbot_master->get_digital_new_press(SBOT_COLOR_SORT_TOGGLE_BTN)) {
            sbot_color_system->setSortingEnabled(!sbot_color_system->isSortingEnabled());
        }

        // Pneumatic toggles (latched)
        // Goal flap / descorer toggle (same physical mechanism)
        if (sbot_master->get_digital_new_press(SBOT_GOAL_FLAP_TOGGLE_BTN)) {
            goal_flap_latched_open = !goal_flap_latched_open;
            printf("SBOT: A pressed -> goal_flap_latched_open=%d\n", goal_flap_latched_open ? 1 : 0);
            fflush(stdout);
        }

        if (sbot_master->get_digital_new_press(SBOT_BATCH_LOADER_TOGGLE_BTN)) {
            batch_loader_latched_extended = !batch_loader_latched_extended;
            printf("SBOT: B pressed -> batch loader latched = %d\n", batch_loader_latched_extended ? 1 : 0);
            fflush(stdout);
            if (batch_loader_latched_extended) {
                sbot_batch_loader->extend();
            } else {
                sbot_batch_loader->retract();
            }
        }

        // Ball handling mode toggles (run until stopped)
        // R1 = storage/intake mode (same motors as top-score but flap forced DOWN)
        // R2 = top-score mode (same motors but flap UP)
        // L1 = middle score (continuous indexer reverse)
        // L2 = low score (continuous intake reverse)
        if (sbot_master->get_digital_new_press(SBOT_COLLECT_BUTTON)) {
            const bool turning_off = storage_mode_active;
            storage_mode_active = !turning_off;
            top_score_active = false;
            mid_score_active = false;
            low_score_active = false;
            // When entering storage mode, force flap closed and sync latched state
            if (storage_mode_active) {
                goal_flap_latched_open = false;
                sbot_goal_flap->close();
                printf("SBOT: R1 ON -> forcing flap closed, latched=false\n");
            }
            printf("SBOT: R1 toggle -> storage_mode_active=%d\n", storage_mode_active ? 1 : 0);
            fflush(stdout);
        }

        if (sbot_master->get_digital_new_press(SBOT_TOP_GOAL_BUTTON)) {
            const bool turning_off = top_score_active;
            top_score_active = !turning_off;
            storage_mode_active = false;
            mid_score_active = false;
            low_score_active = false;
            // When entering top score mode, sync latched state with open position
            if (top_score_active) {
                goal_flap_latched_open = true;
                printf("SBOT: R2 ON -> syncing latched=true (flap will be opened)\n");
            }
            printf("SBOT: R2 toggle -> top_score_active=%d\n", top_score_active ? 1 : 0);
            fflush(stdout);
        }

        if (sbot_master->get_digital_new_press(SBOT_MID_GOAL_BUTTON)) {
            const bool turning_off = mid_score_active;
            mid_score_active = !turning_off;
            storage_mode_active = false;
            top_score_active = false;
            low_score_active = false;
            printf("SBOT: L1 toggle -> mid_score_active=%d\n", mid_score_active ? 1 : 0);
            fflush(stdout);
        }

        if (sbot_master->get_digital_new_press(SBOT_LOW_GOAL_BUTTON)) {
            const bool turning_off = low_score_active;
            low_score_active = !turning_off;
            storage_mode_active = false;
            top_score_active = false;
            mid_score_active = false;
            printf("SBOT: L2 toggle -> low_score_active=%d\n", low_score_active ? 1 : 0);
            fflush(stdout);
        }

        // Apply motor modes based on selected latched action
        if (top_score_active) {
            sbot_intake->setMode(IntakeMode::COLLECT_FORWARD);
            sbot_indexer->setMode(IndexerMode::FEED_FORWARD);
        } else if (storage_mode_active) {
            sbot_intake->setMode(IntakeMode::COLLECT_FORWARD);
            sbot_indexer->setMode(IndexerMode::FEED_FORWARD);
        } else if (mid_score_active) {
            sbot_intake->setMode(IntakeMode::COLLECT_FORWARD);
            sbot_indexer->setMode(IndexerMode::FEED_BACKWARD_MIDDLE);
        } else if (low_score_active) {
            sbot_indexer->setMode(IndexerMode::OFF);
            sbot_intake->setMode(IntakeMode::REVERSE_LOW_GOAL);
        } else {
            sbot_intake->setMode(IntakeMode::OFF);
            sbot_indexer->setMode(IndexerMode::OFF);
        }

        // Apply goal flap state:
        // - Top-score mode forces flap OPEN
        // - Storage mode forces flap CLOSED (pistons down)
        // - Otherwise, manual A-toggle controls it
        if (top_score_active) {
            sbot_goal_flap->open();
        } else if (storage_mode_active) {
            sbot_goal_flap->close();
        } else if (goal_flap_latched_open) {
            sbot_goal_flap->open();
        } else {
            sbot_goal_flap->close();
        }

        // Apply color sorting overrides
        sbot_color_system->update(*sbot_indexer);

        // Apply motor commands
        sbot_intake->update();
        sbot_indexer->update();

        pros::delay(20);
    }
}
