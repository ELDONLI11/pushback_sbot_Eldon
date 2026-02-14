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
#include "autonomous_match_awp.h"
#include "robodash_selector.h"
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
    
    // First, briefly reverse intake to push balls back (0.25s)
    printf("SBOT: Mid score - reversing intake for 250ms\n");
    sbot_intake->setMode(IntakeMode::REVERSE_LOW_GOAL);
    sbot_indexer->setMode(IndexerMode::OFF);
    uint32_t reverse_start = pros::millis();
    while (pros::millis() - reverse_start < 250) {
        sbot_intake->update();
        pros::delay(10);
    }
    
    // Then start normal mid scoring
    printf("SBOT: Mid score - starting normal scoring\n");
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
    printf("MARKERA0\n");
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

    printf("MARKERA1\n");
    fflush(stdout);

    // Default states
    sbot_batch_loader->retract();
    sbot_goal_flap->close();
    sbot_color_system->setAllianceColor(AllianceColor::RED); // default
    sbot_color_system->setSortingEnabled(false);

    // CRITICAL: Initialize LemLib before autonomous system
    printf("MARKERA2\n");
    printf("SBOT: Initializing LemLib...\n");
    fflush(stdout);
    initializeSbotLemLib();
    printf("MARKERA3\n");
    printf("SBOT: LemLib initialized\n");
    fflush(stdout);

    sbot_auton->initialize();

    printf("MARKERA4\n");
    printf("SBOT: subsystems created; defaults applied\n");
    fflush(stdout);

    printf("=== SBOT INITIALIZE COMPLETE ===\n");
    fflush(stdout);
}

void disabled() {
    printf("MARKERB1\n");
    printf("=== SBOT DISABLED() ENTER ===\n");
    fflush(stdout);

    // Show RoboDash selector on brain screen so the driver can pick an autonomous
    // while the robot is disabled (competition switch / field controller connected).
    selector.focus();
    printf("SBOT: RoboDash selector focused on brain screen\n");
    fflush(stdout);
}

void competition_initialize() {
    printf("=== SBOT COMPETITION_INITIALIZE() ENTER ===\n");
    fflush(stdout);
}

void autonomous() {
    printf("MARKER01\n");
    printf("=== SBOT AUTONOMOUS() ENTER ===\n");
    printf("=== SBOT AUTONOMOUS START ===\n");
    printf("SBOT: Running RoboDash selector\n");
    printf("SBOT: selector.run_auton()\n");
    fflush(stdout);

    selector.run_auton();

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

    // ========================================================================
    // DEV MODE: Run autonomous without competition hardware
    // ========================================================================
    // When no field controller / competition switch is connected, PROS skips
    // disabled() and autonomous() and jumps straight to opcontrol().  In that
    // case we let the driver:
    //   1. Pick an autonomous on the brain's RoboDash touchscreen
    //   2. Press Y on the controller to run it (or DOWN to skip to driver)
    // ========================================================================
    if (!pros::competition::is_connected()) {
        printf("SBOT: development mode (no competition control)\n");
        fflush(stdout);

        // Focus the RoboDash selector on the brain screen
        selector.focus();

        if (sbot_master->is_connected()) {
            sbot_master->clear();
            pros::delay(50);
            sbot_master->print(0, 0, "DEV: select on brain");
            pros::delay(50);
            sbot_master->print(1, 0, "Y=run  DOWN=skip");
        }
        printf("SBOT: Select autonomous on brain touchscreen\n");
        printf("SBOT: Press Y to run, DOWN to skip to driver control\n");
        fflush(stdout);

        // Wait for Y (run auton) or DOWN (skip to driver control)
        bool run_auton = false;
        while (true) {
            if (sbot_master->get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
                run_auton = true;
                break;
            }
            if (sbot_master->get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
                break;
            }
            // Break if competition control connects mid-wait
            if (pros::competition::is_connected()) break;
            pros::delay(20);
        }

        if (run_auton) {
            printf("SBOT: DEV MODE - running selected autonomous\n");
            fflush(stdout);
            if (sbot_master->is_connected()) {
                sbot_master->clear();
                pros::delay(50);
                sbot_master->print(0, 0, "Running autonomous...");
            }
            selector.run_auton();
            printf("SBOT: DEV MODE - autonomous complete\n");
            fflush(stdout);
        } else {
            printf("SBOT: DEV MODE - skipping autonomous\n");
            fflush(stdout);
        }

        if (sbot_master->is_connected()) {
            sbot_master->clear();
            pros::delay(50);
            sbot_master->print(0, 0, "Driver control");
        }
    }
    // ========================================================================

    printf("=== SBOT DRIVER CONTROL START ===\n");
    fflush(stdout);

    uint32_t last_heartbeat_ms = pros::millis();

    // Latched pneumatic states for toggle buttons
    bool goal_flap_latched_open = false;
    bool batch_loader_latched_extended = false;

    // Latched ball-handling modes (mutually exclusive)
    bool storage_mode_active = false;
    bool top_score_active = false;
    bool mid_score_active = false;
    bool low_score_active = false;
    bool reverse_intake_active = false;  // Manual reverse intake mode

    while (true) {
        // Periodic heartbeat and selector update
        uint32_t now = pros::millis();

        // Heartbeat so you can confirm the program is alive in the terminal
        if (now - last_heartbeat_ms >= 2000) {
            last_heartbeat_ms = now;
            printf("SBOT: opcontrol alive (%lu ms)\n", static_cast<unsigned long>(now));
        }

        // Drivetrain tank drive control (left stick = left side, right stick = right side)
        sbot_drive->tankControl(*sbot_master);

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
            
            // If descorer is now extended (flap open), turn off storage mode (requires flap closed)
            if (goal_flap_latched_open && storage_mode_active) {
                storage_mode_active = false;
                printf("SBOT: Descorer extended -> turning OFF storage mode\n");
            }
            
            // If descorer is now retracted (flap closed), turn off top score mode (requires flap open)
            if (!goal_flap_latched_open && top_score_active) {
                top_score_active = false;
                printf("SBOT: Descorer retracted -> turning OFF top score mode\n");
            }
            
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
            reverse_intake_active = false;
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
            reverse_intake_active = false;
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
            reverse_intake_active = false;
            printf("SBOT: L1 toggle -> mid_score_active=%d\n", mid_score_active ? 1 : 0);
            fflush(stdout);
        }

        if (sbot_master->get_digital_new_press(SBOT_LOW_GOAL_BUTTON)) {
            const bool turning_off = low_score_active;
            low_score_active = !turning_off;
            storage_mode_active = false;
            top_score_active = false;
            mid_score_active = false;
            reverse_intake_active = false;
            printf("SBOT: L2 toggle -> low_score_active=%d\n", low_score_active ? 1 : 0);
            fflush(stdout);
        }

        // X button = manual reverse intake (for clearing jams or ejecting balls)
        if (sbot_master->get_digital_new_press(SBOT_REVERSE_INTAKE_BTN)) {
            const bool turning_off = reverse_intake_active;
            reverse_intake_active = !turning_off;
            storage_mode_active = false;
            top_score_active = false;
            mid_score_active = false;
            low_score_active = false;
            printf("SBOT: X toggle -> reverse_intake_active=%d\n", reverse_intake_active ? 1 : 0);
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
        } else if (reverse_intake_active) {
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
