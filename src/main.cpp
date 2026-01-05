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

    // Development-mode autonomous selection when not connected to competition control.
    // Mirrors the official project's workflow: allow a short selection window,
    // and (if confirmed) run the selected routine immediately.
    if (!pros::competition::is_connected()) {
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
        printf("SBOT: competition control detected; select auto during DISABLED\n");
        fflush(stdout);
    }

    printf("=== SBOT INITIALIZE COMPLETE ===\n");
    fflush(stdout);
}

void disabled() {
    printf("=== SBOT DISABLED() ENTER ===\n");
    printf("=== SBOT DISABLED - AUTON SELECTOR ACTIVE ===\n");
    fflush(stdout);
    // While disabled, allow autonomous selection via controller
    while (pros::competition::is_disabled()) {
        if (sbot_auton) sbot_auton->updateSelector();
        pros::delay(20);
    }
    printf("=== SBOT EXITING DISABLED ===\n");
    fflush(stdout);
}

void competition_initialize() {
    printf("=== SBOT COMPETITION_INITIALIZE() ENTER ===\n");
    printf("=== SBOT COMPETITION INITIALIZE ===\n");
    fflush(stdout);
    // Nothing special for now; selection handled in disabled/initialize
}

void autonomous() {
    printf("=== SBOT AUTONOMOUS() ENTER ===\n");
    printf("=== SBOT AUTONOMOUS START ===\n");
    fflush(stdout);
    if (!sbot_auton) return;
    sbot_auton->run();
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

        // Periodic heartbeat so you can confirm the program is alive in the terminal
        const uint32_t now = pros::millis();
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
        if (sbot_master->get_digital_new_press(SBOT_GOAL_FLAP_TOGGLE_BTN)) {
            goal_flap_latched_open = !goal_flap_latched_open;
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
            printf("SBOT: R1 toggle -> storage_mode_active=%d\n", storage_mode_active ? 1 : 0);
            fflush(stdout);
        }

        if (sbot_master->get_digital_new_press(SBOT_TOP_GOAL_BUTTON)) {
            const bool turning_off = top_score_active;
            top_score_active = !turning_off;
            storage_mode_active = false;
            mid_score_active = false;
            low_score_active = false;
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
