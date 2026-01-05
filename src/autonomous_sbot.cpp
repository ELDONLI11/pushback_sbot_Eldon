/**
 * autonomous_sbot.cpp - Autonomous selector and stub routines for sbot.
 */

#include "autonomous_sbot.h"
#include "lemlib_config_sbot.h"

#include "intake.h"
#include "indexer.h"
#include "pneumatics.h"

#include <cmath>

// These are owned/created in src/main.cpp.
// We reference them here so autonomous can run mechanism actions.
extern SbotIntake* sbot_intake;
extern SbotIndexer* sbot_indexer;
extern BatchLoaderPiston* sbot_batch_loader;
extern GoalFlapPiston* sbot_goal_flap;

static void sbot_safe_stop_mechanisms() {
    if (sbot_intake) sbot_intake->setMode(IntakeMode::OFF);
    if (sbot_indexer) sbot_indexer->setMode(IndexerMode::OFF);
    if (sbot_goal_flap) sbot_goal_flap->close();
    if (sbot_batch_loader) sbot_batch_loader->retract();

    // Apply immediately
    if (sbot_intake) sbot_intake->update();
    if (sbot_indexer) sbot_indexer->update();
}

static void sbot_run_for_ms(uint32_t duration_ms) {
    const uint32_t start = pros::millis();
    while (pros::millis() - start < duration_ms) {
        if (sbot_intake) sbot_intake->update();
        if (sbot_indexer) sbot_indexer->update();
        pros::delay(10);
    }
}

static void sbot_print_pose(const char* label) {
    if (!sbot_chassis) return;
    const auto pose = sbot_chassis->getPose();
    printf("SBOT POSE [%s]: x=%.2f y=%.2f th=%.2f\n", label, pose.x, pose.y, pose.theta);
}

static void sbot_print_sensors(const char* label) {
    const double imu_heading = sbot_inertial_sensor ? sbot_inertial_sensor->get_heading() : 0.0;
    const double imu_rotation = sbot_inertial_sensor ? sbot_inertial_sensor->get_rotation() : 0.0;
    const double vert_pos = sbot_vertical_encoder ? sbot_vertical_encoder->get_position() : 0.0;
    const double vert_in = sbot_vertical_tracking_wheel ? sbot_vertical_tracking_wheel->getDistanceTraveled() : 0.0;
    printf(
        "SBOT SENSORS [%s]: imu.heading=%.2f imu.rotation=%.2f vertRot.pos=%.2f vert.in=%.2f\n",
        label,
        imu_heading,
        imu_rotation,
        vert_pos,
        vert_in
    );
}

static void sbot_zero_pose_and_sensors(float x = 0, float y = 0, float theta_deg = 0) {
    if (!sbot_chassis) return;
    sbot_chassis->setPose(x, y, theta_deg);

    // IMPORTANT: LemLib IMU integration often relies on IMU *rotation* (continuous), not just heading.
    // Your logs showed heading got reset but rotation did not, which causes odom to think it is already turned.
    if (sbot_inertial_sensor) {
        sbot_inertial_sensor->tare_rotation();
        sbot_inertial_sensor->tare_heading();
        // Be explicit about the requested frame.
        sbot_inertial_sensor->set_rotation(theta_deg);
        sbot_inertial_sensor->set_heading(theta_deg);
    }
    if (sbot_vertical_encoder) {
        sbot_vertical_encoder->reset_position();
    }
    pros::delay(60);
}

static double sbot_norm_heading(double deg) {
    while (deg < 0) deg += 360.0;
    while (deg >= 360.0) deg -= 360.0;
    return deg;
}

static double sbot_mirror_heading(double heading_deg) {
    // Mirror across the field centerline (LEFT <-> RIGHT).
    // With LemLib convention (+Y forward, +X right, 0° facing +Y), this is: heading -> 360 - heading.
    return sbot_norm_heading(360.0 - heading_deg);
}

struct SbotPoint {
    double x;
    double y;
};

static SbotPoint sbot_mirror_point_y(const SbotPoint& p) {
    return {p.x, -p.y};
}

static SbotPoint sbot_mirror_point_x(const SbotPoint& p) {
    return {-p.x, p.y};
}

static bool sbot_drive_to(const SbotPoint& p, uint32_t timeout_ms, bool mirrored_y = false, bool forwards = true) {
    if (!validateSbotLemLibInitialization()) return false;
    if (!sbot_chassis) return false;

    const SbotPoint target = mirrored_y ? sbot_mirror_point_y(p) : p;

    lemlib::MoveToPointParams params;
    params.forwards = forwards;
    sbot_chassis->moveToPoint(target.x, target.y, timeout_ms, params);
    sbot_chassis->waitUntilDone();
    return true;
}

static bool sbot_turn_to(double heading_deg, uint32_t timeout_ms, bool mirrored_y = false) {
    if (!validateSbotLemLibInitialization()) return false;
    if (!sbot_chassis) return false;

    const double target_heading = mirrored_y ? sbot_mirror_heading(heading_deg) : sbot_norm_heading(heading_deg);
    sbot_chassis->turnToHeading(target_heading, timeout_ms);
    sbot_chassis->waitUntilDone();
    return true;
}

static bool sbot_drive_relative(double distance_in, uint32_t timeout_ms, bool forwards = true) {
    if (!validateSbotLemLibInitialization()) return false;
    if (!sbot_chassis) return false;

    const auto pose = sbot_chassis->getPose();
    const double heading_rad = pose.theta * M_PI / 180.0;
    // LemLib odom convention: at theta=0°, robot faces +Y. (+X is right)
    const double dx = distance_in * std::sin(heading_rad);
    const double dy = distance_in * std::cos(heading_rad);

    lemlib::MoveToPointParams params;
    params.forwards = forwards;
    sbot_chassis->moveToPoint(pose.x + dx, pose.y + dy, timeout_ms, params);
    sbot_chassis->waitUntilDone();
    return true;
}

static void sbot_intake_on_storage() {
    if (sbot_intake) sbot_intake->setMode(IntakeMode::COLLECT_FORWARD);
    if (sbot_indexer) sbot_indexer->setMode(IndexerMode::FEED_FORWARD);
    if (sbot_goal_flap) sbot_goal_flap->close();
    if (sbot_intake) sbot_intake->update();
    if (sbot_indexer) sbot_indexer->update();
}

static void sbot_score_mid_for(uint32_t ms) {
    if (!sbot_indexer) return;

    // Mimic driver helper behavior: intake forward assists while indexer reverses.
    if (sbot_intake) sbot_intake->setMode(IntakeMode::COLLECT_FORWARD);
    sbot_indexer->setMode(IndexerMode::FEED_BACKWARD_MIDDLE);
    sbot_run_for_ms(ms);
    sbot_indexer->setMode(IndexerMode::OFF);
    if (sbot_intake) sbot_intake->setMode(IntakeMode::OFF);
    sbot_run_for_ms(120);
}

static void sbot_score_top_for(uint32_t ms) {
    if (!sbot_indexer) return;
    if (sbot_goal_flap) sbot_goal_flap->open();
    if (sbot_intake) sbot_intake->setMode(IntakeMode::COLLECT_FORWARD);
    sbot_indexer->setMode(IndexerMode::FEED_FORWARD);
    sbot_run_for_ms(ms);
    sbot_safe_stop_mechanisms();
}

static void sbot_set_match_start_pose() {
    // Start pose definition (Phase 4+):
    // - Robot BACK bumper touching the black "park zone" strip at the far end of the alliance goal.
    // - Heading 0° points "away from the goal" into the field (+Y in LemLib convention).
    // This makes all coordinates in these routines local to the start placement.
    if (!sbot_chassis) return;
    sbot_zero_pose_and_sensors(0, 0, 0);
}

enum class SbotAutoSide {
    RIGHT = 0,
    LEFT
};

enum class SbotAutoAlliance {
    RED = 0,
    BLUE
};

static SbotPoint sbot_rotate180_point(const SbotPoint& p) {
    return {-p.x, -p.y};
}

static double sbot_rotate180_heading(double heading_deg) {
    return sbot_norm_heading(heading_deg + 180.0);
}

static SbotPoint sbot_apply_auto_transform(const SbotPoint& p, SbotAutoSide side, SbotAutoAlliance alliance) {
    SbotPoint out = p;
    if (alliance == SbotAutoAlliance::BLUE) {
        out = sbot_rotate180_point(out);
    }
    if (side == SbotAutoSide::LEFT) {
        // Mirror RIGHT <-> LEFT. With LemLib, X is right, so negate X.
        out = sbot_mirror_point_x(out);
    }
    return out;
}

static double sbot_apply_auto_transform_heading(double heading_deg, SbotAutoSide side, SbotAutoAlliance alliance) {
    double out = heading_deg;
    if (alliance == SbotAutoAlliance::BLUE) {
        out = sbot_rotate180_heading(out);
    }
    if (side == SbotAutoSide::LEFT) {
        out = sbot_mirror_heading(out);
    }
    return sbot_norm_heading(out);
}

static bool sbot_auto_drive_to(const SbotPoint& p, uint32_t timeout_ms, SbotAutoSide side, SbotAutoAlliance alliance, bool forwards = true) {
    if (!validateSbotLemLibInitialization()) return false;
    if (!sbot_chassis) return false;

    const SbotPoint target = sbot_apply_auto_transform(p, side, alliance);

    lemlib::MoveToPointParams params;
    params.forwards = forwards;
    sbot_chassis->moveToPoint(target.x, target.y, timeout_ms, params);
    sbot_chassis->waitUntilDone();
    return true;
}

static bool sbot_auto_turn_to(double heading_deg, uint32_t timeout_ms, SbotAutoSide side, SbotAutoAlliance alliance) {
    if (!validateSbotLemLibInitialization()) return false;
    if (!sbot_chassis) return false;

    const double target_heading = sbot_apply_auto_transform_heading(heading_deg, side, alliance);
    sbot_chassis->turnToHeading(target_heading, timeout_ms);
    sbot_chassis->waitUntilDone();
    return true;
}

struct SbotRRPathTuning {
    // Points are in a local, start-relative frame:
    // - Start pose is (0,0,0)
    // LemLib convention:
    // - +Y is "away from the goal" into the field (forward)
    // - +X is "to the robot's right" when facing into the field
    // These are first-pass guesses and should be tuned.

    // Step points
    SbotPoint step1;
    SbotPoint step2;
    SbotPoint step4;

    // Headings
    double step3_turn_heading_deg;

    // Timeouts
    uint32_t drive_timeout_ms;
    uint32_t turn_timeout_ms;

    // Scoring
    uint32_t top_score_ms;
};

static SbotRRPathTuning sbot_rr_default_tuning() {
    SbotRRPathTuning t;

    // Match the user sketch (Red Right):
    // 1) pull away from goal
    // 2) angle to the near cluster
    // 3) turn because intake and scoring are opposite sides
    // 4) return/approach goal for scoring
    // 5) score top goal
    // Previously authored as (forward, left). Convert to LemLib (right, forward):
    // new.x = -old.left
    // new.y = old.forward
    t.step1 = {0, 18};
    t.step2 = {-14, 40};
    t.step3_turn_heading_deg = 180; // turn around so scoring side leads
    t.step4 = {-4, 10};             // approach the goal/parking area (tune)

    t.drive_timeout_ms = 2500;
    t.turn_timeout_ms = 1600;
    t.top_score_ms = 900;
    return t;
}

static void sbot_run_red_right_1_to_5(SbotAutoSide side, SbotAutoAlliance alliance) {
    // Single source of truth for match autos. All 4 match autos call into here with transforms.
    // This follows the user's drawn steps 1→5 for Red Right; LEFT mirrors; BLUE is 180° rotated.
    printf("SBOT AUTON: MATCH AUTO RR-1to5 (%s %s)\n",
           (alliance == SbotAutoAlliance::RED) ? "RED" : "BLUE",
           (side == SbotAutoSide::RIGHT) ? "RIGHT" : "LEFT");

    if (!validateSbotLemLibInitialization()) return;

    sbot_safe_stop_mechanisms();
    sbot_set_match_start_pose();
    sbot_print_pose("start");

    const auto tune = sbot_rr_default_tuning();

    // Step 1: Start collecting immediately (store inside robot while driving out)
    printf("RR STEP 1\n");
    sbot_intake_on_storage();
    sbot_auto_drive_to(tune.step1, tune.drive_timeout_ms, side, alliance, true);
    sbot_print_pose("after step1");

    // Step 2: Continue collecting to the cluster
    printf("RR STEP 2\n");
    sbot_intake_on_storage();
    sbot_auto_drive_to(tune.step2, tune.drive_timeout_ms, side, alliance, true);
    sbot_print_pose("after step2");

    // Step 3: Turn so the scoring side is oriented correctly
    // (intake side and scoring side are opposite sides)
    printf("RR STEP 3\n");
    sbot_auto_turn_to(tune.step3_turn_heading_deg, tune.turn_timeout_ms, side, alliance);
    sbot_print_pose("after step3");

    // Step 4: Approach the goal/parking strip while keeping balls staged.
    // NOTE: Depending on how your mechanism scores (front vs back), you may flip `forwards`.
    printf("RR STEP 4\n");
    sbot_intake_on_storage();
    sbot_auto_drive_to(tune.step4, tune.drive_timeout_ms, side, alliance, false /* backwards */);
    sbot_print_pose("before score");

    // Step 5: Score top goal
    printf("RR STEP 5: TOP SCORE\n");
    sbot_score_top_for(tune.top_score_ms);
    sbot_print_pose("after top score");

    sbot_safe_stop_mechanisms();
    printf("SBOT AUTON: MATCH AUTO RR-1to5 complete\n");
}

static void sbot_run_match_auto(SbotAutoSide side, SbotAutoAlliance alliance) {
    sbot_run_red_right_1_to_5(side, alliance);
}

static void sbot_run_skills_auto() {
    printf("SBOT AUTON: SKILLS (first-pass)\n");
    if (!validateSbotLemLibInitialization()) return;

    sbot_safe_stop_mechanisms();
    sbot_set_match_start_pose();
    sbot_print_pose("skills start");

    const uint32_t start_ms = pros::millis();
    const uint32_t hard_stop_ms = 55000; // leave time buffer

    int cycle = 0;
    while (pros::millis() - start_ms < hard_stop_ms) {
        cycle++;
        printf("SBOT SKILLS: cycle %d\n", cycle);

        // Collect run
        sbot_intake_on_storage();
        sbot_drive_to({0, 36}, 3000, false);

        // Score mid
        sbot_turn_to(90, 1500, false);
        sbot_drive_relative(8, 1200, true);
        sbot_score_mid_for(SBOT_MID_GOAL_SCORE_TIME_MS);

        // Go "back" to start area
        sbot_turn_to(270, 1500, false);
        sbot_drive_to({0, 0}, 3000, false, false);

        // Small top feed attempt
        sbot_turn_to(0, 1500, false);
        sbot_drive_relative(10, 1500, true);
        sbot_score_top_for(750);

        // Reset for next cycle
        sbot_turn_to(180, 1500, false);
        sbot_drive_relative(10, 1500, true);
        sbot_turn_to(0, 1500, false);
        sbot_safe_stop_mechanisms();

        // Prevent tight looping if time is nearly up
        pros::delay(100);
    }

    sbot_safe_stop_mechanisms();
    printf("SBOT AUTON: SKILLS complete\n");
}

static const char* sbot_mode_name(int idx) {
    static const char* mode_names[] = {
        "DISABLED",     // 0
        "Red Left",     // 1
        "Red Right",    // 2
        "Blue Left",    // 3
        "Blue Right",   // 4
        "Skills",       // 5
        "Test: Drive",  // 6
        "Test: Turn",   // 7
        "Test: Intake", // 8
        "Test: Indexer" // 9
    };

    if (idx < 0) return "<invalid>";
    if (idx > 9) return "<invalid>";
    return mode_names[idx];
}

// =========================== Selector ===============================

SbotAutoSelector::SbotAutoSelector()
    : selected_mode(SbotAutoMode::DISABLED),
      selector_position(0),
      mode_confirmed(false) {}

void SbotAutoSelector::displayOptions() {
    // Display on controller screen (legacy project behavior)
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    if (!master.is_connected()) return;

    const int idx = selector_position;
    const char* name = sbot_mode_name(idx);

    if (mode_confirmed) {
        master.print(0, 0, "READY: %s", name);
        master.print(1, 0, "A: change  L/R: nav");
    } else {
        master.print(0, 0, "%d: %s", idx, name);
        master.print(1, 0, "L/R: change  A: ok");
    }
}

void SbotAutoSelector::handleInput() {
    pros::Controller master(pros::E_CONTROLLER_MASTER);

    static int last_pos = -1;
    static bool last_confirmed = false;

    bool left_pressed = master.get_digital_new_press(SBOT_AUTO_PREV_BTN);
    bool right_pressed = master.get_digital_new_press(SBOT_AUTO_NEXT_BTN);
    bool a_pressed = master.get_digital_new_press(SBOT_AUTO_CONFIRM_BTN);

    const int max_index = 9; // 0..9

    if (!mode_confirmed) {
        if (left_pressed) {
            selector_position--;
            if (selector_position < 0) selector_position = max_index;
        }
        if (right_pressed) {
            selector_position++;
            if (selector_position > max_index) selector_position = 0;
        }
        if (a_pressed) {
            selected_mode = static_cast<SbotAutoMode>(selector_position);
            mode_confirmed = true;
        }
    } else {
        if (a_pressed) {
            mode_confirmed = false; // allow reselection
        }
    }

    // Print only on change to avoid spamming the terminal.
    if (selector_position != last_pos || mode_confirmed != last_confirmed) {
        last_pos = selector_position;
        last_confirmed = mode_confirmed;

        if (mode_confirmed) {
            printf("SBOT AUTO: READY %d (%s)\n", selector_position, sbot_mode_name(selector_position));
        } else {
            printf("SBOT AUTO: select %d (%s) [L/R to change, A to confirm]\n",
                   selector_position,
                   sbot_mode_name(selector_position));
        }
    }
}

bool SbotAutoSelector::update() {
    handleInput();
    displayOptions();
    return mode_confirmed;
}

// ======================= Autonomous System ==========================

SbotAutonomousSystem::SbotAutonomousSystem() {}

void SbotAutonomousSystem::initialize() {
    // Initialize LemLib for sbot (safe to call once)
    initializeSbotLemLib();
}

void SbotAutonomousSystem::updateSelector() {
    selector.update();
}

void SbotAutonomousSystem::run() {
    SbotAutoMode mode = selector.getMode();

    switch (mode) {
        case SbotAutoMode::RED_LEFT:   runRedLeft();   break;
        case SbotAutoMode::RED_RIGHT:  runRedRight();  break;
        case SbotAutoMode::BLUE_LEFT:  runBlueLeft();  break;
        case SbotAutoMode::BLUE_RIGHT: runBlueRight(); break;
        case SbotAutoMode::SKILLS:     runSkills();    break;
        case SbotAutoMode::TEST_DRIVE:   runTestDrive();   break;
        case SbotAutoMode::TEST_TURN:    runTestTurn();    break;
        case SbotAutoMode::TEST_INTAKE:  runTestIntake();  break;
        case SbotAutoMode::TEST_INDEXER: runTestIndexer(); break;
        case SbotAutoMode::DISABLED:
        default:
            // Do nothing
            break;
    }
}

// ---- Match autonomous stubs ----

void SbotAutonomousSystem::runRedLeft() {
    sbot_run_match_auto(SbotAutoSide::LEFT, SbotAutoAlliance::RED);
}

void SbotAutonomousSystem::runRedRight() {
    sbot_run_match_auto(SbotAutoSide::RIGHT, SbotAutoAlliance::RED);
}

void SbotAutonomousSystem::runBlueLeft() {
    sbot_run_match_auto(SbotAutoSide::LEFT, SbotAutoAlliance::BLUE);
}

void SbotAutonomousSystem::runBlueRight() {
    sbot_run_match_auto(SbotAutoSide::RIGHT, SbotAutoAlliance::BLUE);
}

void SbotAutonomousSystem::runSkills() {
    sbot_run_skills_auto();
}

// ---- Test autonomous stubs ----

void SbotAutonomousSystem::runTestDrive() {
    printf("SBOT AUTON TEST: RECTANGLE (replaces drive test)\n");
    if (!validateSbotLemLibInitialization()) return;

    sbot_safe_stop_mechanisms();

    // Continuous path: use coast so it doesn't hard-stop at each corner.
    if (sbot_chassis) sbot_chassis->setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

    sbot_print_pose("before start");
    sbot_print_sensors("before start");

    // Reset frame
    sbot_zero_pose_and_sensors(0, 0, 0);
    sbot_print_pose("start");
    sbot_print_sensors("start");

    // Rectangle: 2 VEX tiles per side.
    // Field tiles are 24", so 2 tiles = 48".
    constexpr float kLegIn = 48.0f;
    constexpr int kTimeoutMs = 9500;

    // LemLib convention: +Y forward at 0°, +X right.
    // We use boomerang (moveToPose) to blend the corner turn into the approach so it doesn't stop.
    lemlib::MoveToPoseParams params;
    params.forwards = true;
    params.maxSpeed = 60;      // reduced speed
    params.minSpeed = 0;       // prioritize reaching the actual corner (less corner-cutting)
    params.earlyExitRange = 0; // no early exit
    params.lead = 0.35;        // straighter approach

    struct Target {
        float x;
        float y;
        float theta;
        const char* label;
    };

    // Use segment-aligned headings so the robot doesn't start by turning.
    // Heading convention in this project: 0° is +Y (forward); clockwise-positive.
    // Segment headings (left turns): 0 -> -90 -> 180 -> 90.
    const Target targets[] = {
        {0,       kLegIn,   0,   "corner1"},
        {-kLegIn, kLegIn,  -90,  "corner2"},
        {-kLegIn, 0,       180,  "corner3"},
        {0,       0,        90,  "back home"}
    };

    for (int i = 0; i < 4; i++) {
        const auto before = sbot_chassis->getPose();

        sbot_chassis->moveToPose(targets[i].x, targets[i].y, targets[i].theta, kTimeoutMs, params);
        sbot_chassis->waitUntilDone();

        const auto after = sbot_chassis->getPose();
        printf(
            "SBOT RECT: %s delta: dx=%.2f dy=%.2f dth=%.2f\n",
            targets[i].label,
            after.x - before.x,
            after.y - before.y,
            after.theta - before.theta
        );
        sbot_print_pose(targets[i].label);
        sbot_print_sensors(targets[i].label);
    }

    // Optional: square ends with a final left turn to face 0° again.
    lemlib::TurnToHeadingParams leftTurnParams;
    leftTurnParams.direction = AngularDirection::CCW_COUNTERCLOCKWISE;
    leftTurnParams.maxSpeed = 70;
    leftTurnParams.minSpeed = 15;
    sbot_chassis->turnToHeading(0, 2500, leftTurnParams);
    sbot_chassis->waitUntilDone();

    // 3) Reverse sequence, driving backwards ("back direction")
    // Start with a right turn, then drive backwards 2 tiles, and repeat.
    printf("SBOT AUTON TEST: RECTANGLE reverse (backwards)\n");

    lemlib::MoveToPoseParams backParams = params;
    backParams.forwards = false;
    backParams.maxSpeed = 55; // a little slower for backwards driving

    // Reverse traversal waypoints (clockwise), but driven backwards:
    // (0,0,0) -> (-48,0,90) -> (-48,48,180) -> (0,48,270) -> (0,0,0)
    const Target backTargets[] = {
        {-kLegIn, 0,       90,  "back corner1"},
        {-kLegIn, kLegIn,  180, "back corner2"},
        {0,       kLegIn,  270, "back corner3"},
        {0,       0,       0,   "back home"}
    };

    for (int i = 0; i < 4; i++) {
        const auto before = sbot_chassis->getPose();

        sbot_chassis->moveToPose(backTargets[i].x, backTargets[i].y, backTargets[i].theta, kTimeoutMs, backParams);
        sbot_chassis->waitUntilDone();

        const auto after = sbot_chassis->getPose();
        printf(
            "SBOT RECT BACK: %s delta: dx=%.2f dy=%.2f dth=%.2f\n",
            backTargets[i].label,
            after.x - before.x,
            after.y - before.y,
            after.theta - before.theta
        );
        sbot_print_pose(backTargets[i].label);
        sbot_print_sensors(backTargets[i].label);
    }

    sbot_print_pose("end");
    sbot_print_sensors("end");

    sbot_safe_stop_mechanisms();
}

void SbotAutonomousSystem::runTestTurn() {
    printf("SBOT AUTON TEST: TURN\n");
    if (!validateSbotLemLibInitialization()) return;

    sbot_safe_stop_mechanisms();

    sbot_zero_pose_and_sensors(0, 0, 0);
    sbot_print_pose("after setPose");
    sbot_chassis->turnToHeading(90, 3000);
    sbot_chassis->waitUntilDone();
    sbot_print_pose("after 90");
    sbot_chassis->turnToHeading(0, 3000);
    sbot_chassis->waitUntilDone();
    sbot_print_pose("after 0");

    sbot_safe_stop_mechanisms();
}

void SbotAutonomousSystem::runTestIntake() {
    printf("SBOT AUTON TEST: INTAKE\n");

    sbot_safe_stop_mechanisms();

    if (!sbot_intake) {
        printf("SBOT AUTON TEST: INTAKE missing sbot_intake\n");
        return;
    }

    // Forward intake
    sbot_intake->setMode(IntakeMode::COLLECT_FORWARD);
    sbot_run_for_ms(800);

    // Stop
    sbot_intake->setMode(IntakeMode::OFF);
    sbot_run_for_ms(150);

    printf("SBOT AUTON TEST: INTAKE done\n");
}

void SbotAutonomousSystem::runTestIndexer() {
    printf("SBOT AUTON TEST: INDEXER\n");

    sbot_safe_stop_mechanisms();

    if (!sbot_indexer) {
        printf("SBOT AUTON TEST: INDEXER missing sbot_indexer\n");
        return;
    }

    // Forward feed
    sbot_indexer->setMode(IndexerMode::FEED_FORWARD);
    sbot_run_for_ms(650);

    // Reverse briefly (middle/eject direction)
    sbot_indexer->setMode(IndexerMode::FEED_BACKWARD_MIDDLE);
    sbot_run_for_ms(350);

    // Stop
    sbot_indexer->setMode(IndexerMode::OFF);
    sbot_run_for_ms(150);

    printf("SBOT AUTON TEST: INDEXER done\n");
}
