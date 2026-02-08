/**
 * autonomous_tests.cpp
 *
 * Autonomous test routines for sbot.
 */
#include "autonomous_sbot.h"

#include "autonomous_constants.h"
#include "autonomous_match_awp.h"
#include "autonomous_match_helpers.h"
#include "autonomous_infrastructure.h"
#include "lemlib_config_sbot.h"

#include "intake.h"
#include "indexer.h"
#include "pneumatics.h"

#include <cmath>
#include <cstdio>

// LemLib path-follow assets must be declared at global scope.
ASSET(low_txt);

void SbotAutonomousSystem::runTestJerryPoseMonitor() {
    // Manual calibration helper: push/rotate the robot by hand and watch odometry.
    // Prints pose (x,y,theta) and IMU heading on the controller screen.
    printf("SBOT AUTON TEST: Pose Monitor (controller display)\n");

    if (!validateSbotLemLibInitialization() || !sbot_chassis) {
        printf("SBOT Pose Monitor: LemLib/chassis not initialized\n");
        pros::Controller master(pros::E_CONTROLLER_MASTER);
        if (master.is_connected()) {
            master.print(0, 0, "POSE MON: no chassis");
            master.print(1, 0, "Check LemLib init");
        }
        pros::delay(1500);
        return;
    }

    sbot_safe_stop_mechanisms();

    // Easier to push around by hand.
    if (sbot_chassis) sbot_chassis->setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

    // Deterministic frame for calibration.
    sbot_zero_pose_and_sensors(0, 0, 0);
    pros::delay(50);

    pros::Controller master(pros::E_CONTROLLER_MASTER);
    if (master.is_connected()) {
        master.clear();
        master.print(0, 0, "POSE MON (B=exit)");
        master.print(1, 0, "Move robot by hand");
    }
    pros::delay(600);

    uint32_t last_controller_ms = 0;
    uint32_t last_terminal_ms = 0;

    while (!master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
        const uint32_t now = pros::millis();
        const auto pose = sbot_chassis->getPose();
        const double imu_h = sbot_get_best_heading_deg();

        if (now - last_controller_ms >= 100) {
            last_controller_ms = now;
            if (master.is_connected()) {
                // Controller screen is tight: keep it compact.
                master.print(0, 0, "x%6.2f y%6.2f", pose.x, pose.y);
                master.print(1, 0, "th%6.1f imu%5.1f", pose.theta, imu_h);
            }
        }

        if (now - last_terminal_ms >= 500) {
            last_terminal_ms = now;
            printf("POSE MON: x=%.2f y=%.2f th=%.1f imu=%.1f\n", pose.x, pose.y, pose.theta, imu_h);
        }

        pros::delay(20);
    }

    if (master.is_connected()) {
        master.clear();
        master.print(0, 0, "POSE MON: exit");
    }

    // Restore typical behavior for other routines.
    if (sbot_chassis) sbot_chassis->setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    pros::delay(250);
}

void SbotAutonomousSystem::runTestPoseFinderX0Line90() {
    // Starting-reference helper:
    // - Physically place the robot on the FIELD x=0 line
    // - Set the robot heading to 90° (LemLib convention: 0° is +Y, 90° is +X)
    // Then move the robot by hand to the desired match start and read off x/y/theta.
    printf("SBOT AUTON TEST: Pose Finder (x=0 line, heading=90)\n");

    if (!validateSbotLemLibInitialization() || !sbot_chassis) {
        printf("SBOT Pose Finder: LemLib/chassis not initialized\n");
        pros::Controller master(pros::E_CONTROLLER_MASTER);
        if (master.is_connected()) {
            master.print(0, 0, "POSE FIND: no chassis");
            master.print(1, 0, "Check LemLib init");
        }
        pros::delay(1500);
        return;
    }

    sbot_safe_stop_mechanisms();

    // Easier to push around by hand.
    sbot_chassis->setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

    // Declare the current physical placement as (0,0,90).
    // NOTE: this assumes the robot is already placed on the x=0 line with 90° heading when you start this test.
    sbot_zero_pose_and_sensors(0, 0, 90);
    pros::delay(50);

    pros::Controller master(pros::E_CONTROLLER_MASTER);
    if (master.is_connected()) {
        master.clear();
        master.print(0, 0, "POSE FIND (B=exit)");
        master.print(1, 0, "Start x0 th90, move");
    }
    pros::delay(700);

    uint32_t last_controller_ms = 0;
    uint32_t last_terminal_ms = 0;

    while (!master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
        const uint32_t now = pros::millis();
        const auto pose = sbot_chassis->getPose();

        // "Jerry-style" mapping, but RELATIVE to this test's start.
        // We intentionally want the display to read (0,0) at the moment we set pose to (0,0,90)
        // on the x=0 line.
        // Mapping (relative):
        //   jx = ourY
        //   jy = -ourX
        const double jerry_x = pose.y;
        const double jerry_y = -pose.x;

        if (now - last_controller_ms >= 100) {
            last_controller_ms = now;
            if (master.is_connected()) {
                master.print(0, 0, "jx%7.2f jy%7.2f", jerry_x, jerry_y);
                master.print(1, 0, "th%6.1f", pose.theta);
            }
        }

        if (now - last_terminal_ms >= 500) {
            last_terminal_ms = now;
            printf("POSE FIND: rel_jerry(%.2f,%.2f) th=%.1f\n", jerry_x, jerry_y, pose.theta);
        }

        pros::delay(20);
    }

    if (master.is_connected()) {
        master.clear();
        master.print(0, 0, "POSE FIND: exit");
    }

    // Restore typical behavior for other routines.
    sbot_chassis->setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    pros::delay(250);
}

void SbotAutonomousSystem::runTestFollowJerryPath() {
    printf("SBOT AUTON TEST: FOLLOW PATH (LemLib follow)\n");
    if (!validateSbotLemLibInitialization() || !sbot_chassis) return;

    sbot_safe_stop_mechanisms();

    // Start collecting immediately.
    sbot_intake_on_storage();

    // For following a continuous path, COAST avoids hard stops.
    sbot_chassis->setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

    // For LemLib follow(), the robot pose frame must match the path file's coordinate frame.
    // IMPORTANT: keep IMU heading/rotation consistent with the chassis pose heading.
    // This test path is authored in the same absolute Jerry frame as Red Left start.
    // Match the first waypoint in static/low.txt so follow() doesn't start with a lateral offset.
    sbot_zero_pose_and_sensors(-50.23f, 15.31f, 90.0f);

    // Record starting IMU heading for post-follow turns.
    // All turns will be calculated as offsets from this initial calibration.
    const double start_imu_heading = sbot_chassis->getPose(false, true).theta;  // Should be ~0° when pose=90°
    constexpr double kStartPoseHeading = 90.0;
    printf("SBOT FOLLOW START: pose=%.1f, std=%.1f\n", kStartPoseHeading, start_imu_heading);

    sbot_print_pose("before follow");
    sbot_print_sensors("before follow");

    printf("SBOT FOLLOW: asset bytes=%u\n", static_cast<unsigned>(low_txt.size));

    // Keep LemLib logs quiet for normal runs.
    lemlib::infoSink()->setLowestLevel(lemlib::Level::WARN);

    // Follow the compiled path asset.
    // Note: lookahead is in inches.
    // Smaller lookahead => tighter tracking (often slower/more oscillation if too small).
    constexpr float kLookaheadIn = 10.0f;
    // Keep the follow test bounded (~4-5 seconds).
    constexpr int kTimeoutMs = 5000;

    // Run async so we can report whether motion actually starts.
    sbot_chassis->follow(low_txt, kLookaheadIn, kTimeoutMs, true /*forwards*/, true /*async*/);

    const uint32_t start_ms = pros::millis();
    bool ever_in_motion = false;
    bool printed_end = false;

    // Keep waiting up to the timeout window, but avoid spamming the terminal.
    while (pros::millis() - start_ms < static_cast<uint32_t>(kTimeoutMs + 250)) {
        const bool in_motion = sbot_chassis->isInMotion();
        if (in_motion) ever_in_motion = true;

        sbot_trace_follow_progress(start_ms, pros::millis());

        if (!in_motion && ever_in_motion && !printed_end) {
            printed_end = true;
            printf("SBOT FOLLOW: motion complete at t=%u ms\n",
                   static_cast<unsigned>(pros::millis() - start_ms));
        }

        pros::delay(20);
    }

    // Wait for completion (if LemLib actually queued/runs a motion, this blocks).
    sbot_chassis->waitUntilDone();

    sbot_print_pose("after follow path");
    sbot_print_sensors("after follow path");

    // --- Manual angle finding mode ---
    // Loop to let you manually position the robot and read the IMU angles
    printf("\n=== ANGLE FINDER MODE ===\n");
    printf("Manually rotate robot to desired positions and note the IMU heading.\n");
    printf("Press B button to exit and continue.\n\n");

    pros::Controller master(pros::E_CONTROLLER_MASTER);
    uint32_t last_print = pros::millis();

    while (!master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
        const uint32_t now = pros::millis();

        if (now - last_print >= 200) {  // Print 5 times per second
            last_print = now;

            const auto pose = sbot_chassis->getPose(false, false);  // pose frame
            const auto std = sbot_chassis->getPose(false, true);    // std frame
            const double imu_h = sbot_inertial_sensor ? sbot_inertial_sensor->get_heading() : 0.0;

            printf("x=%.2f y=%.2f | pose.th=%.1f std.th=%.1f imu.h=%.1f\n",
                   pose.x, pose.y, pose.theta, std.theta, imu_h);
        }

        pros::delay(20);
    }

    printf("\n=== ANGLE FINDER MODE EXIT ===\n");
    printf("Press B again to skip remaining autonomous and go to driver control.\n\n");

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
        // Skip the rest of autonomous
        sbot_safe_stop_mechanisms();
        sbot_chassis->setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
        return;
    }

    // --- Post-run sequence (final) ---
    // Using manually measured IMU headings for each desired robot orientation.
    // All angles are absolute IMU values, eliminating coordinate frame confusion.
    printf("SBOT POSTFOLLOW: Starting post-follow sequence with measured IMU targets\n");

    // Keep post-follow actions at low speed for safety and repeatability.
    constexpr int kPostTurnMaxSpeed = SBOT_MATCH_TURN_MAX_SPEED / 2;
    constexpr int kPostDriveMaxSpeed = SBOT_MATCH_MAX_SPEED / 2;

    lemlib::TurnToHeadingParams post_turn_params;
    post_turn_params.maxSpeed = kPostTurnMaxSpeed;
    post_turn_params.minSpeed = 10;

    lemlib::MoveToPointParams post_drive_params;
    post_drive_params.forwards = true;
    post_drive_params.maxSpeed = kPostDriveMaxSpeed;
    post_drive_params.minSpeed = 0;

    // 1) Wait after follow completes
    pros::delay(250);

    // 2) Turn to face away from start (imu=268°)
    printf("SBOT POSTFOLLOW: turn to imu=268 (face away)\n");
    sbot_chassis->turnToHeading(268.0, 2500, post_turn_params);
    sbot_wait_until_done_timed("postfollow.turn_away");

    // 3) Drop loader down and wait
    if (sbot_batch_loader) sbot_batch_loader->extend();
    pros::delay(1000);

    // 4) Turn to point back toward starting area (imu=313°)
    printf("SBOT POSTFOLLOW: turn to imu=313\n");
    sbot_chassis->turnToHeading(313.0, 2500, post_turn_params);
    sbot_wait_until_done_timed("postfollow.turn_toward_start");

    // 5) Back to (-24, 24) maintaining heading imu=310°
    printf("SBOT POSTFOLLOW: back to (-24, 24) at imu=310\n");
    post_drive_params.forwards = false;  // BACKWARDS
    sbot_match_turn_point_turn(
        "postfollow_to_-24_24",
        -24.0f,
        24.0f,
        310.0f,
        2500,
        4000,
        post_turn_params,
        post_drive_params
    );
    post_drive_params.forwards = true;  // Reset to default

    // 6) Turn to imu=38°
    printf("SBOT POSTFOLLOW: turn to imu=38\n");
    sbot_chassis->turnToHeading(38.0, 2500, post_turn_params);
    sbot_wait_until_done_timed("postfollow.turn_to_38");

    // Lift matchloader after turning
    if (sbot_batch_loader) sbot_batch_loader->retract();

    // 7) Move to (-12, 24) at imu=39°
    printf("SBOT POSTFOLLOW: move to (-12, 24) at imu=39\n");
    sbot_match_turn_point_turn(
        "postfollow_to_-12_24",
        -12.0f,
        24.0f,
        39.0f,
        2500,
        4000,
        post_turn_params,
        post_drive_params,
        0,
        0,
        0.0,
        0.0,
        false,
        true
    );

    // 8) Turn to imu=170°
    printf("SBOT POSTFOLLOW: turn to imu=170\n");
    sbot_chassis->turnToHeading(170.0, 2500, post_turn_params);
    sbot_wait_until_done_timed("postfollow.turn_to_170");

    // 9) Low score
    sbot_match_score_low_for(1500);

    // LemLib logs remain at WARN.

    // Stop mechanisms after the post-run actions.
    sbot_safe_stop_mechanisms();

    sbot_print_pose("after follow");
    sbot_print_sensors("after follow");

    sbot_safe_stop_mechanisms();
    sbot_chassis->setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
}

void SbotAutonomousSystem::runTestSweepToLowGoal() {
    // Focused test: start from the end of the cluster sweep and run ONLY the Center Lower (front) score.
    // Uses the exact same Stage 2 logic/params as match auton.
    printf("SBOT AUTON TEST: SWEEP -> CENTER LOWER (front score)\n");
    sbot_run_match_auto(
        SbotAutoSide::LEFT,
        SbotAutoAlliance::RED,
        false /* solo_awp */,
        true /* start_from_cluster_sweep */,
        true /* stop_after_stage2 */,
        true /* stage2_skip_pre_turn */
    );
}

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

    lemlib::TurnToHeadingParams turnParams;
    turnParams.maxSpeed = 70;
    turnParams.minSpeed = 0;

    lemlib::MoveToPointParams params;
    params.forwards = true;
    params.maxSpeed = 60;      // reduced speed
    params.minSpeed = 0;       // prioritize reaching the actual corner
    params.earlyExitRange = 0; // no early exit

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

        char label[64];
        std::snprintf(label, sizeof(label), "test.drive.rect.%s", targets[i].label);
        sbot_match_turn_point_turn(
            label,
            targets[i].x,
            targets[i].y,
            targets[i].theta,
            2500,
            static_cast<uint32_t>(kTimeoutMs),
            turnParams,
            params,
            static_cast<uint32_t>(kTimeoutMs + 750),
            250,
            1.0,
            5.0
        );

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
    leftTurnParams.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE;
    leftTurnParams.maxSpeed = 70;
    leftTurnParams.minSpeed = 15;
    sbot_chassis->turnToHeading(0, 2500, leftTurnParams);
    sbot_wait_until_done_timed("test.drive.rect.turn_to_0");

    // 3) Reverse sequence, driving backwards ("back direction")
    // Start with a right turn, then drive backwards 2 tiles, and repeat.
    printf("SBOT AUTON TEST: RECTANGLE reverse (backwards)\n");

    lemlib::MoveToPointParams backParams = params;
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

        char label[64];
        std::snprintf(label, sizeof(label), "test.drive.rect.back.%s", backTargets[i].label);
        sbot_match_turn_point_turn(
            label,
            backTargets[i].x,
            backTargets[i].y,
            backTargets[i].theta,
            2500,
            static_cast<uint32_t>(kTimeoutMs),
            turnParams,
            backParams,
            static_cast<uint32_t>(kTimeoutMs + 750),
            250,
            1.0,
            7.0
        );

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

    // Restore BRAKE so subsequent match autons/tests don't inherit COAST behavior.
    if (sbot_chassis) sbot_chassis->setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
}

void SbotAutonomousSystem::runTestDriveShort() {
    // Short drive test: fixed start/end pose.
    printf("SBOT AUTON TEST: SHORT DRIVE\n");
    if (!validateSbotLemLibInitialization()) return;

    sbot_safe_stop_mechanisms();

    // Continuous path: use coast so it doesn't hard-stop at each corner.
    if (sbot_chassis) sbot_chassis->setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

    sbot_print_pose("before start");
    sbot_print_sensors("before start");

    // Reset frame: start at (0,0,0)
    sbot_zero_pose_and_sensors(0, 0, 0);
    sbot_print_pose("start");
    sbot_print_sensors("start");

    // Target pose: end at (9,9,45)
    constexpr float kTargetX = 9.0f;
    constexpr float kTargetY = 9.0f;
    constexpr float kTargetThetaDeg = 45.0f;
    constexpr int kTimeoutMs = 4500;
    printf("SBOT SHORT DRIVE: start(0,0,0) -> target(%.2f,%.2f,%.1f)\n", kTargetX, kTargetY, kTargetThetaDeg);

    lemlib::TurnToHeadingParams turnParams;
    turnParams.maxSpeed = 70;
    turnParams.minSpeed = 0;

    lemlib::MoveToPointParams params;
    params.forwards = true;
    params.maxSpeed = 60;      // reduced speed
    params.minSpeed = 0;       // prioritize reaching the actual corner
    params.earlyExitRange = 0; // no early exit

    const auto before = sbot_chassis->getPose();
    sbot_match_turn_point_turn(
        "test.drive.short.target",
        kTargetX,
        kTargetY,
        kTargetThetaDeg,
        2500,
        static_cast<uint32_t>(kTimeoutMs),
        turnParams,
        params,
        static_cast<uint32_t>(kTimeoutMs + 500),
        200,
        0.75,
        7.0
    );

    const auto after = sbot_chassis->getPose();
    printf(
        "SBOT SHORT DRIVE delta: dx=%.2f dy=%.2f dth=%.2f\n",
        after.x - before.x,
        after.y - before.y,
        after.theta - before.theta
    );
    sbot_print_pose("after short drive");
    sbot_print_sensors("after short drive");

    sbot_print_pose("end");
    sbot_print_sensors("end");

    sbot_safe_stop_mechanisms();

    // Restore BRAKE so subsequent match autons/tests don't inherit COAST behavior.
    if (sbot_chassis) sbot_chassis->setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
}

void SbotAutonomousSystem::runTestLowGoalCustomStart() {
    printf("SBOT AUTON TEST: Low Goal (custom start)\n");
    if (!validateSbotLemLibInitialization()) return;

    sbot_safe_stop_mechanisms();
    if (sbot_chassis) sbot_chassis->setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);

    // You will place the robot physically, then we declare that placement as the start pose here.
    // Update these constants between runs.
    constexpr float kStartX = 0.0f;
    constexpr float kStartY = 0.0f;
    constexpr float kStartThetaDeg = 0.0f;

    // Red Left Center Lower (front score) pose target (same as match logic contact->pose):
    // contact(9.0, 40.5) heading=45°, front=SBOT_FRONT_BUMPER_IN => pose(3.70, 35.20)
    const SbotPoint target = sbot_match_pose_from_front_contact({9.0, 40.5}, 45.0, SBOT_FRONT_BUMPER_IN);
    constexpr double kTargetHeadingDeg = 45.0;

    // Reset sensors/odom for deterministic starting state.
    sbot_zero_pose_and_sensors(0, 0, 0);
    pros::delay(40);
    sbot_chassis->setPose(kStartX, kStartY, kStartThetaDeg);
    pros::delay(40);
    sbot_chassis->setPose(kStartX, kStartY, kStartThetaDeg);

    sbot_print_pose("custom start");
    sbot_print_sensors("custom start");

    printf(
        "SBOT TEST: start(%.2f,%.2f,%.1f) -> target(%.2f,%.2f,%.1f)\n",
        kStartX,
        kStartY,
        kStartThetaDeg,
        target.x,
        target.y,
        kTargetHeadingDeg
    );
    sbot_print_jerry_target("test_low_goal_pose_target", target.x, target.y);

    lemlib::TurnToHeadingParams turnParams;
    turnParams.maxSpeed = 70;
    turnParams.minSpeed = 0;

    lemlib::MoveToPointParams params;
    params.forwards = true;
    params.maxSpeed = 95;
    params.minSpeed = 0;
    params.earlyExitRange = 0;

    // Make LemLib's internal timeout long so our wait loop determines whether it converged.
    const uint32_t motion_timeout_ms = 15000;
    const uint32_t wait_timeout_ms = 6000;

    sbot_lemlib_debug_window_begin("test.low_goal_custom_start");
    sbot_match_turn_point_turn(
        "test.low_goal_custom_start",
        target.x,
        target.y,
        static_cast<float>(kTargetHeadingDeg),
        2500,
        motion_timeout_ms,
        turnParams,
        params,
        wait_timeout_ms,
        500,
        0.5,
        6.0
    );
    sbot_lemlib_debug_window_end("test.low_goal_custom_start");

    sbot_print_pose("after test approach");
    sbot_print_sensors("after test approach");

    sbot_safe_stop_mechanisms();
}

void SbotAutonomousSystem::runTestTurn() {
    printf("SBOT AUTON TEST: TURN\n");
    if (!validateSbotLemLibInitialization()) return;

    sbot_safe_stop_mechanisms();

    // Match-auton turning behavior (avoid post-turn coasting).
    if (sbot_chassis) sbot_chassis->setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);

    // Deterministic sensor + pose frame for turn/odometry tuning.
    // This makes the tracking-wheel offset estimate meaningful across runs.
    sbot_zero_pose_and_sensors(0, 0, 0);
    sbot_print_pose("start");
    sbot_print_sensors("start");

    // During a perfect in-place turn, a vertical tracking wheel that's laterally offset from the
    // rotation center will roll an arc length proportional to the offset and the turn angle:
    //   dWheelIn ~= offsetIn * dThetaRad
    // => offsetIn ~= dWheelIn / dThetaRad
    // This is exactly the value LemLib expects as the TrackingWheel "distance" (left/right offset).
    auto estimate_offset_for_turn = [&](double target_heading_deg, const char* label, double& total_dtheta_rad, double& total_dvert_in) {
        const double imu_rot0 = sbot_inertial_sensor ? sbot_inertial_sensor->get_rotation() : 0.0;
        const double vert0 = sbot_vertical_tracking_wheel ? sbot_vertical_tracking_wheel->getDistanceTraveled() : 0.0;

        lemlib::TurnToHeadingParams params;
        // Slower turn reduces tracking-wheel slip and improves offset estimation stability.
        params.maxSpeed = 50;
        params.minSpeed = 0;

        if (sbot_chassis) sbot_chassis->setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
        sbot_chassis->turnToHeading(target_heading_deg, 3000, params);
        sbot_wait_until_done_timed(label);
        if (sbot_chassis) sbot_chassis->setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);

        const double imu_rot1 = sbot_inertial_sensor ? sbot_inertial_sensor->get_rotation() : 0.0;
        const double vert1 = sbot_vertical_tracking_wheel ? sbot_vertical_tracking_wheel->getDistanceTraveled() : 0.0;

        const double dtheta_deg = imu_rot1 - imu_rot0;
        const double dtheta_rad = dtheta_deg * M_PI / 180.0;
        const double dvert = vert1 - vert0;

        if (std::fabs(dtheta_rad) < 1e-3) {
            printf("SBOT TURN OFFSET EST [%s]: dTheta too small (%.3f deg)\n", label, dtheta_deg);
            return;
        }

        const double offset_est = dvert / dtheta_rad;
        printf(
            "SBOT TURN OFFSET EST [%s]: dTheta=%.2fdeg dVert=%.2fin => offset=%.3fin (sign from sensor)\n",
            label,
            dtheta_deg,
            dvert,
            offset_est
        );

        total_dtheta_rad += dtheta_rad;
        total_dvert_in += dvert;
    };

    // Accumulate multiple quarter-turns to reduce noise/slip effects.
    // Total offset is computed from total dVert / total dTheta, which is much more stable than
    // any single segment.
    double total_dtheta_rad = 0.0;
    double total_dvert_in = 0.0;

    constexpr int kCycles = 3; // 1 cycle = 360deg total
    for (int cycle = 1; cycle <= kCycles; cycle++) {
        char label[64];
        std::snprintf(label, sizeof(label), "test.turn.c%d.to90", cycle);
        estimate_offset_for_turn(90, label, total_dtheta_rad, total_dvert_in);

        std::snprintf(label, sizeof(label), "test.turn.c%d.to180", cycle);
        estimate_offset_for_turn(180, label, total_dtheta_rad, total_dvert_in);

        std::snprintf(label, sizeof(label), "test.turn.c%d.to270", cycle);
        estimate_offset_for_turn(270, label, total_dtheta_rad, total_dvert_in);

        std::snprintf(label, sizeof(label), "test.turn.c%d.to0", cycle);
        estimate_offset_for_turn(0, label, total_dtheta_rad, total_dvert_in);
    }

    if (std::fabs(total_dtheta_rad) > 1e-3) {
        const double offset_total = total_dvert_in / total_dtheta_rad;
        printf(
            "SBOT TURN OFFSET RECOMMEND: total dTheta=%.1fdeg total dVert=%.2fin => offset=%.3fin\n",
            total_dtheta_rad * 180.0 / M_PI,
            total_dvert_in,
            offset_total
        );
        printf(
            "SBOT TURN OFFSET NOTE: set SBOT_TRACKING_WHEEL_DISTANCE to %.3f (or flip sign if drift direction worsens)\n",
            offset_total
        );
    } else {
        printf("SBOT TURN OFFSET RECOMMEND: total dTheta too small; rerun on-field\n");
    }

    sbot_print_pose("after turns");
    sbot_print_sensors("after turns");

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
