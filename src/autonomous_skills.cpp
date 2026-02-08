/**
 * autonomous_skills.cpp
 *
 * Skills autonomous routine for sbot.
 */
#include "autonomous_skills.h"

#include "autonomous_infrastructure.h"
#include "autonomous_constants.h"
#include "config_sbot.h"
#include "lemlib_config_sbot.h"

#include <cmath>
#include <cstdio>

static bool sbot_drive_relative_skills(double distance_in, uint32_t timeout_ms, bool forwards = true) {
    if (!validateSbotLemLibInitialization() || !sbot_chassis) return false;

    const auto pose = sbot_chassis->getPose();
    const double heading_rad = pose.theta * M_PI / 180.0;
    // LemLib odom convention: at theta=0°, robot faces +Y. (+X is right)
    const double dx = distance_in * std::sin(heading_rad);
    const double dy = distance_in * std::cos(heading_rad);

    const double target_x = forwards ? (pose.x + dx) : (pose.x - dx);
    const double target_y = forwards ? (pose.y + dy) : (pose.y - dy);

    lemlib::MoveToPointParams params;
    params.forwards = forwards;
    params.maxSpeed = SBOT_MATCH_MAX_SPEED;
    sbot_chassis->moveToPoint(target_x, target_y, timeout_ms, params);
    sbot_wait_until_done_timed("skills.drive_relative");
    return true;
}

void sbot_run_skills_auto() {
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
        sbot_drive_relative_skills(8, 1200, true);
        sbot_score_mid_for(SBOT_MID_GOAL_SCORE_TIME_MS);

        // Go "back" to start area
        sbot_turn_to(270, 1500, false);
        sbot_drive_to({0, 0}, 3000, false, false);

        // Small top feed attempt
        sbot_turn_to(0, 1500, false);
        sbot_drive_relative_skills(10, 1500, true);
        sbot_score_top_for(750);

        // Reset for next cycle
        sbot_turn_to(180, 1500, false);
        sbot_drive_relative_skills(10, 1500, true);
        sbot_turn_to(0, 1500, false);
        sbot_safe_stop_mechanisms();

        // Prevent tight looping if time is nearly up
        pros::delay(100);
    }

    sbot_safe_stop_mechanisms();
    printf("SBOT AUTON: SKILLS complete\n");
}
