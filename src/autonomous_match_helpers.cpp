/**
 * autonomous_match_helpers.cpp
 *
 * Match-specific helper functions with tuned behavior/signatures.
 */
#include "autonomous_match_helpers.h"

#include "autonomous_constants.h"
#include "config_sbot.h"
#include "indexer.h"
#include "intake.h"
#include "lemlib_config_sbot.h"
#include "pneumatics.h"

#include <cmath>
#include <cstdio>

void sbot_match_wait_until_pose_close_or_timeout_timed(
    const char* label,
    uint32_t overall_timeout_ms,
    uint32_t min_time_ms,
    const SbotPoint& target,
    double close_dist_in,
    double target_heading_deg,
    double close_heading_deg
) {
    if (!sbot_chassis) return;

    const uint32_t start = pros::millis();
    bool closed = false;

    uint32_t last_trace_ms = start;
    auto last_trace_pose = sbot_chassis->getPose();
    const double last_trace_vert_in0 = sbot_vertical_tracking_wheel ? sbot_vertical_tracking_wheel->getDistanceTraveled() : 0.0;
    double last_trace_vert_in = last_trace_vert_in0;

    // NOTE: LemLib's isInMotion() is not always a reliable indicator for motion completion.
    // If it reports false while the robot is still being commanded, we'd skip the loop and fail
    // to cancel the motion on timeout, which looks like "tiny wheel movements forever".
    while (pros::millis() - start < overall_timeout_ms) {
        pros::delay(10);
        const uint32_t now = pros::millis();

        const auto pose = sbot_chassis->getPose();
        const double dx = target.x - pose.x;
        const double dy = target.y - pose.y;
        const double dist = std::sqrt(dx * dx + dy * dy);
        const double hErr = std::fabs(sbot_heading_error_deg(target_heading_deg, sbot_get_best_heading_deg()));

        if (SBOT_TRACE_POSE_WAIT_PROGRESS && (now - last_trace_ms >= SBOT_TRACE_POSE_WAIT_PERIOD_MS)) {
            const auto pose_now = pose;
            const double imu_heading = sbot_inertial_sensor ? sbot_inertial_sensor->get_heading() : 0.0;
            const double imu_rotation = sbot_inertial_sensor ? sbot_inertial_sensor->get_rotation() : 0.0;
            const double vert_in_now = sbot_vertical_tracking_wheel ? sbot_vertical_tracking_wheel->getDistanceTraveled() : 0.0;

            const double dpx = pose_now.x - last_trace_pose.x;
            const double dpy = pose_now.y - last_trace_pose.y;
            const double dp = std::sqrt(dpx * dpx + dpy * dpy);
            const double dvert = vert_in_now - last_trace_vert_in;

            printf(
                "SBOT WAIT TRACE [%s] t=%ums dist=%.2f hErr=%.2f pose(%.2f,%.2f,%.1f) dPose=%.2f vert.in=%.2f dVert=%.2f imu.h=%.1f imu.r=%.1f\n",
                label,
                static_cast<unsigned>(now - start),
                dist,
                hErr,
                pose_now.x,
                pose_now.y,
                pose_now.theta,
                dp,
                vert_in_now,
                dvert,
                imu_heading,
                imu_rotation
            );

            last_trace_ms = now;
            last_trace_pose = pose_now;
            last_trace_vert_in = vert_in_now;
        }

        if ((now - start >= min_time_ms) && (dist <= close_dist_in) && (hErr <= close_heading_deg)) {
            closed = true;
            sbot_chassis->cancelAllMotions();
            break;
        }

        // If LemLib says we're not in motion, don't spin forever waiting for a state change.
        // We still rely on the timeout to cancel any lingering command.
        if (!sbot_chassis->isInMotion() && (now - start >= min_time_ms)) {
            break;
        }
    }

    const uint32_t dur = pros::millis() - start;
    const bool timed_out = (dur >= overall_timeout_ms);
    if (timed_out) sbot_chassis->cancelAllMotions();

    // Debug proof: show how far we are from the target when we exit.
    const auto pose_end = sbot_chassis->getPose();
    const double dx_end = target.x - pose_end.x;
    const double dy_end = target.y - pose_end.y;
    const double dist_end = std::sqrt(dx_end * dx_end + dy_end * dy_end);
    const double hErr_end = std::fabs(sbot_heading_error_deg(target_heading_deg, sbot_get_best_heading_deg()));
    const bool in_motion_end = sbot_chassis->isInMotion();
    const bool close_end = (dist_end <= close_dist_in) && (hErr_end <= close_heading_deg);
    const bool ended_not_close = !close_end && !timed_out;

    if (SBOT_PRINT_WAIT_TIMES) {
        printf(
            "SBOT WAIT [%s]: %u ms%s%s%s endDist=%.2f endHErr=%.2f inMotion=%d\n",
            label,
            dur,
            (closed || close_end) ? " (close)" : "",
            timed_out ? " (timed out)" : "",
            ended_not_close ? " (ended not close)" : "",
            dist_end,
            hErr_end,
            in_motion_end ? 1 : 0
        );
    }
}

void sbot_match_turn_point_turn(
    const char* label,
    float target_x,
    float target_y,
    float target_heading_deg,
    uint32_t turn_timeout_ms,
    uint32_t drive_motion_timeout_ms,
    const lemlib::TurnToHeadingParams& turn_params,
    const lemlib::MoveToPointParams& drive_params,
    uint32_t drive_wait_timeout_ms,
    uint32_t drive_min_time_ms,
    double drive_close_dist_in,
    double drive_close_heading_deg,
    bool do_pre_turn,
    bool do_post_turn
) {
    if (!sbot_chassis) return;

    char stage_label[96];

    if (do_pre_turn) {
        std::snprintf(stage_label, sizeof(stage_label), "%s.pre_turn", label);
        sbot_chassis->turnToHeading(target_heading_deg, static_cast<int>(turn_timeout_ms), turn_params);
        sbot_wait_until_done_or_timed_out_timed(stage_label, turn_timeout_ms + 250);
    }

    std::snprintf(stage_label, sizeof(stage_label), "%s.drive", label);
    sbot_chassis->moveToPoint(target_x, target_y, static_cast<int>(drive_motion_timeout_ms), drive_params);
    if (drive_wait_timeout_ms > 0 && drive_close_dist_in > 0.0 && drive_close_heading_deg > 0.0) {
        sbot_match_wait_until_pose_close_or_timeout_timed(
            stage_label,
            drive_wait_timeout_ms,
            drive_min_time_ms,
            {target_x, target_y},
            drive_close_dist_in,
            target_heading_deg,
            drive_close_heading_deg
        );
    } else if (drive_wait_timeout_ms > 0) {
        sbot_wait_until_done_or_timed_out_timed(stage_label, drive_wait_timeout_ms);
    } else {
        sbot_wait_until_done_or_timed_out_timed(stage_label, drive_motion_timeout_ms + 250);
    }

    if (do_post_turn) {
        std::snprintf(stage_label, sizeof(stage_label), "%s.post_turn", label);
        sbot_chassis->turnToHeading(target_heading_deg, static_cast<int>(turn_timeout_ms), turn_params);
        sbot_wait_until_done_or_timed_out_timed(stage_label, turn_timeout_ms + 250);
    }
}

bool sbot_match_drive_relative(double distance_in, uint32_t timeout_ms, bool forwards) {
    if (!validateSbotLemLibInitialization()) return false;
    if (!sbot_chassis) return false;

    const auto pose = sbot_chassis->getPose();
    const double heading_rad = pose.theta * M_PI / 180.0;
    // LemLib odom convention: at theta=0°, robot faces +Y. (+X is right)
    const double dx = distance_in * std::sin(heading_rad);
    const double dy = distance_in * std::cos(heading_rad);

    // IMPORTANT:
    // - dx/dy represent the robot's *forward* direction in field coordinates.
    // - If we want to drive backwards while keeping the same heading, the target point must be
    //   behind the robot: pose - (dx,dy). (params.forwards=false)
    const double target_x = forwards ? (pose.x + dx) : (pose.x - dx);
    const double target_y = forwards ? (pose.y + dy) : (pose.y - dy);

    lemlib::MoveToPointParams params;
    params.forwards = forwards;
    params.maxSpeed = SBOT_MATCH_MAX_SPEED;
    sbot_chassis->moveToPoint(target_x, target_y, timeout_ms, params);
    sbot_wait_until_done_timed("drive_relative");
    return true;
}

bool sbot_match_drive_relative_stall_exit(
    double distance_in,
    uint32_t motion_timeout_ms,
    bool forwards,
    uint32_t stall_window_ms,
    double stall_epsilon_in,
    int maxSpeed
) {
    if (!validateSbotLemLibInitialization()) return false;
    if (!sbot_chassis) return false;

    const auto pose = sbot_chassis->getPose();
    const double heading_rad = pose.theta * M_PI / 180.0;
    const double dx = distance_in * std::sin(heading_rad);
    const double dy = distance_in * std::cos(heading_rad);

    const double target_x = forwards ? (pose.x + dx) : (pose.x - dx);
    const double target_y = forwards ? (pose.y + dy) : (pose.y - dy);

    lemlib::MoveToPointParams params;
    params.forwards = forwards;
    params.maxSpeed = maxSpeed;

    sbot_chassis->moveToPoint(target_x, target_y, motion_timeout_ms, params);
    sbot_wait_until_done_or_stalled_timed("drive_relative_stall_exit", motion_timeout_ms, stall_window_ms, stall_epsilon_in);
    return true;
}

SbotPoint sbot_match_offset_forward(const SbotPoint& p, double heading_deg, double distance_in) {
    const double heading_rad = heading_deg * M_PI / 180.0;
    // LemLib convention: 0° faces +Y.
    const double fx = std::sin(heading_rad);
    const double fy = std::cos(heading_rad);
    return {p.x + fx * distance_in, p.y + fy * distance_in};
}

SbotPoint sbot_match_pose_from_front_contact(const SbotPoint& contact, double heading_deg, double front_bumper_in) {
    return sbot_match_offset_forward(contact, heading_deg, -front_bumper_in);
}

SbotPoint sbot_match_pose_from_back_contact(const SbotPoint& contact, double heading_deg, double back_bumper_in) {
    return sbot_match_offset_forward(contact, heading_deg, back_bumper_in);
}

void sbot_match_score_mid_for(uint32_t ms, bool run_intake) {
    if (!sbot_indexer) return;

    // Mimic driver helper behavior: intake forward assists while indexer reverses.
    // For RED LEFT, we skip intake to keep balls for long goal
    if (run_intake && sbot_intake) sbot_intake->setMode(IntakeMode::COLLECT_FORWARD);
    sbot_indexer->setMode(IndexerMode::FEED_BACKWARD_MIDDLE);

    const uint32_t start = pros::millis();
    while (pros::millis() - start < ms) {
        if (run_intake && sbot_intake) sbot_intake->update();
        sbot_indexer->update();
        pros::delay(10);
    }

    sbot_indexer->setMode(IndexerMode::OFF);
    sbot_indexer->update();
    if (sbot_intake) {
        sbot_intake->setMode(IntakeMode::OFF);
        sbot_intake->update();
    }
    pros::delay(120);
}

void sbot_match_score_low_for(uint32_t ms) {
    // Match driver behavior: low-goal scoring is intake reverse only.
    if (!sbot_intake) return;

    if (sbot_indexer) {
        sbot_indexer->setMode(IndexerMode::OFF);
        sbot_indexer->update();
    }
    sbot_intake->setMode(IntakeMode::REVERSE_LOW_GOAL);

    const uint32_t start = pros::millis();
    while (pros::millis() - start < ms) {
        sbot_intake->update();
        pros::delay(10);
    }

    sbot_intake->setMode(IntakeMode::OFF);
    sbot_intake->update();
    pros::delay(120);
}

void sbot_match_score_top_for(uint32_t ms) {
    if (!sbot_indexer) return;
    // Open flap for scoring
    if (sbot_goal_flap) sbot_goal_flap->open();
    if (sbot_intake) sbot_intake->setMode(IntakeMode::COLLECT_FORWARD);
    sbot_indexer->setMode(IndexerMode::FEED_FORWARD);

    const uint32_t start = pros::millis();
    while (pros::millis() - start < ms) {
        if (sbot_intake) sbot_intake->update();
        sbot_indexer->update();
        pros::delay(10);
    }

    sbot_safe_stop_mechanisms();
}
