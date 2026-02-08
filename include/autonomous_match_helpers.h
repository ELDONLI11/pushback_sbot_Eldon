/**
 * autonomous_match_helpers.h
 *
 * Match-specific helper functions with tuned behavior/signatures.
 */
#ifndef AUTONOMOUS_MATCH_HELPERS_H
#define AUTONOMOUS_MATCH_HELPERS_H

#include <cstdint>

#include "autonomous_constants.h"
#include "autonomous_infrastructure.h"
#include "lemlib/api.hpp"

void sbot_match_wait_until_pose_close_or_timeout_timed(
    const char* label,
    uint32_t overall_timeout_ms,
    uint32_t min_time_ms,
    const SbotPoint& target,
    double close_dist_in,
    double target_heading_deg,
    double close_heading_deg
);

void sbot_match_turn_point_turn(
    const char* label,
    float target_x,
    float target_y,
    float target_heading_deg,
    uint32_t turn_timeout_ms,
    uint32_t drive_motion_timeout_ms,
    const lemlib::TurnToHeadingParams& turn_params,
    const lemlib::MoveToPointParams& drive_params,
    uint32_t drive_wait_timeout_ms = 0,
    uint32_t drive_min_time_ms = 0,
    double drive_close_dist_in = 0.0,
    double drive_close_heading_deg = 0.0,
    bool do_pre_turn = true,
    bool do_post_turn = true
);

bool sbot_match_drive_relative(double distance_in, uint32_t timeout_ms, bool forwards = true);

bool sbot_match_drive_relative_stall_exit(
    double distance_in,
    uint32_t motion_timeout_ms,
    bool forwards,
    uint32_t stall_window_ms = 300,
    double stall_epsilon_in = 0.35,
    int maxSpeed = SBOT_MATCH_MAX_SPEED
);

SbotPoint sbot_match_offset_forward(const SbotPoint& p, double heading_deg, double distance_in);
SbotPoint sbot_match_pose_from_front_contact(const SbotPoint& contact, double heading_deg, double front_bumper_in);
SbotPoint sbot_match_pose_from_back_contact(const SbotPoint& contact, double heading_deg, double back_bumper_in);

void sbot_match_score_mid_for(uint32_t ms, bool run_intake = true);
void sbot_match_score_low_for(uint32_t ms);
void sbot_match_score_top_for(uint32_t ms);

#endif // AUTONOMOUS_MATCH_HELPERS_H
