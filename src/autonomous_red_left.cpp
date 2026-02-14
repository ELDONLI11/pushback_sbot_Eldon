/**
 * autonomous_red_left.cpp
 *
 * Red Left canonical tuning for AWP half-field match autonomous.
 */
#include "autonomous_match_awp.h"

#include <algorithm>

SbotAwpHalfTuning sbot_awp_half_red_left_tuning() {
    SbotAwpHalfTuning t;

    // NOTE: These are conservative first-pass guesses.
    // Tune on a real field by logging pose prints and adjusting the points.
    // Conventions: +Y forward into field, +X to robot-right at 0°.

    // No obstacle: drive directly from start to the first cluster.
    t.clear_barrier_in = 0.0;

    // Cluster (RED LEFT) from Jerry field points.
    t.cluster1 = sbot_from_jerry(RedLeft::CLUSTER1_JERRY_X, RedLeft::CLUSTER1_JERRY_Y);
    t.cluster_collect_ms = 150;

    // Center Goal – Lower (RED LEFT / BLUE RIGHT): from the cluster,
    // user-measured direction is forward-right about ~0.75 tile diagonally.
    // IMPORTANT: keep the robot on the same line from cluster -> goal for reliable scoring.
    const double center_lower_dx = 18.0;
    const double center_lower_dy = 18.0;
    const SbotPoint center_lower_approach = {t.cluster1.x + center_lower_dx, t.cluster1.y + center_lower_dy};

    // Center Goal – Middle: separate tuning (primarily for RED RIGHT / BLUE LEFT).
    const double center_middle_dx = 18;
    const double center_middle_dy = 18.0;
    const SbotPoint center_middle_approach = {t.cluster1.x - center_middle_dx, t.cluster1.y + center_middle_dy};

    // Center Goal – Lower approach (front-score).
    t.low_goal_approach = center_lower_approach;
    t.low_goal_heading_deg = -45;
    // Lower-goal scoring: add extra time to ensure balls fully clear.
    t.low_goal_score_ms = SBOT_LOW_GOAL_SCORE_TIME_MS + 750;
    // Use a measured front-bumper contact point for the Center Goal.
    t.use_low_goal_contact = true;
    t.low_goal_contact = sbot_from_jerry(RedLeft::CENTER_LOW_GOAL_JERRY_X, RedLeft::CENTER_LOW_GOAL_JERRY_Y);

    // Center Goal – Middle (back-score).
    t.mid_goal_approach = center_middle_approach;
    t.mid_goal_heading_deg = -135;
    t.mid_goal_score_ms = std::max<uint32_t>(SBOT_MID_GOAL_SCORE_TIME_MS, SBOT_MIN_SCORE_TIME_MS);
    // Measured back-bumper contact point for Center Goal – Middle.
    t.use_mid_goal_contact = true;
    t.mid_goal_contact = sbot_from_jerry(RedLeft::CENTER_MID_GOAL_JERRY_X, RedLeft::CENTER_MID_GOAL_JERRY_Y);

    // Stage 5: Long Goal scoring
    t.high_goal_heading_deg = 180;
    t.high_goal_score_ms = SBOT_MIN_SCORE_TIME_MS;
    // Back into long goal end from loader: drive to Jerry (-24, 48) then back in slightly.
    t.high_goal_back_in_from_tube_in = 24.0;

    // Force retreat to a measured absolute point (start-relative frame).
    t.use_post_score_retreat_point = true;
    t.post_score_retreat_point = sbot_from_jerry(RedLeft::RETREAT_POINT_JERRY_X, RedLeft::RETREAT_POINT_JERRY_Y);

    // After retreat, turn to face alliance wall where the loader is.
    t.tube_face_heading_deg = 180;

    // Your measured loader protrusion when deployed.
    t.loader_down_extra_front_in = 6.0;

    // Loader (tube) pose points (fallback when not using contact points).
    t.tube1 = {-33, -11.0};
    t.tube_pull_ms = 250;

    // Loader contact point (field feature, Jerry coords).
    // This is where the FRONT of the robot/loader should contact the match loader.
    t.use_tube1_contact = true;
    t.tube1_contact = sbot_from_jerry(RedLeft::LOADER_CONTACT_JERRY_X, RedLeft::LOADER_CONTACT_JERRY_Y);
    t.tube_extra_seat_in = 4.0;

    // Solo AWP Stage 6: Second cluster collection
    t.cluster2 = sbot_from_jerry(RedLeft::CLUSTER2_JERRY_X, RedLeft::CLUSTER2_JERRY_Y);
    t.cluster2_collect_ms = 150;

    // Solo AWP Stage 7: Center Middle Goal (back-score from opposite side)
    t.use_mid_goal_solo_contact = true;
    t.mid_goal_solo_contact = sbot_from_jerry(RedLeft::SOLO_MID_GOAL_JERRY_X, RedLeft::SOLO_MID_GOAL_JERRY_Y);
    // Calculate pose target from contact point using back bumper
    const double mid_solo_heading = -135.0;
    t.mid_goal_solo_approach = sbot_pose_from_back_contact(t.mid_goal_solo_contact, mid_solo_heading, SBOT_BACK_BUMPER_IN);
    t.mid_goal_solo_heading_deg = mid_solo_heading;

    // Keep old tube2 for now (unused in new solo design)
    t.tube2 = {54, -24};
    t.tube2_pulloff = {-18, -18};

    // Timeouts: keep tight so we don't burn match time if something is slightly off.
    // We rely on pose-close exit thresholds to end motions quickly once we're in position.
    t.drive_timeout_ms = 5500;
    t.turn_timeout_ms = 1300;

    return t;
}
