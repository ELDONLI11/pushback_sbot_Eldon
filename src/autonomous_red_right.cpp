/**
 * autonomous_red_right.cpp
 *
 * Red Right tuning derived from Red Left (Jerry mirrored).
 */
#include "autonomous_match_awp.h"

SbotAwpHalfTuning sbot_awp_half_red_right_tuning() {
    // Start from the same defaults as RL (timeouts, scoring times, etc).
    auto t = sbot_awp_half_red_left_tuning();

    // Replace Jerry-derived points with their mirrored Jerry counterparts.
    // NOTE: these calls depend on sbot_jerry_start_* having been set to the RR start.
    t.cluster1 = sbot_from_jerry(-21.0, -21.0);

    // Retreat point: (-48, 48) -> (-48, -48)
    t.use_post_score_retreat_point = true;
    t.post_score_retreat_point = sbot_from_jerry(-48.0, -48.0);

    // Center Goal contacts mirrored.
    if (t.use_low_goal_contact) t.low_goal_contact = sbot_from_jerry(-9.0, -9.0);
    if (t.use_mid_goal_contact) t.mid_goal_contact = sbot_from_jerry(-9.0, 9.0);

    // Tube contact mirrored: (-73, 48) -> (-73, -48)
    if (t.use_tube1_contact) t.tube1_contact = sbot_from_jerry(-73.0, -48.0);

    // Solo AWP: Mirror cluster2 and mid_goal_solo
    t.cluster2 = sbot_from_jerry(24.0, -24.0);  // (24, 24) -> (24, -24)
    if (t.use_mid_goal_solo_contact) {
        t.mid_goal_solo_contact = sbot_from_jerry(9.0, -9.0);  // (9, 9) -> (9, -9)
        const double mid_solo_heading = sbot_mirror_heading(-135.0);  // -135° -> 135°
        t.mid_goal_solo_approach = sbot_pose_from_back_contact(t.mid_goal_solo_contact, mid_solo_heading, SBOT_BACK_BUMPER_IN);
        t.mid_goal_solo_heading_deg = mid_solo_heading;
    }

    // Mirror remaining internal-only geometry across the centerline.
    t.tube1 = sbot_mirror_point_x(t.tube1);
    t.low_goal_approach = sbot_mirror_point_x(t.low_goal_approach);
    t.mid_goal_approach = sbot_mirror_point_x(t.mid_goal_approach);
    t.tube2 = sbot_mirror_point_x(t.tube2);
    t.tube2_pulloff = sbot_mirror_point_x(t.tube2_pulloff);

    // Mirror headings.
    t.low_goal_heading_deg = sbot_mirror_heading(t.low_goal_heading_deg);
    t.mid_goal_heading_deg = sbot_mirror_heading(t.mid_goal_heading_deg);
    t.high_goal_heading_deg = sbot_mirror_heading(t.high_goal_heading_deg);
    t.tube_face_heading_deg = sbot_mirror_heading(t.tube_face_heading_deg);

    // Override Stage 2: RED RIGHT should use Center Goal – Middle (back-score).
    const double center_middle_dx = 0.0;
    const double center_middle_dy = 13.0;
    t.mid_goal_approach = {t.cluster1.x + center_middle_dx, t.cluster1.y + center_middle_dy};
    t.mid_goal_heading_deg = -135;

    // Keep Center-Lower distinct (not used in this path).
    const double center_lower_dx = 13.0;
    const double center_lower_dy = 13.0;
    t.low_goal_approach = {t.cluster1.x + center_lower_dx, t.cluster1.y + center_lower_dy};
    t.low_goal_heading_deg = -45;

    return t;
}
