/**
 * autonomous_match_awp.h
 *
 * Shared match autonomous helpers and tuning for AWP half-field routines.
 * Red-Left and Red-Right tunings are provided by separate compilation units.
 */
#ifndef AUTONOMOUS_MATCH_AWP_H
#define AUTONOMOUS_MATCH_AWP_H

#include <cstdint>

#include "autonomous_infrastructure.h"
#include "autonomous_constants.h"
#include "config_sbot.h"

struct SbotAwpHalfTuning {
    // All points are defined for RED LEFT canonical frame.
    // They are transformed for other alliances via mirror/rotation.
    // Frame is start-relative as described in sbot_set_match_start_pose().

    // Stage 0: ensure we are not touching the park zone barrier
    double clear_barrier_in;

    // Stage 1: collect the nearby block cluster
    SbotPoint cluster1;                     // Target cluster position
    uint32_t cluster_collect_ms;            // Dwell time at cluster

    // Stage 2: Center Goal scoring
    // - (RED LEFT, BLUE RIGHT): Center Goal – Lower (front score)
    // - (RED RIGHT, BLUE LEFT): Center Goal – Middle (back score)
    SbotPoint low_goal_approach;            // Lower goal pose target
    double low_goal_heading_deg;            // Lower goal heading
    uint32_t low_goal_score_ms;             // Lower goal score duration
    bool use_low_goal_contact;              // Use contact point conversion
    SbotPoint low_goal_contact;             // Lower goal bumper contact point

    SbotPoint mid_goal_approach;            // Middle goal pose target
    double mid_goal_heading_deg;            // Middle goal heading
    uint32_t mid_goal_score_ms;             // Middle goal score duration
    bool use_mid_goal_contact;              // Use contact point conversion
    SbotPoint mid_goal_contact;             // Middle goal bumper contact point

    // Stage 3: Retreat after first score
    bool use_post_score_retreat_point;      // Use absolute retreat point
    SbotPoint post_score_retreat_point;     // Retreat endpoint (absolute pose)
    double tube_face_heading_deg;           // Heading to face loader

    // Stage 4: Loader pull
    double loader_down_extra_front_in;      // Extra loader protrusion when deployed
    SbotPoint tube1;                        // Loader pose target (fallback)
    uint32_t tube_pull_ms;                  // Loader pull duration
    bool use_tube1_contact;                 // Use contact point conversion
    SbotPoint tube1_contact;                // Loader bumper contact point
    double tube_extra_seat_in;              // Extra distance to push into loader after contact

    // Stage 5: Long Goal scoring
    double high_goal_heading_deg;           // Long goal heading
    uint32_t high_goal_score_ms;            // Long goal score duration
    double high_goal_back_in_from_tube_in;  // Distance to back into goal from loader

    // Solo AWP: second loader pull
    SbotPoint tube2;                        // Second loader position (solo only)
    SbotPoint tube2_pulloff;                // Pulloff after second load (solo only)

    // Solo AWP: second cluster collection
    SbotPoint cluster2;                     // Second cluster position (solo AWP)
    uint32_t cluster2_collect_ms;           // Dwell time at second cluster

    // Solo AWP: second goal scoring (Center Middle from opposite side)
    SbotPoint mid_goal_solo_approach;       // Middle goal approach for solo (from cluster 2)
    double mid_goal_solo_heading_deg;       // Heading for solo middle goal (45° for back-score)
    bool use_mid_goal_solo_contact;         // Use contact point for solo middle
    SbotPoint mid_goal_solo_contact;        // Middle goal contact point (solo)

    // Timeouts
    uint32_t drive_timeout_ms;
    uint32_t turn_timeout_ms;
};

SbotAwpHalfTuning sbot_awp_half_red_left_tuning();
SbotAwpHalfTuning sbot_awp_half_red_right_tuning();

void sbot_run_match_auto(
    SbotAutoSide side,
    SbotAutoAlliance alliance,
    bool solo_awp,
    bool start_from_cluster_sweep = false,
    bool stop_after_stage2 = false,
    bool stage2_skip_pre_turn = false
);

#endif // AUTONOMOUS_MATCH_AWP_H
