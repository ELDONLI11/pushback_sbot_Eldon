/**
 * robodash_selector.cpp
 *
 * RoboDash integration for autonomous selector.
 */
#include "robodash_selector.h"

#include "autonomous_match_awp.h"
#include "autonomous_skills.h"

namespace {
int g_sel_idx = 0;
bool g_sel_confirmed = false;
bool g_has_selection = false;
}

rd::Selector selector({
    {"Disabled", [] {
        sbot_robodash_set_selection(0, false);
    }},
    {"Red Left", [] {
        sbot_robodash_set_selection(1, true);
        sbot_run_match_auto(SbotAutoSide::LEFT, SbotAutoAlliance::RED, false);
    }},
    {"Red Right", [] {
        sbot_robodash_set_selection(2, true);
        sbot_run_match_auto(SbotAutoSide::RIGHT, SbotAutoAlliance::RED, false);
    }},
    {"Blue Left", [] {
        sbot_robodash_set_selection(3, true);
        sbot_run_match_auto(SbotAutoSide::LEFT, SbotAutoAlliance::BLUE, false);
    }},
    {"Blue Right", [] {
        sbot_robodash_set_selection(4, true);
        sbot_run_match_auto(SbotAutoSide::RIGHT, SbotAutoAlliance::BLUE, false);
    }},
    {"Red Left (Solo AWP)", [] {
        sbot_robodash_set_selection(5, true);
        sbot_run_match_auto(SbotAutoSide::LEFT, SbotAutoAlliance::RED, true);
    }},
    {"Red Right (Solo AWP)", [] {
        sbot_robodash_set_selection(6, true);
        sbot_run_match_auto(SbotAutoSide::RIGHT, SbotAutoAlliance::RED, true);
    }},
    {"Blue Left (Solo AWP)", [] {
        sbot_robodash_set_selection(7, true);
        sbot_run_match_auto(SbotAutoSide::LEFT, SbotAutoAlliance::BLUE, true);
    }},
    {"Blue Right (Solo AWP)", [] {
        sbot_robodash_set_selection(8, true);
        sbot_run_match_auto(SbotAutoSide::RIGHT, SbotAutoAlliance::BLUE, true);
    }},
    {"Skills", [] {
        sbot_robodash_set_selection(9, true);
        sbot_run_skills_auto();
    }},
});

void sbot_robodash_init() {}

void sbot_robodash_update_selector(int /*idx*/, const char* /*name*/, bool /*confirmed*/) {}

bool sbot_robodash_get_selection(int* idx, bool* confirmed) {
    if (!g_has_selection || !idx || !confirmed) return false;
    *idx = g_sel_idx;
    *confirmed = g_sel_confirmed;
    return true;
}

void sbot_robodash_set_selection(int idx, bool confirmed) {
    g_sel_idx = idx;
    g_sel_confirmed = confirmed;
    g_has_selection = true;
}
