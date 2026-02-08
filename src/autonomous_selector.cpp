/**
 * autonomous_selector.cpp
 *
 * RoboDash-based autonomous selector.
 */
#include "autonomous_sbot.h"
#include "robodash_selector.h"

#include <cstdio>

static const char* sbot_mode_name(int idx) {
    static const char* mode_names[] = {
        "DISABLED",     // 0
        "Red Left",     // 1
        "Red Right",    // 2
        "Blue Left",    // 3
        "Blue Right",   // 4
        "Red Left (Solo AWP)",   // 5
        "Red Right (Solo AWP)",  // 6
        "Blue Left (Solo AWP)",  // 7
        "Blue Right (Solo AWP)", // 8
        "Skills",                // 9
        "Test: Sweep->Low Goal", // 10
        "Test: Drive",           // 11
        "Test: Turn",            // 12
        "Test: Intake",          // 13
        "Test: Indexer",         // 14
        "Test: Drive Short",     // 15
        "Test: LowGoal (custom start)", // 16
        "Test: Pose Monitor (x,y,th)",  // 17
        "Test: Follow Path (LemLib follow)", // 18
        "Test: Pose Finder (x0 line, 90deg)", // 19
        "Test: Drive Forward 2in" // 20
    };

    if (idx < 0 || idx > 20) return "<invalid>";
    return mode_names[idx];
}

SbotAutoSelector::SbotAutoSelector()
    : selected_mode(SbotAutoMode::DISABLED),
      selector_position(0),
      mode_confirmed(false),
      last_confirmed_position(0) {}

bool SbotAutoSelector::update() {
    int idx = selector_position;
    bool confirmed = mode_confirmed;
    if (sbot_robodash_get_selection(&idx, &confirmed)) {
        selector_position = idx;
        if (confirmed) {
            selected_mode = static_cast<SbotAutoMode>(selector_position);
            mode_confirmed = true;
            last_confirmed_position = selector_position;
            printf("SBOT AUTO: READY %d (%s) [RoboDash]\n", selector_position, sbot_mode_name(selector_position));
        } else {
            mode_confirmed = false;
            printf("SBOT AUTO: select %d (%s) [RoboDash]\n", selector_position, sbot_mode_name(selector_position));
        }
        sbot_robodash_update_selector(selector_position, sbot_mode_name(selector_position), mode_confirmed);
    }
    return mode_confirmed;
}

void SbotAutoSelector::forceDisplayRefresh() {
    sbot_robodash_update_selector(selector_position, sbot_mode_name(selector_position), mode_confirmed);
}
