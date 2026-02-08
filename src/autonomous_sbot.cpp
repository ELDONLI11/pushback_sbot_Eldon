/**
 * autonomous_sbot.cpp - Autonomous system dispatch for sbot.
 */

#include "autonomous_sbot.h"

#include "autonomous_match_awp.h"
#include "autonomous_skills.h"
#include "lemlib_config_sbot.h"
#include "robodash_selector.h"

#include <cstdio>

SbotAutonomousSystem::SbotAutonomousSystem() {}

void SbotAutonomousSystem::initialize() {
    // Initialize LemLib for sbot (safe to call once)
    initializeSbotLemLib();
    sbot_robodash_init();
}

void SbotAutonomousSystem::runLeft() {
    printf("MARKER04\n");
    printf("\n=== SbotAutonomousSystem::runLeft() CALLED ===\n");
    printf("SBOT: Executing RED LEFT autonomous\n");
    fflush(stdout);
    sbot_run_match_auto(SbotAutoSide::LEFT, SbotAutoAlliance::RED, false);
    printf("=== SbotAutonomousSystem::runLeft() COMPLETE ===\n\n");
    fflush(stdout);
}

void SbotAutonomousSystem::runRight() {
    sbot_run_match_auto(SbotAutoSide::RIGHT, SbotAutoAlliance::RED, false);
}

void SbotAutonomousSystem::runSkills() {
    sbot_run_skills_auto();
}
