/**
 * autonomous_sbot.h - Autonomous selector and routines for sbot.
 */

#ifndef _SBOT_AUTONOMOUS_SBOT_H_
#define _SBOT_AUTONOMOUS_SBOT_H_

#include "api.h"
#include "config_sbot.h"

class SbotAutonomousSystem {
public:
    SbotAutonomousSystem();

    void initialize();
    void runRight(); // Exposed for emergency fallback
    void runLeft();  // Exposed for direct autonomous call

private:
    // Simple stubs for now – can be filled with LemLib paths later
    void runSkills();

    void runTestSweepToLowGoal();

    void runTestDrive();
    void runTestDriveShort();
    void runTestDriveForward2In();
    void runTestLowGoalCustomStart();
    void runTestTurn();
    void runTestIntake();
    void runTestIndexer();
    void runTestJerryPoseMonitor();
    void runTestFollowJerryPath();
    void runTestPoseFinderX0Line90();
};

#endif // _SBOT_AUTONOMOUS_SBOT_H_
