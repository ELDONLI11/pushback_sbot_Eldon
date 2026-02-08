/**
 * autonomous_sbot.h - Autonomous selector and routines for sbot.
 */

#ifndef _SBOT_AUTONOMOUS_SBOT_H_
#define _SBOT_AUTONOMOUS_SBOT_H_

#include "api.h"
#include "config_sbot.h"

// NOTE: LemLib will be configured in a separate file; here we just
// declare usage and provide a selector with the requested modes.

enum class SbotAutoMode {
    DISABLED = 0,
    LEFT,                       // 1 (was RED_LEFT)
    RIGHT,                      // 2 (was RED_RIGHT)
    RED_LEFT_SOLO_AWP,          // 3 (was 5)
    BLUE_LEFT_SOLO_AWP,         // 4 (was 7)
    BLUE_RIGHT_SOLO_AWP,        // 5 (was 8)
    SKILLS,                     // 6 (was 9)
    TEST_SWEEP_TO_LOW_GOAL,     // 7 (was 10)
    TEST_DRIVE,                 // 8 (was 11)
    TEST_TURN,                  // 9 (was 12)
    TEST_INTAKE,                // 10 (was 13)
    TEST_INDEXER,               // 11 (was 14)
    TEST_DRIVE_SHORT,           // 12 (was 15)
    TEST_LOW_GOAL_CUSTOM_START, // 13 (was 16)
    TEST_JERRY_POSE_MONITOR,    // 14 (was 17)
    TEST_FOLLOW_JERRY_PATH,     // 15 (was 18)
    TEST_POSE_FINDER_X0_LINE_90,// 16 (was 19)
    TEST_DRIVE_FORWARD_2IN      // 17 (was 20)
};

class SbotAutoSelector {
public:
    SbotAutoSelector();

    bool update();              // handle input and refresh display; true when confirmed
    SbotAutoMode getMode() const { return selected_mode; }
    bool isConfirmed() const { return mode_confirmed; }
    void forceDisplayRefresh();  // Force screen update even if state unchanged (prevents blank screen)

private:
    SbotAutoMode selected_mode;
    int selector_position;
    bool mode_confirmed;
    int last_confirmed_position;  // Remember last confirmed selection across disabled periods

    void displayOptions();
    void handleInput();
};

class SbotAutonomousSystem {
public:
    SbotAutonomousSystem();

    void initialize();
    void updateSelector();
    void run(); // call from autonomous()
    void runRight(); // Exposed for emergency fallback
    void runLeft();  // Exposed for direct autonomous call

    SbotAutoSelector& getSelector() { return selector; }

private:
    SbotAutoSelector selector;

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
