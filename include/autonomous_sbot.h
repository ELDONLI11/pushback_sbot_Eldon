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
    RED_LEFT,
    RED_RIGHT,
    BLUE_LEFT,
    BLUE_RIGHT,
    SKILLS,
    TEST_DRIVE,
    TEST_TURN,
    TEST_INTAKE,
    TEST_INDEXER
};

class SbotAutoSelector {
public:
    SbotAutoSelector();

    bool update();              // handle input and refresh display; true when confirmed
    SbotAutoMode getMode() const { return selected_mode; }
    bool isConfirmed() const { return mode_confirmed; }

private:
    SbotAutoMode selected_mode;
    int selector_position;
    bool mode_confirmed;

    void displayOptions();
    void handleInput();
};

class SbotAutonomousSystem {
public:
    SbotAutonomousSystem();

    void initialize();
    void updateSelector();
    void run(); // call from autonomous()

    SbotAutoSelector& getSelector() { return selector; }

private:
    SbotAutoSelector selector;

    // Simple stubs for now – can be filled with LemLib paths later
    void runRedLeft();
    void runRedRight();
    void runBlueLeft();
    void runBlueRight();
    void runSkills();

    void runTestDrive();
    void runTestTurn();
    void runTestIntake();
    void runTestIndexer();
};

#endif // _SBOT_AUTONOMOUS_SBOT_H_
