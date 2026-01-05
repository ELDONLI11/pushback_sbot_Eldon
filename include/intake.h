/**
 * intake.h - Intake and helper intake control for sbot.
 */

#ifndef _SBOT_INTAKE_H_
#define _SBOT_INTAKE_H_

#include "api.h"
#include "config_sbot.h"

enum class IntakeMode {
    OFF = 0,
    COLLECT_FORWARD,
    REVERSE_LOW_GOAL
};

class SbotIntake {
public:
    SbotIntake();

    void setMode(IntakeMode mode);
    IntakeMode getMode() const { return mode; }

    void update(); // call periodically to apply mode

private:
    pros::Motor main_intake;
    pros::Motor helper_intake;
    IntakeMode mode;
};

#endif // _SBOT_INTAKE_H_
