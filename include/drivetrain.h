/**
 * drivetrain.h - 6-motor tank drive for sbot.
 */

#ifndef _SBOT_DRIVETRAIN_H_
#define _SBOT_DRIVETRAIN_H_

#include "api.h"
#include "config_sbot.h"

class SbotDrivetrain {
public:
    SbotDrivetrain();

    void arcadeTankControl(pros::Controller& master);
    void setBrakeMode(pros::v5::MotorBrake mode);
    void stop();

private:
    pros::Motor left_front;
    pros::Motor left_middle;
    pros::Motor left_back;
    pros::Motor right_front;
    pros::Motor right_middle;
    pros::Motor right_back;

    int applyDeadzone(int value) const;
};

#endif // _SBOT_DRIVETRAIN_H_
