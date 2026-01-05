/**
 * pneumatics.h - Batch loader and goal flap pistons for sbot.
 */

#ifndef _SBOT_PNEUMATICS_H_
#define _SBOT_PNEUMATICS_H_

#include "api.h"
#include "config_sbot.h"

class BatchLoaderPiston {
public:
    BatchLoaderPiston();

    void extend();
    void retract();
    bool isExtended() const { return state; }

private:
    pros::adi::DigitalOut solenoid;
    bool state; // true = extended
};

class GoalFlapPiston {
public:
    GoalFlapPiston();

    void open();   // allow top-goal scoring
    void close();  // block balls (default)
    bool isOpen() const { return state; }

private:
    pros::adi::DigitalOut solenoid;
    bool state; // true = open
};

#endif // _SBOT_PNEUMATICS_H_
