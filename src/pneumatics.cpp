/**
 * pneumatics.cpp - Pneumatic actuators for sbot.
 */

#include "pneumatics.h"

BatchLoaderPiston::BatchLoaderPiston()
    : solenoid(SBOT_BATCH_LOADER_PISTON_PORT, SBOT_PISTON_RETRACTED_STATE),
      state(false) {}

void BatchLoaderPiston::extend() {
    state = true;
    solenoid.set_value(SBOT_PISTON_EXTENDED_STATE);
    printf("SBOT PNEU: batch loader EXTEND (ADI %c -> %d)\n", SBOT_BATCH_LOADER_PISTON_PORT, SBOT_PISTON_EXTENDED_STATE ? 1 : 0);
}

void BatchLoaderPiston::retract() {
    state = false;
    solenoid.set_value(SBOT_PISTON_RETRACTED_STATE);
    printf("SBOT PNEU: batch loader RETRACT (ADI %c -> %d)\n", SBOT_BATCH_LOADER_PISTON_PORT, SBOT_PISTON_RETRACTED_STATE ? 1 : 0);
}

GoalFlapPiston::GoalFlapPiston()
    : solenoid(SBOT_GOAL_FLAP_PISTON_PORT, SBOT_PISTON_RETRACTED_STATE),
      state(false) {}

void GoalFlapPiston::open() {
    state = true;
    solenoid.set_value(SBOT_PISTON_EXTENDED_STATE);
}

void GoalFlapPiston::close() {
    state = false;
    solenoid.set_value(SBOT_PISTON_RETRACTED_STATE);
}
