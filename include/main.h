/**
 * main.h - Entry point header for the sbot robot project.
 *
 * This project is independent of the pushback codebase and is intended
 * for the "yes bot" robot with 6-motor drivetrain, intake, helper intake,
 * indexer, pneumatics, inertial sensor, odometry wheel, and color sensor.
 */

#ifndef _SBOT_MAIN_H_
#define _SBOT_MAIN_H_

#include "api.h"

// Prototypes for the PROS competition callbacks.
// IMPORTANT: these must have C linkage so the PROS kernel can find/call them.
#ifdef __cplusplus
extern "C" {
#endif

void initialize(void);
void disabled(void);
void competition_initialize(void);
void autonomous(void);
void opcontrol(void);

#ifdef __cplusplus
}
#endif

#endif // _SBOT_MAIN_H_
