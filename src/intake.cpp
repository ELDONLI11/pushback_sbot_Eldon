/**
 * intake.cpp - Intake implementation for sbot.
 */

#include "intake.h"

SbotIntake::SbotIntake()
    : main_intake((SBOT_INTAKE_MAIN_MOTOR_REVERSED ? -SBOT_INTAKE_MAIN_MOTOR_PORT : SBOT_INTAKE_MAIN_MOTOR_PORT),
                    pros::v5::MotorGears::green,
                    pros::v5::MotorUnits::degrees),
        helper_intake((SBOT_INTAKE_HELPER_MOTOR_REVERSED ? -SBOT_INTAKE_HELPER_MOTOR_PORT : SBOT_INTAKE_HELPER_MOTOR_PORT),
                    pros::v5::MotorGears::green,
                    pros::v5::MotorUnits::degrees),
            mode(IntakeMode::OFF) {}

void SbotIntake::setMode(IntakeMode newMode) {
    mode = newMode;
}

void SbotIntake::update() {
    int main_speed = 0;
    int helper_speed = 0;

    switch (mode) {
        case IntakeMode::COLLECT_FORWARD:
            main_speed = SBOT_INTAKE_FORWARD_SPEED;
            helper_speed = SBOT_INTAKE_FORWARD_SPEED;
            break;
        case IntakeMode::REVERSE_LOW_GOAL:
            // For low-goal scoring, run the main intake slower to reduce jam/whip,
            // while keeping the helper assisting the outflow.
            main_speed = static_cast<int>(SBOT_INTAKE_REVERSE_LOW_GOAL * 0.25);
            helper_speed = SBOT_INTAKE_REVERSE_LOW_GOAL;
            break;
        case IntakeMode::OFF:
        default:
            main_speed = 0;
            helper_speed = 0;
            break;
    }

    main_intake.move_velocity(main_speed);
    helper_intake.move_velocity(helper_speed);
}
