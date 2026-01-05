/**
 * drivetrain.cpp - 6-motor tank drive implementation for sbot.
 */

#include "drivetrain.h"

SbotDrivetrain::SbotDrivetrain()
        : left_front(-SBOT_LEFT_FRONT_MOTOR_PORT, SBOT_DRIVE_GEARSET),
            left_middle(-SBOT_LEFT_MIDDLE_MOTOR_PORT, SBOT_DRIVE_GEARSET),
            left_back(-SBOT_LEFT_BACK_MOTOR_PORT, SBOT_DRIVE_GEARSET),
            right_front(SBOT_RIGHT_FRONT_MOTOR_PORT, SBOT_DRIVE_GEARSET),
            right_middle(SBOT_RIGHT_MIDDLE_MOTOR_PORT, SBOT_DRIVE_GEARSET),
            right_back(SBOT_RIGHT_BACK_MOTOR_PORT, SBOT_DRIVE_GEARSET) {

    left_front.set_brake_mode(SBOT_DRIVE_BRAKE_MODE);
    left_middle.set_brake_mode(SBOT_DRIVE_BRAKE_MODE);
    left_back.set_brake_mode(SBOT_DRIVE_BRAKE_MODE);
    right_front.set_brake_mode(SBOT_DRIVE_BRAKE_MODE);
    right_middle.set_brake_mode(SBOT_DRIVE_BRAKE_MODE);
    right_back.set_brake_mode(SBOT_DRIVE_BRAKE_MODE);
}

int SbotDrivetrain::applyDeadzone(int value) const {
    if (std::abs(value) < SBOT_JOYSTICK_DEADZONE) return 0;
    return value;
}

void SbotDrivetrain::arcadeTankControl(pros::Controller& master) {
    int left_input = master.get_analog(SBOT_TANK_LEFT_STICK);
    int right_input = master.get_analog(SBOT_TANK_RIGHT_STICK);

    left_input = applyDeadzone(left_input);
    right_input = applyDeadzone(right_input);

    // Scale by sensitivity and send as voltage percentage (-127..127)
    left_input = static_cast<int>(left_input * SBOT_TANK_SENSITIVITY);
    right_input = static_cast<int>(right_input * SBOT_TANK_SENSITIVITY);

    left_front.move(left_input);
    left_middle.move(left_input);
    left_back.move(left_input);

    right_front.move(right_input);
    right_middle.move(right_input);
    right_back.move(right_input);
}

void SbotDrivetrain::setBrakeMode(pros::v5::MotorBrake mode) {
    left_front.set_brake_mode(mode);
    left_middle.set_brake_mode(mode);
    left_back.set_brake_mode(mode);
    right_front.set_brake_mode(mode);
    right_middle.set_brake_mode(mode);
    right_back.set_brake_mode(mode);
}

void SbotDrivetrain::stop() {
    left_front.move(0);
    left_middle.move(0);
    left_back.move(0);
    right_front.move(0);
    right_middle.move(0);
    right_back.move(0);
}
