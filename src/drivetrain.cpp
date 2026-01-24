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

int SbotDrivetrain::applyCurve(int value) const {
    // Apply squared curve while preserving sign (optional)
    // This gives fine control at low speeds, aggressive acceleration at high speeds
    if (value == 0) return 0;
    
    // If squared curve disabled, return linear response
    if (!SBOT_USE_SQUARED_CURVE) {
        return value;
    }
    
    // Normalize to -1.0 to 1.0, square it, then scale back to -127 to 127
    double normalized = value / 127.0;
    double curved = normalized * std::abs(normalized); // Square while keeping sign
    
    // Blend between linear and squared based on scaling factor
    // scaling=1.0 -> full squared, scaling=0.0 -> linear
    double blended = normalized + (curved - normalized) * SBOT_CURVE_SCALING;
    
    return static_cast<int>(blended * 127.0);
}

int SbotDrivetrain::applySlewRate(int current, int target, int max_change) const {
    // Adaptive slew rate limiting with direction reversal protection
    
    int delta = target - current;
    
    // If already at target, no change needed
    if (std::abs(delta) <= 1) {
        return target;
    }
    
    // Check if this is a direction reversal (sign change)
    bool is_reversing = (current > SBOT_REVERSAL_DEADBAND && target < -SBOT_REVERSAL_DEADBAND) ||
                        (current < -SBOT_REVERSAL_DEADBAND && target > SBOT_REVERSAL_DEADBAND);
    
    if (is_reversing && SBOT_FORCE_STOP_ON_REVERSAL) {
        // Force stop before reversing: gradually go to zero first
        if (std::abs(current) > SBOT_REVERSAL_DEADBAND) {
            // Still moving in old direction, slow down to zero
            int step = std::min(SBOT_SLEW_RATE_NORMAL, std::abs(current));
            return current > 0 ? current - step : current + step;
        } else {
            // Close enough to zero, now allow movement in new direction
            int step = std::min(SBOT_SLEW_RATE_NORMAL, std::abs(delta));
            return delta > 0 ? step : -step;
        }
    }
    
    // Apply slew rate limiting (same rate for all movements)
    if (std::abs(delta) <= SBOT_SLEW_RATE_NORMAL) {
        return target;
    }
    
    return current + (delta > 0 ? SBOT_SLEW_RATE_NORMAL : -SBOT_SLEW_RATE_NORMAL);
}

void SbotDrivetrain::arcadeTankControl(pros::Controller& master) {
    int left_input = master.get_analog(SBOT_TANK_LEFT_STICK);
    int right_input = master.get_analog(SBOT_TANK_RIGHT_STICK);

    // Step 1: Apply deadzone
    left_input = applyDeadzone(left_input);
    right_input = applyDeadzone(right_input);

    // Step 2: Apply squared curve for better low-speed control
    left_input = applyCurve(left_input);
    right_input = applyCurve(right_input);

    // Step 3: Scale by sensitivity
    left_input = static_cast<int>(left_input * SBOT_TANK_SENSITIVITY);
    right_input = static_cast<int>(right_input * SBOT_TANK_SENSITIVITY);

    // Step 4: Apply adaptive slew rate limiting to prevent tipping
    // Automatically uses SBOT_SLEW_RATE_NORMAL or SBOT_SLEW_RATE_REVERSAL
    // based on whether direction is reversing
    left_input = applySlewRate(prev_left_cmd, left_input, 0);
    right_input = applySlewRate(prev_right_cmd, right_input, 0);

    // Store for next iteration
    prev_left_cmd = left_input;
    prev_right_cmd = right_input;

    // Send commands to motors
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
