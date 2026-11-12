#pragma once

#include "pros/abstract_motor.hpp"

namespace motor_slew {

// Move currentVoltage toward targetVoltage by at most maxStep millivolts.
// PROS motor voltage commands are limited to -12000 to 12000 mV.
int stepVoltage(int currentVoltage, int targetVoltage, int maxStep);

// Slew and send a voltage command to a PROS Motor or MotorGroup.
// Keep currentVoltage between calls and initialize it to the motor's current
// commanded voltage (usually 0 when starting).
int moveVoltage(
    pros::AbstractMotor& motor,
    int targetVoltage,
    int& currentVoltage,
    int maxStep
);

// Move currentVelocity toward targetVelocity by at most maxStep RPM.
int stepVelocity(int currentVelocity, int targetVelocity, int maxStep);

// Slew and send an RPM command to a PROS Motor or MotorGroup. The target is
// bounded to the slowest configured cartridge in a motor group.
int moveVelocity(
    pros::AbstractMotor& motor,
    int targetVelocity,
    int& currentVelocity,
    int maxStep
);

} // namespace motor_slew
