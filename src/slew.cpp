#include "slew.hpp"

#include <cstdlib>

namespace motor_slew {

namespace {

int velocityLimit(const pros::AbstractMotor& motor) {
    int limit = 600;
    bool foundGearset = false;

    for (pros::MotorGears gearset : motor.get_gearing_all()) {
        int gearsetLimit;
        switch (gearset) {
            case pros::MotorGears::red:
                gearsetLimit = 100;
                break;
            case pros::MotorGears::green:
                gearsetLimit = 200;
                break;
            case pros::MotorGears::blue:
                gearsetLimit = 600;
                break;
            default:
                continue;
        }

        if (!foundGearset || gearsetLimit < limit) limit = gearsetLimit;
        foundGearset = true;
    }

    return limit;
}

} // namespace

int stepVoltage(int currentVoltage, int targetVoltage, int maxStep) {
    constexpr int MAX_VOLTAGE = 12000;
    if (targetVoltage > MAX_VOLTAGE) targetVoltage = MAX_VOLTAGE;
    if (targetVoltage < -MAX_VOLTAGE) targetVoltage = -MAX_VOLTAGE;

    if (maxStep <= 0) return currentVoltage;

    const int difference = targetVoltage - currentVoltage;
    if (std::abs(difference) <= maxStep) return targetVoltage;

    return currentVoltage + (difference > 0 ? maxStep : -maxStep);
}

int moveVoltage(
    pros::AbstractMotor& motor,
    int targetVoltage,
    int& currentVoltage,
    int maxStep
) {
    currentVoltage = stepVoltage(currentVoltage, targetVoltage, maxStep);
    motor.move_voltage(currentVoltage);
    return currentVoltage;
}

int stepVelocity(int currentVelocity, int targetVelocity, int maxStep) {
    const int difference = targetVelocity - currentVelocity;
    if (maxStep <= 0) return currentVelocity;
    if (std::abs(difference) <= maxStep) return targetVelocity;

    return currentVelocity + (difference > 0 ? maxStep : -maxStep);
}

int moveVelocity(
    pros::AbstractMotor& motor,
    int targetVelocity,
    int& currentVelocity,
    int maxStep
) {
    const int limit = velocityLimit(motor);
    if (targetVelocity > limit) targetVelocity = limit;
    if (targetVelocity < -limit) targetVelocity = -limit;

    currentVelocity = stepVelocity(currentVelocity, targetVelocity, maxStep);
    motor.move_velocity(currentVelocity);
    return currentVelocity;
}

} // namespace motor_slew
