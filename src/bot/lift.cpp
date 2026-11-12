#include "bot/lift.hpp"

#include <cmath>

namespace bot {
namespace {

constexpr double PI = 3.14159265358979323846;
constexpr double MAX_MOTOR_VOLTAGE_MV = 12000.0;

double clampVoltage(double voltage) {
    if (voltage > MAX_MOTOR_VOLTAGE_MV) return MAX_MOTOR_VOLTAGE_MV;
    if (voltage < -MAX_MOTOR_VOLTAGE_MV) return -MAX_MOTOR_VOLTAGE_MV;
    return voltage;
}

}

double degreesToHeight(
    double armAngleDegrees,
    double startingOffsetHeight,
    double armLength
) {
    return startingOffsetHeight +
        armLength * std::sin(armAngleDegrees * PI / 180.0);
}

Lift::Lift(
    std::initializer_list<pros::Motor*> motors,
    pros::Rotation& angleSensor,
    LiftConfig config
) : motors_(motors), angleSensor_(angleSensor), config_(config) {
    if (config_.loopPeriodMs < 1) config_.loopPeriodMs = 10;
}

void Lift::setTargetHeight(double targetHeight) {
    const double maxHeight = getMaxHeight();
    const double minHeight = config_.startingOffsetHeight;
    if (targetHeight > maxHeight) targetHeight = maxHeight;
    if (targetHeight < minHeight) targetHeight = minHeight;
    targetHeight_.store(targetHeight);
}

double Lift::getTargetHeight() const {
    return targetHeight_.load();
}

// Arm Angle is from Horizontal, so 0 degrees is horizontal and 90 degrees is vertical.
double Lift::getMaxHeight() const {
    return degreesToHeight(
        config_.maxArmAngleDegrees,
        config_.startingOffsetHeight,
        config_.armLength
    );
}

double Lift::getCurrentHeight() const {
    const double sensorPositionDegrees = angleSensor_.get_position() / 100.0;
    const double armAngleDegrees =
        config_.armAngleOffsetDegrees +
        sensorPositionDegrees * config_.armDegreesPerSensorDegree;
    return degreesToHeight(
        armAngleDegrees,
        config_.startingOffsetHeight,
        config_.armLength
    );
}

int Lift::update() {
    // PROS Rotation reports position in centidegrees and velocity in
    // centidegrees per second.
    const double sensorPositionDegrees = angleSensor_.get_position() / 100.0;
    const double armVelocityDegreesPerSecond = angleSensor_.get_velocity() / 100.0;

    const double armAngleDegrees =
        config_.armAngleOffsetDegrees +
        sensorPositionDegrees * config_.armDegreesPerSensorDegree;
    const double armAngleRadians = armAngleDegrees * PI / 180.0;
    const double currentHeight = degreesToHeight(armAngleDegrees,
        config_.startingOffsetHeight, config_.armLength);
    const double heightError = targetHeight_.load() - currentHeight;

    // Gravity compensation holds the lift against its load; feedback drives
    // position error toward zero and damps motion.
    const double feedforwardMv = config_.kG * std::cos(armAngleRadians);
    const double feedbackMv =
        config_.kP * heightError -
        config_.kD * armVelocityDegreesPerSecond * config_.armDegreesPerSensorDegree;
    const int voltageMv = static_cast<int>(
        std::lround(clampVoltage(feedforwardMv + feedbackMv))
    );

    for (pros::Motor* motor : motors_) {
        if (motor != nullptr) motor->move_voltage(voltageMv);
    }
    return voltageMv;
}

void Lift::run() {
    running_.store(true);
    while (running_.load()) {
        update();
        pros::delay(config_.loopPeriodMs);
    }
    for (pros::Motor* motor : motors_) {
        if (motor != nullptr) motor->move_voltage(0);
    }
}

void Lift::stop() {
    running_.store(false);
}

} // namespace bot
