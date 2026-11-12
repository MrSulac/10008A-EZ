#pragma once

#include "pros/motors.hpp"
#include "pros/rotation.hpp"

#include <atomic>
#include <initializer_list>
#include <vector>

namespace bot {

struct LiftConfig {
    // Feedback gains. kP is mV per height unit; kD is mV/(arm degree/second).
    double kP = 0.0;
    double kD = 0.0;

    // Gravity feedforward magnitude in mV at the arm's horizontal position.
    double kG = 0.0;

    // Lift geometry. Keep these in the same height units (for example inches).
    double startingOffsetHeight = 0.0;
    double armLength = 0.0;
    // Stop before the arms reach vertical / the lift's near-toggle position.
    double maxArmAngleDegrees = 85.0;

    // Converts rotation sensor degrees to approximate lift arm degrees.
    double armAngleOffsetDegrees = 0.0;
    double armDegreesPerSensorDegree = 1.0;

    int loopPeriodMs = 10;
};

// Assumes the arm angle is measured from horizontal. The result uses the same
// distance units as startingOffsetHeight and armLength.
double degreesToHeight(
    double armAngleDegrees,
    double startingOffsetHeight,
    double armLength
);

class Lift {
public:
    explicit Lift(
        std::initializer_list<pros::Motor*> motors,
        pros::Rotation& angleSensor,
        LiftConfig config = {}
    );

    // Target height is in the same units as LiftConfig::armLength.
    void setTargetHeight(double targetHeight);
    double getTargetHeight() const;
    double getMaxHeight() const;
    double getCurrentHeight() const;

    // Calculates and applies one voltage-control iteration; returns mV sent.
    int update();

    // Run this blocking loop in a PROS task. stop() exits the loop.
    void run();
    void stop();

private:
    std::vector<pros::Motor*> motors_;
    pros::Rotation& angleSensor_;
    LiftConfig config_;
    std::atomic<double> targetHeight_{0.0};
    std::atomic<bool> running_{true};
};

} // namespace bot
