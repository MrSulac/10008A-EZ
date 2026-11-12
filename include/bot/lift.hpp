#pragma once

#include "pros/motors.hpp"
#include "pros/rotation.hpp"

#include <atomic>
#include <initializer_list>
#include <vector>

namespace bot {

// Configuration for the lift's movement behavior
struct LiftConfig {
    // Feedback gains. kP is mV per height unit; kD is mV/(arm degree/second).
    double kP = 0.0;
    double kD = 0.0;

    // Gravity feedforward magnitude in mV at the arm's horizontal position.
    double kG = 0.0;

    // Lift geometry. Keep these in the same height units as the configured arm length units.
    double startingOffsetHeight = 0.0;
    double armLength = 0.0;
    // The maximum angle the lift can reach, in degrees
    // Prevents lift from trying to go beyond its physical limits
    double maxArmAngleDegrees = 85.0;

    // Converts rotation sensor degrees to approximate lift arm degrees
    // Should be 1 to 1 if the sensor is on the lift arm axle
    double armAngleOffsetDegrees = 0.0;
    double armDegreesPerSensorDegree = 1.0;

    int loopPeriodMs = 10;
};

// Arm is angle is measured from the horizontal
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
