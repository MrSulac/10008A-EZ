#pragma once

#include "api.h"
#include "bot/claw.hpp"
#include "bot/lift.hpp"
#include "bot/intake.hpp"
#include "util.hpp"
#include <vector>

struct Sensor {
    pros::Distance& distanceSensor;

    // Position of the sensor relative to the robot's center
    // Positive Y is Forward, Positive X is Right
    Vec2 offset;
    // Orientation of the sensor relative to the robot's forward direction (in degrees)
    double angle;

    Vec2 S;
    Vec2 U;

    double measurement;
    double predictedDistance;

    int wall;
};

extern pros::Distance frontDistance;
extern pros::Distance backDistance;
extern pros::Distance leftDistance;
extern pros::Distance rightDistance;

extern Sensor frontSensor;
extern Sensor backSensor;
extern Sensor leftSensor;
extern Sensor rightSensor;

extern std::vector<Sensor> allSensor;

namespace bot {
extern pros::Motor liftMotorLeft;
extern pros::Motor liftMotorRight;
extern pros::Rotation liftRotationSensor;
extern LiftConfig liftConfig;
extern Lift liftController;

void runLiftControlTask(void*);

extern pros::Distance clawDistanceSensor;
extern ez::Piston clawPiston;
extern ClawConfig clawConfig;
extern Claw clawController;
void runClawAutoTask(void*);

// Points to the selected string literal ("Red" or "Blue"), or nullptr when
// there is no valid selection stored on the SD card.
extern const char* allianceColor;
void loadAllianceColor();
void setAllianceColor(const char* color);
void runAllianceSelectionTask(void*);
} // namespace bot
