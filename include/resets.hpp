#pragma once

#include "bot/config.hpp"
#include <vector>

struct Segment;

namespace resets {

// Position and heading in inches and degrees, used by the reset calculations.
struct Pose {
    double x;
    double y;
    double theta;
};

struct WallMatch {
    bool found;
    int wall;
    double predictedDistance;
    double error;
};

extern std::vector<Sensor> localizationSensors;

Vec2 getRobotPosition();
Vec2 getSensorPosition(Vec2 robotPosition, double robotHeading, Vec2 sensorOffset);
Vec2 getSensorDirection(double robotHeading, double sensorAngle);
void updateSensorRay(Sensor& sensor, Pose pose);
void updateSensorRays(std::vector<Sensor>& sensors, Pose pose);

double calculateT(Vec2 S, Vec2 U, Vec2 A, Vec2 B);
double calculateU(Vec2 S, Vec2 U, Vec2 A, Vec2 B);
bool filterTAndU(double t, double u);
WallMatch findBestWallMatch(Sensor& sensor);
double getSensorWeight(const Sensor& sensor, const Segment& wall);

bool solveRobotPosition(
    std::vector<Sensor>& sensors,
    Pose odometryPose,
    Pose& calculatedPose
);

double getMedianMeasurement(pros::Distance& sensor, int n);
// Takes one set of sensor readings and applies a position correction if the
// localization solver finds a valid solution. Returns whether it succeeded.
bool runLocalizationOnce();
void runLocalization();
void setLocalizationEnabled(bool enabled);

} // namespace resets
