#include "resets.hpp"
#include "subsystems.hpp"
#include "fieldmap.hpp"

#include <cmath>

namespace resets {

// Sensor Ray Updates

Vec2 getRobotPosition() {
    Pose currentPose;
    currentPose.x = chassis.odom_x_get();
    currentPose.y = chassis.odom_y_get();
    currentPose.theta = chassis.odom_theta_get();
    return {
        currentPose.x,
        currentPose.y
    };
}

Vec2 getSensorPosition(
    Vec2 robotPosition,
    double robotHeading,
    Vec2 sensorOffset
) {
    Vec2 rotatedOffset =
        rotate(sensorOffset, robotHeading);

    return add(
        robotPosition,
        rotatedOffset
    );
}

Vec2 getSensorDirection(
    double robotHeading,
    double sensorAngle
) {
    double angle =
        robotHeading + sensorAngle;

    return {
        cos(degToRad(angle)),
        sin(degToRad(angle))
    };
}

void updateSensorRay(Sensor& sensor, Pose pose) {

    Vec2 robotPosition = {
        pose.x,
        pose.y
    };

    sensor.S = getSensorPosition(
        robotPosition,
        pose.theta,
        sensor.offset
    );

    sensor.U = getSensorDirection(
        pose.theta,
        sensor.angle
    );
}

std::vector<Sensor> localizationSensors = {
    frontSensor,
    backSensor,
    leftSensor,
    rightSensor
};

void updateSensorRays(
    std::vector<Sensor>& sensors,
    Pose pose
) {
    for (Sensor& sensor : sensors) {
        updateSensorRay(sensor, pose);
    }
}

// Ray Prediction

double calculateT(Vec2 S, Vec2 U, Vec2 A, Vec2 B) {
    Vec2 AS = subtract(A, S);
    Vec2 E = subtract(B, A);

    double denominator = cross(U, E);

    return cross(AS, E) / denominator;
}

double calculateU(Vec2 S, Vec2 U, Vec2 A, Vec2 B) {
    Vec2 AS = subtract(A, S);
    Vec2 E = subtract(B, A);

    double denominator = cross(U, E);

    return cross(AS, U) / denominator;
}

bool filterTAndU(double t, double u) {
    const double EPSILON = 1e-6;

    if (t < -EPSILON)
        return false;

    if (u < -EPSILON || u > 1.0 + EPSILON)
        return false;

    return true;
}

WallMatch findBestWallMatch(Sensor& sensor) {
    WallMatch bestMatch = {
        false,
        -1,
        0,
        0
    };

    double smallestError = -1;
    const double EPSILON = 1e-6;

    for (const Segment& segment : localizationMap) {
        Vec2 E = subtract(segment.B, segment.A);
        double denominator = cross(sensor.U, E);

        if (std::abs(denominator) < EPSILON)
            continue;

        double t = calculateT(
            sensor.S,
            sensor.U,
            segment.A,
            segment.B
        );

        double u = calculateU(
            sensor.S,
            sensor.U,
            segment.A,
            segment.B
        );

        if (!filterTAndU(t, u))
            continue;

        double error = std::abs(sensor.measurement - t);

        if (smallestError < 0 || error < smallestError) {
            smallestError = error;

            bestMatch = {
                true,
                segment.id,
                t,
                error
            };
        }
    }

    if (bestMatch.found) {
        sensor.wall = bestMatch.wall;
        sensor.predictedDistance = bestMatch.predictedDistance;
    }

    return bestMatch;
}

double getSensorWeight(
    const Sensor& sensor,
    const Segment& wall
) {
    Vec2 wallDirection = normalize(
        subtract(wall.B, wall.A)
    );

    return std::abs(cross(sensor.U, wallDirection));
}

bool solveRobotPosition(
    std::vector<Sensor>& sensors,
    Pose odometryPose,
    Pose& calculatedPose
) {
    updateSensorRays(sensors, odometryPose);

    const double EPSILON = 1e-6;

    double sumNormalXSquared = 0;
    double sumNormalXNormalY = 0;
    double sumNormalYSquared = 0;
    double sumNormalXConstant = 0;
    double sumNormalYConstant = 0;

    int validSensorCount = 0;

    for (Sensor& sensor : sensors) {
        if (sensor.measurement <= 0)
            continue;

        WallMatch match = findBestWallMatch(sensor);

        if (!match.found)
            continue;

        const Segment* wall = nullptr;

        for (const Segment& segment : localizationMap) {
            if (segment.id == match.wall) {
                wall = &segment;
                break;
            }
        }

        if (wall == nullptr)
            continue;

        Vec2 wallDirection = normalize(
            subtract(wall->B, wall->A)
        );

        Vec2 wallNormal = {
            -wallDirection.y,
            wallDirection.x
        };

        double sensorWeight = getSensorWeight(sensor, *wall);

        if (sensorWeight < EPSILON)
            continue;

        Vec2 rotatedOffset = rotate(
            sensor.offset,
            odometryPose.theta
        );

        Vec2 measuredRay = multiply(
            sensor.U,
            sensor.measurement
        );

        Vec2 positionTerm = subtract(
            subtract(wall->A, rotatedOffset),
            measuredRay
        );

        double c = dot(wallNormal, positionTerm);

        sumNormalXSquared +=
            sensorWeight * wallNormal.x * wallNormal.x;

        sumNormalXNormalY +=
            sensorWeight * wallNormal.x * wallNormal.y;

        sumNormalYSquared +=
            sensorWeight * wallNormal.y * wallNormal.y;

        sumNormalXConstant +=
            sensorWeight * wallNormal.x * c;

        sumNormalYConstant +=
            sensorWeight * wallNormal.y * c;

        validSensorCount++;
    }

    if (validSensorCount < 2)
        return false;

    double determinant =
        sumNormalXSquared * sumNormalYSquared -
        sumNormalXNormalY * sumNormalXNormalY;

    if (std::abs(determinant) < EPSILON)
        return false;

    calculatedPose = odometryPose;
    calculatedPose.x = (
        sumNormalXConstant * sumNormalYSquared -
        sumNormalXNormalY * sumNormalYConstant
    ) / determinant;

    calculatedPose.y = (
        sumNormalXSquared * sumNormalYConstant -
        sumNormalXNormalY * sumNormalXConstant
    ) / determinant;

    return true;
}

// Distance Sensor Checks

double getMedianMeasurement(
    pros::Distance& sensor,
    int n
) {
    std::vector<double> readings;
    readings.reserve(n);

    for (int i = 0; i < n; i++) {

        double reading = sensor.get();

        readings.push_back(reading);

        pros::delay(5);
    }

    return mmToInches(median(readings));
}

volatile bool localizationEnabled = false;

void setLocalizationEnabled(bool enabled) {
    localizationEnabled = enabled;
}

bool runLocalizationOnce() {
    for (Sensor& sensor : localizationSensors) {
        sensor.measurement = getMedianMeasurement(
            sensor.distanceSensor,
            5
        );
    }

    Pose odometryPose;
    odometryPose.x = chassis.odom_x_get();
    odometryPose.y = chassis.odom_y_get();
    odometryPose.theta = chassis.odom_theta_get();
    Pose calculatedPose = odometryPose;

    if (!solveRobotPosition(
        localizationSensors,
        odometryPose,
        calculatedPose
    )) {
        return false;
    }

    // Correct translation while preserving EzTemplate's IMU heading.
    chassis.odom_xy_set(calculatedPose.x, calculatedPose.y);
    return true;
}

void runLocalization() {
    while (true) {
        if (!localizationEnabled) {
            pros::delay(20);
            continue;
        }

        runLocalizationOnce();
        pros::delay(20);
    }
}

}