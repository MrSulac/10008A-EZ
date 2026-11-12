#include "bot/config.hpp"

pros::Distance frontDistance(1);
pros::Distance backDistance(2);
pros::Distance leftDistance(3);
pros::Distance rightDistance(4);

Sensor frontSensor = {
    frontDistance,
    {0, 5.75},
    0,

    {0, 0},
    {0, 0},

    0,
    0,

    1
};

Sensor backSensor = {
    backDistance,
    {0, -5.75},
    180,

    {0, 0},
    {0, 0},

    0,
    0,

    1
};

Sensor leftSensor = {
    leftDistance,
    {-5.75, 0},
    270,

    {0, 0},
    {0, 0},

    0,
    0,

    2
};

Sensor rightSensor = {
    rightDistance,
    {5.75, 0},
    90,

    {0, 0},
    {0, 0},

    0,
    0,

    2
};

std::vector<Sensor> allSensor = {
    frontSensor,
    backSensor,
    leftSensor,
    rightSensor
};

namespace bot {

// Sample ports: two lift motors and one rotation sensor.
pros::Motor liftMotorLeft(8, pros::MotorGears::green);
pros::Motor liftMotorRight(-9, pros::MotorGears::green);
pros::Rotation liftAngleSensor(10);

// Replace the zero gains with values tuned for the actual lift before use.
constexpr double LIFT_KP_MV_PER_INCH = 0.0;
constexpr double LIFT_KD_MV_PER_DEGREE_PER_SECOND = 0.0;
constexpr double LIFT_KG_MV = 0.0;
constexpr double LIFT_OFFSET_HEIGHT_INCHES = 0.0;
constexpr double LIFT_ARM_LENGTH_INCHES = 12.0;
constexpr double LIFT_ARM_ANGLE_OFFSET_DEGREES = 0.0;
constexpr double LIFT_ARM_DEGREES_PER_SENSOR_DEGREE = 1.0;

namespace {
LiftConfig makeLiftConfig() {
    LiftConfig config;
    config.kP = LIFT_KP_MV_PER_INCH;
    config.kD = LIFT_KD_MV_PER_DEGREE_PER_SECOND;
    config.kG = LIFT_KG_MV;
    config.startingOffsetHeight = LIFT_OFFSET_HEIGHT_INCHES;
    config.armLength = LIFT_ARM_LENGTH_INCHES;
    config.armAngleOffsetDegrees = LIFT_ARM_ANGLE_OFFSET_DEGREES;
    config.armDegreesPerSensorDegree = LIFT_ARM_DEGREES_PER_SENSOR_DEGREE;
    config.loopPeriodMs = 10;
    return config;
}
} // namespace

LiftConfig liftConfig = makeLiftConfig();

Lift liftController(
    {&liftMotorLeft, &liftMotorRight},
    liftAngleSensor,
    liftConfig
);

void runLiftControlTask(void*) {
    liftController.run();
}

// Sample claw hardware ports; change these to match the robot.
pros::Distance clawDistanceSensor(11);
ez::Piston clawPiston(1, false);

ClawConfig clawConfig;
Claw clawController(clawDistanceSensor, clawPiston, clawConfig);

void runClawAutoTask(void*) {
    clawController.runAuto();
}

} // namespace bot
