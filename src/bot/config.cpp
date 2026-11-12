#include "bot/config.hpp"
#include "main.h"

//cstdio for SD Card functions, cstring for string comparison
#include <cstdio>
#include <cstring>

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

const char* allianceColor = nullptr;

void loadAllianceColor() {
    allianceColor = nullptr;
    if (!pros::usd::is_installed()) return;

    FILE* file = std::fopen("/usd/alliance.txt", "r");
    if (file == nullptr) return;

    char storedColor[8] = {};
    const bool readOk = std::fscanf(file, "%7s", storedColor) == 1;
    std::fclose(file);

    if (!readOk) return;
    if (std::strcmp(storedColor, "Red") == 0) allianceColor = "Red";
    else if (std::strcmp(storedColor, "Blue") == 0) allianceColor = "Blue";
}

void setAllianceColor(const char* color) {
    if (color == nullptr || std::strcmp(color, "Red") == 0) allianceColor = color;
    else if (std::strcmp(color, "Blue") == 0) allianceColor = "Blue";
    else return;

    if (!pros::usd::is_installed()) {
        allianceColor = nullptr;
        return;
    }

    FILE* file = std::fopen("/usd/alliance.txt", "w");
    if (file == nullptr) {
        allianceColor = nullptr;
        return;
    }
    if (allianceColor != nullptr) std::fprintf(file, "%s\n", allianceColor);
    std::fclose(file);
}

void runAllianceSelectionTask(void*) {
    bool comboWasPressed = false;
    while (true) {
        const bool autonomousMode = pros::competition::is_autonomous();
        const bool comboPressed =
            master.get_digital(pros::E_CONTROLLER_DIGITAL_L2) &&
            master.get_digital(pros::E_CONTROLLER_DIGITAL_R2);

        if (!autonomousMode && comboPressed && !comboWasPressed) {
            setAllianceColor(
                allianceColor != nullptr && std::strcmp(allianceColor, "Red") == 0
                    ? "Blue"
                    : "Red"
            );
            master.rumble(".");
        }

        comboWasPressed = comboPressed;
        pros::delay(50);
    }
}

// Sample ports: two lift motors and one rotation sensor.
pros::Motor liftMotorLeft(8, pros::MotorGears::green);
pros::Motor liftMotorRight(-9, pros::MotorGears::green);
pros::Rotation liftRotationSensor(10);

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
    liftRotationSensor,
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
