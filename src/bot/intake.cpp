#include "bot/intake.hpp"

namespace bot {

// Sample 11 W V5 motor on port 12; change the port and cartridge to match the robot.
pros::Motor intakeMotor(12, pros::MotorGears::green);

} // namespace bot
