#include "main.h"
#include "bot/config.hpp"

#include <cmath>

void test_robot_functions() {
  // Add testing functions here, eg driving, intaking, lifting, etc.
}

/**
 * Gives you some extras to run in your opcontrol:
 * - run your autonomous routine in opcontrol by pressing DOWN and B
 *   - to prevent this from accidentally happening at a competition, this
 *     is only enabled when you're not connected to competition control.
 * - gives you a GUI to change your PID values live by pressing X
 */
void ez_template_extras() {
  // Only run this when not connected to a competition switch
  if (!pros::competition::is_connected()) {
    // PID Tuner
    // - after you find values that you're happy with, you'll have to set them in auton.cpp

    // Enable / Disable PID Tuner
    //  When enabled:
    //  * use A and Y to increment / decrement the constants
    //  * use the arrow keys to navigate the constants
    if (master.get_digital_new_press(DIGITAL_X)) chassis.pid_tuner_toggle();

    // Trigger the selected autonomous routine
    if (master.get_digital(DIGITAL_B) && master.get_digital(DIGITAL_DOWN)) {
      pros::motor_brake_mode_e_t preference = chassis.drive_brake_get();
      autonomous();
      chassis.drive_brake_set(preference);
    }

    if (master.get_digital(DIGITAL_LEFT) && master.get_digital(DIGITAL_Y)) {
      test_robot_functions();
    }

    // Allow PID Tuner to iterate
    chassis.pid_tuner_iterate();
  }

  // Disable PID Tuner when connected to a comp switch
  else {
    if (chassis.pid_tuner_enabled()) chassis.pid_tuner_disable();
  }
}

void runOpcontrol() {
    bot::clawController.setAutoEnabled(true);
    pros::Task clawAutoTask(
        bot::runClawAutoTask,
        nullptr,
        "Claw Auto"
    );

    // Start by holding the lift where it is when driver control begins.
    const double liftStartHeight = bot::liftController.getCurrentHeight();
    const double liftMinimumHeight = bot::liftConfig.startingOffsetHeight;
    const double liftMaximumHeight = bot::liftController.getMaxHeight();
    bot::liftController.setTargetHeight(liftStartHeight);

    // This is preference to what you like to drive on
    chassis.drive_brake_set(MOTOR_BRAKE_COAST);

    while (true) {
        // Gives you some extras to make EZ-Template ezier
        ez_template_extras();        

        // Curved right-stick input commands a height relative to the lift's
        // starting height. The center position keeps that starting height.
        constexpr double JOYSTICK_DEADBAND = 5.0;
        const double joystick = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        double normalizedInput = 0.0;
        if (std::abs(joystick) > JOYSTICK_DEADBAND) {
            const double magnitude =
                (std::abs(joystick) - JOYSTICK_DEADBAND) /
                (127.0 - JOYSTICK_DEADBAND);
            // Quadratic drive curve
            normalizedInput = std::copysign(magnitude * magnitude, joystick);
        }

        const double targetHeight = normalizedInput >= 0.0
            ? liftStartHeight + normalizedInput * (liftMaximumHeight - liftStartHeight)
            : liftStartHeight + normalizedInput * (liftStartHeight - liftMinimumHeight);
        bot::liftController.setTargetHeight(targetHeight);

        // chassis.opcontrol_tank();  // Tank control
        // chassis.opcontrol_arcade_standard(ez::SPLIT);   // Standard split arcade
        chassis.opcontrol_arcade_standard(ez::SINGLE);  // Standard single arcade
        // chassis.opcontrol_arcade_flipped(ez::SPLIT);    // Flipped split arcade
        // chassis.opcontrol_arcade_flipped(ez::SINGLE);   // Flipped single arcade

        // Toggle Claw
        if (master.get_digital_new_press(DIGITAL_R1)) {
          bot::clawPiston.set(!bot::clawPiston.get());
        }

        

        pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
