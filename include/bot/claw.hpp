#pragma once

#include "EZ-Template/piston.hpp"
#include "pros/distance.hpp"

#include <atomic>

namespace bot {

// Configuration for the claw's closing behavior
struct ClawConfig {
    // Distance in mm at which the claw will close automatically
    int closeDistanceMm = 80;
    // Delay in ms after the distance is met before the claw closes
    int closeDelayMs = 100;
    // Delay in ms between sensor checks in the automatic loop
    int loopPeriodMs = 30;
};

class Claw {
public:
    Claw(
        pros::Distance& distanceSensor,
        ez::Piston& piston,
        ClawConfig config = {}
    );

    // Manual pneumatic motions.
    void open();
    void close();

    // Runs one sensor check; true means an object was close and the claw closed.
    bool autoCloseOnce();

    // Blocking automatic close loop, intended to run as a PROS task.
    void runAuto();
    void stopAuto();
    void setAutoEnabled(bool enabled);

private:
    pros::Distance& distanceSensor_;
    ez::Piston& piston_;
    ClawConfig config_;
    std::atomic<bool> autoRunning_{true};
    std::atomic<bool> autoEnabled_{false};
};

} // namespace bot