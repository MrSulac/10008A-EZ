#pragma once

#include "EZ-Template/piston.hpp"
#include "pros/distance.hpp"

#include <atomic>

namespace bot {

struct ClawConfig {
    // PROS distance sensor readings are in millimeters.
    int closeDistanceMm = 80;
    int closeDelayMs = 100;
    int loopPeriodMs = 20;
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
