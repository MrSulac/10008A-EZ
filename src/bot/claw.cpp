#include "bot/claw.hpp"
#include "pros/misc.hpp"

namespace bot {

Claw::Claw(
    pros::Distance& distanceSensor,
    ez::Piston& piston,
    ClawConfig config
) : distanceSensor_(distanceSensor), piston_(piston), config_(config) {
    if (config_.closeDistanceMm < 1) config_.closeDistanceMm = 80;
    if (config_.closeDelayMs < 0) config_.closeDelayMs = 0;
    if (config_.loopPeriodMs < 1) config_.loopPeriodMs = 20;
}

void Claw::open() {
    piston_.set(false);
}

void Claw::close() {
    piston_.set(true);
}

bool Claw::autoCloseOnce() {
    const int distanceMm = distanceSensor_.get();
    if (distanceMm <= 0 || distanceMm > config_.closeDistanceMm) return false;

    pros::delay(config_.closeDelayMs);
    close();
    return true;
}

void Claw::runAuto() {
    autoRunning_.store(true);
    while (autoRunning_.load()) {
        const bool driverMode = !pros::competition::is_connected() ||
            (!pros::competition::is_autonomous() && !pros::competition::is_disabled());
        if (autoEnabled_.load() && driverMode) autoCloseOnce();
        pros::delay(config_.loopPeriodMs);
    }
}

void Claw::stopAuto() {
    autoRunning_.store(false);
}

void Claw::setAutoEnabled(bool enabled) {
    autoEnabled_.store(enabled);
}

} // namespace bot
