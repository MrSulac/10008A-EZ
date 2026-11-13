#pragma once

#include "motion_profile.hpp"

// Time-parameterized Ramsete trajectory tracking in inches, clockwise degrees
// from +Y. Angular velocities and internal heading errors use radians.
namespace ramsete {
struct Config {
    double b = 0.015; // >0, rad^2/in^2. Matches the existing profile follower.
    double zeta = 0.7; // Damping: strictly between 0 and 1.
};
// Pure calculation: returns corrected center speed (in/s) and turn rate (rad/s).
// reference.angularVelocity is used directly, rather than inferred from a point.
motion::ChassisVelocity feedback(ekf::Pose actual, motion::Sample reference,
    Config config = {});
// Follows the profile clock; reuses EKF, motor feedback, limits and settling.
// motion::cancel() stops this controller as well as the other path followers.
motion::Result follow(const motion::Path& path, Config config = {},
    motion::Gains driveGains = {}, std::uint32_t timeoutMs = 0);
} // namespace ramsete
