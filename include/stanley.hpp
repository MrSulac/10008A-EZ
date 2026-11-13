#pragma once

#include "motion_profile.hpp"

// Stanley heading/cross-track feedback adapted to differential-drive turn rate.
// Uses EKF pose, inches and clockwise degrees from +Y; forward paths only.
namespace stanley {
struct Config {
    double crossTrackGain = 2.0;  // 1/sec; larger values pull toward the path faster.
    double headingGain = 3.0;     // 1/sec; steering-angle error -> yaw rate.
    double softeningSpeed = 3.0;  // in/sec; keeps cross-track feedback finite at rest.
    double controlPointOffset = 0.0; // Inches forward of the drive axle/pose center.
    double searchAhead = 12.0;    // Arc-length search window after first projection.
    double launchSpeed = 3.0;     // in/sec; leaves the profile's zero-speed start.
};
// reference.pose is the nearest path point; velocity is the requested speed,
// curvature is clockwise 1/inch. Does not read sensors or touch motors.
motion::ChassisVelocity feedback(ekf::Pose actual, motion::Sample reference,
    double measuredSpeed, Config config = {});
// Blocking follower. Reuses motion::Path and the shared motor-control loop.
// Use motion::cancel() to stop either this controller or the original follower.
motion::Result follow(const motion::Path& path, Config config = {},
    motion::Gains driveGains = {}, std::uint32_t timeoutMs = 0);
} // namespace stanley
