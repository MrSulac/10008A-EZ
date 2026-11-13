#include "stanley.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace stanley {
namespace {
constexpr double PI = 3.14159265358979323846;
constexpr double RAD = PI / 180.0;
double wrap(double angle) { return std::remainder(angle, 2.0 * PI); }
bool positive(double value) { return std::isfinite(value) && value > 0; }
bool validConfig(const Config& config) {
    return std::isfinite(config.crossTrackGain) && config.crossTrackGain >= 0 &&
        positive(config.headingGain) && positive(config.softeningSpeed) &&
        std::isfinite(config.controlPointOffset) && config.controlPointOffset >= 0 &&
        positive(config.searchAhead) && config.searchAhead > config.controlPointOffset &&
        positive(config.launchSpeed);
}
bool validPose(ekf::Pose pose) {
    return std::isfinite(pose.x) && std::isfinite(pose.y) && std::isfinite(pose.theta);
}
struct Projection {
    motion::Sample reference;
    double squaredError = std::numeric_limits<double>::infinity();
    std::size_t segment = 0;
};
// Project onto line segments, not just discrete samples. A full initial search
// is followed by a bounded window; no allocation or whole-path scan per tick.
Projection project(const motion::Path& path, double x, double y,
    std::size_t firstSegment, double maximumDistance) {
    Projection nearest;
    for (std::size_t i = firstSegment; i + 1 < path.size(); ++i) {
        const auto& a = path.node(i);
        const auto& b = path.node(i + 1);
        if (a.distance > maximumDistance) break;
        const double dx = b.x - a.x, dy = b.y - a.y;
        const double squaredLength = dx * dx + dy * dy;
        if (squaredLength < 1e-12) continue;
        const double u = std::max(0.0, std::min(1.0,
            ((x - a.x) * dx + (y - a.y) * dy) / squaredLength));
        const double px = a.x + u * dx, py = a.y + u * dy;
        const double error = (px - x) * (px - x) + (py - y) * (py - y);
        if (error >= nearest.squaredError) continue;
        nearest.squaredError = error;
        nearest.segment = i;
        nearest.reference.pose = {px, py, (a.heading + wrap(b.heading - a.heading) * u) / RAD};
        nearest.reference.curvature = a.curvature + (b.curvature - a.curvature) * u;
        nearest.reference.distance = a.distance + (b.distance - a.distance) * u;
        // Constant acceleration makes v^2 linear in arc length.
        nearest.reference.velocity = std::sqrt(std::max(0.0,
            a.velocity * a.velocity + (b.velocity * b.velocity - a.velocity * a.velocity) * u));
    }
    return nearest;
}
struct Tracker {
    Config config;
    bool initialized = false;
    std::size_t segment = 0;
    double progress = 0;
};
motion::TrackingCommand track(const motion::Path& path, ekf::Pose pose,
    const motion::Sample&, double measuredSpeed, void* context) {
    auto& state = *static_cast<Tracker*>(context);
    if (!validPose(pose) || !std::isfinite(measuredSpeed)) {
        return {{std::numeric_limits<double>::quiet_NaN(), 0}, false};
    }
    const auto first = state.initialized && state.segment > 2 ? state.segment - 2 : 0;
    const double last = state.initialized ? state.progress + state.config.searchAhead : path.length();
    const auto center = project(path, pose.x, pose.y, first, last);
    if (!std::isfinite(center.squaredError)) {
        return {{std::numeric_limits<double>::quiet_NaN(), 0}, false};
    }
    state.segment = center.segment;
    state.progress = std::max(state.progress, center.reference.distance);
    state.initialized = true;
    // Completion follows actual spatial progress, not elapsed profile time.
    // The shared terminal controller settles center position and final heading.
    if (state.progress >= path.length() - .75) return {{0, 0}, true};

    auto reference = center.reference;
    if (state.config.controlPointOffset > 0) {
        const double t = pose.theta * RAD;
        const auto front = project(path,
            pose.x + state.config.controlPointOffset * std::sin(t),
            pose.y + state.config.controlPointOffset * std::cos(t), first, last);
        reference.pose = front.reference.pose;
        reference.curvature = front.reference.curvature;
    }
    // A spatial profile has zero speed at s=0. A small launch target starts
    // motion; the shared wheel-target acceleration limiter still ramps it.
    if (state.progress < 1.0) reference.velocity = std::max(reference.velocity, state.config.launchSpeed);
    const double curvature = std::abs(reference.curvature);
    double speedCap = std::min(path.limits().maxVelocity,
        path.limits().maxWheelVelocity / (1.0 + curvature * path.limits().trackWidth * .5));
    if (curvature > 1e-8) speedCap = std::min({speedCap,
        std::sqrt(path.limits().maxLateralAcceleration / curvature),
        path.limits().maxAngularVelocity / curvature});
    reference.velocity = std::min(reference.velocity, speedCap);
    return {feedback(pose, reference, measuredSpeed, state.config), false};
}
} // namespace

motion::ChassisVelocity feedback(ekf::Pose actual, motion::Sample reference,
    double measuredSpeed, Config config) {
    if (!validConfig(config) || !validPose(actual) || !validPose(reference.pose) ||
        !std::isfinite(reference.velocity) || reference.velocity < 0 ||
        !std::isfinite(reference.curvature) || !std::isfinite(measuredSpeed)) return {};
    const double heading = actual.theta * RAD;
    const double px = actual.x + config.controlPointOffset * std::sin(heading);
    const double py = actual.y + config.controlPointOffset * std::cos(heading);
    const double pathHeading = reference.pose.theta * RAD;
    // Positive error means the path is to the robot's right in our CW frame.
    const double crossTrack = std::cos(pathHeading) * (reference.pose.x - px) -
                             std::sin(pathHeading) * (reference.pose.y - py);
    const double headingError = wrap(pathHeading - heading);
    const double steering = headingError + std::atan2(config.crossTrackGain * crossTrack,
        std::abs(measuredSpeed) + config.softeningSpeed);
    const double speed = reference.velocity * std::max(0.0, std::cos(headingError));
    // Differential drive has no steering servo. Apply the Stanley angle as
    // proportional yaw feedback, plus the mapped path's curvature feedforward.
    return {speed, speed * reference.curvature + config.headingGain * wrap(steering)};
}

motion::Result follow(const motion::Path& path, Config config,
    motion::Gains driveGains, std::uint32_t timeoutMs) {
    if (!validConfig(config)) {
        motion::cancel();
        return motion::Result::InvalidPath;
    }
    Tracker tracker{config};
    return motion::followWithController(path, track, &tracker, driveGains, timeoutMs);
}
} // namespace stanley
