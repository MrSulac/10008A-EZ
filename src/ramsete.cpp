#include "ramsete.hpp"

#include <cmath>

namespace ramsete {
namespace {
constexpr double PI = 3.14159265358979323846;
constexpr double RAD = PI / 180.0;
bool validConfig(Config config) {
    return std::isfinite(config.b) && config.b > 0 &&
        std::isfinite(config.zeta) && config.zeta > 0 && config.zeta < 1;
}
bool validPose(ekf::Pose pose) {
    return std::isfinite(pose.x) && std::isfinite(pose.y) && std::isfinite(pose.theta);
}
motion::TrackingCommand track(const motion::Path& path, ekf::Pose actual,
    const motion::Sample& reference, double, void* context) {
    const auto config = *static_cast<const Config*>(context);
    return {feedback(actual, reference, config), reference.distance >= path.length()};
}
} // namespace

motion::ChassisVelocity feedback(ekf::Pose actual, motion::Sample reference, Config config) {
    if (!validConfig(config) || !validPose(actual) || !validPose(reference.pose) ||
        !std::isfinite(reference.velocity) || !std::isfinite(reference.angularVelocity)) return {};
    const double theta = actual.theta * RAD;
    const double dx = reference.pose.x - actual.x;
    const double dy = reference.pose.y - actual.y;
    // Rotate field displacement into robot-forward and robot-right coordinates.
    const double forwardError = std::sin(theta) * dx + std::cos(theta) * dy;
    const double rightError = std::cos(theta) * dx - std::sin(theta) * dy;
    const double angleError = std::remainder((reference.pose.theta - actual.theta) * RAD, 2.0 * PI);
    const double v = reference.velocity, omega = reference.angularVelocity;
    const double k = 2.0 * config.zeta * std::sqrt(omega * omega + config.b * v * v);
    // sinc(0)=1; the series avoids cancellation or a divide-by-zero near zero.
    const double sinc = std::abs(angleError) < 1e-6
        ? 1.0 - angleError * angleError / 6.0 : std::sin(angleError) / angleError;
    return {v * std::cos(angleError) + k * forwardError,
            omega + k * angleError + config.b * v * sinc * rightError};
}

motion::Result follow(const motion::Path& path, Config config,
    motion::Gains driveGains, std::uint32_t timeoutMs) {
    if (!validConfig(config)) {
        motion::cancel();
        return motion::Result::InvalidPath;
    }
    return motion::followWithController(path, track, &config, driveGains, timeoutMs);
}
} // namespace ramsete
