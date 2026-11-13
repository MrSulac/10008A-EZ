// Host-only: actual production path math with inert test hardware stubs.
#include "../../src/motion_profile.cpp"
#include "../../src/ramsete.cpp"
#include <iostream>
#include <iomanip>
#include <vector>
namespace ekf { Estimate getEstimate() { return {}; } }
int main() {
    std::size_t count;
    if (!(std::cin >> count) || count < 2 || count > motion::Path::CAPACITY) return 1;
    std::vector<motion::Waypoint> points(count);
    for (auto& p : points) if (!(std::cin >> p.x >> p.y >> p.heading)) return 1;
    motion::Limits limits;
    if (!(std::cin >> limits.maxVelocity >> limits.maxAcceleration >> limits.maxWheelVelocity
        >> limits.maxWheelAcceleration >> limits.maxLateralAcceleration >> limits.maxAngularVelocity
        >> limits.trackWidth >> limits.sampleSpacing)) return 1;
    static motion::Path path;
    if (!path.build(points.data(), points.size(), limits) || path.duration() > 600) return 2;
    std::cout << std::fixed << std::setprecision(5) << "[";
    const auto steps = static_cast<std::size_t>(std::ceil(path.duration() / .02));
    for (std::size_t i = 0; i <= steps; ++i) {
        const double t = std::min(i * .02, path.duration());
        const auto s = path.sample(t);
        if (i) std::cout << ',';
        std::cout << "{\"t\":" << t << ",\"x\":" << s.pose.x << ",\"y\":" << s.pose.y
            << ",\"theta\":" << s.pose.theta << ",\"v\":" << s.velocity << ",\"omega\":" << s.angularVelocity << '}';
    }
    std::cout << "]\n";
}
