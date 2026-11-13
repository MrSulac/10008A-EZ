#pragma once

#include "ekf.hpp"
#include <array>
#include <cstddef>
#include <cstdint>
#include <initializer_list>

namespace motion {
struct Waypoint { double x, y, heading; }; // Inches, clockwise degrees from +Y.
struct Limits {
    double maxVelocity = 24;       // Center speed, inches/sec.
    double maxAcceleration = 30;  // Center tangential acceleration, inches/sec^2.
    double maxWheelVelocity = 30;
    double maxWheelAcceleration = 45;
    double maxLateralAcceleration = 24;
    double maxAngularVelocity = 2.5; // Radians/sec.
    double trackWidth = ekf::Config{}.trackWidth;
    double sampleSpacing = .5;
};
struct Sample {
    ekf::Pose pose;
    double velocity = 0, acceleration = 0, angularVelocity = 0;
    double curvature = 0, distance = 0;
};
class Path {
 public:
    static constexpr std::size_t CAPACITY = 768;
    struct Node { double x, y, heading, curvature, distance, velocity, time; };
    bool build(std::initializer_list<Waypoint> points, Limits limits = {});
    bool build(const Waypoint* points, std::size_t pointCount, Limits limits = {});
    bool valid() const { return count_ > 1; }
    double duration() const { return valid() ? nodes_[count_ - 1].time : 0; }
    double length() const { return valid() ? nodes_[count_ - 1].distance : 0; }
    std::size_t size() const { return count_; }
    const Node& node(std::size_t index) const { return nodes_[index]; }
    const Limits& limits() const { return limits_; }
    Sample sample(double seconds) const;
 private:
    std::array<Node, CAPACITY> nodes_{};
    std::size_t count_ = 0;
    Limits limits_;
};
struct ChassisVelocity { double forward = 0, angular = 0; };
// Default Ramsete feedback (compatibility API). No motors touched here.
ChassisVelocity feedback(ekf::Pose actual, Sample reference);
struct Gains {
    double kS = 5.0;               // Motor command units (-127..127).
    double kV = 127.0 / (3.14159265358979323846 * 3.25 * 360.0 / 60.0); // 3.25-in wheels, 360 RPM.
    double kA = .04;
    double velocityKp = 1.0;
    double positionTolerance = .75;
    double headingTolerance = 3.0; // Degrees.
    double maxCommand = 100;
    std::uint32_t settleMs = 200;
};
enum class Result { Settled, Timeout, Cancelled, InvalidPath, LocalizationUnavailable };
// Blocking autonomous follower; all exits command zero. Path must outlive call.
Result follow(const Path& path, Gains gains = {}, std::uint32_t timeoutMs = 0);
// Alternate path trackers share motor control, settling, cancellation and health
// checks. The callback only computes velocities; context must outlive the call.
struct TrackingCommand { ChassisVelocity velocity; bool atEnd = false; };
using TrackingController = TrackingCommand (*)(const Path&, ekf::Pose,
    const Sample& timeReference, double measuredSpeed, void* context);
Result followWithController(const Path& path, TrackingController controller,
    void* context, Gains gains = {}, std::uint32_t timeoutMs = 0);
void cancel(); // Called from disabled()/opcontrol(); also cancels a live follow().
const char* resultName(Result result);
} // namespace motion
