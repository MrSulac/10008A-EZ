#pragma once

#include <array>
#include <cstdint>

namespace ekf {
// Inches and clockwise degrees from +Y. Covariance uses inches/radians.
struct Pose { double x = 0, y = 0, theta = 0; };
using Matrix = std::array<std::array<double, 3>, 3>;
struct Wall { double ax, ay, bx, by; };
struct SensorMount { double x, y, angle; }; // Angle in degrees, clockwise.
struct Config {
    double trackWidth = 12.0; // TEMPORARY: measure/tune effective drive width.
    double wheelVariancePerInch = 0.012;
    double slipVariancePerInch = 0.004;
    double rangeStdDev = 1.5;
    double gateSquared = 9.0;
};
struct RangePrediction {
    bool valid = false;
    double distance = 0;
    std::array<double, 3> jacobian{};
};
// Standalone, allocation-free EKF math; theta is stored internally in radians.
class Filter {
 public:
    explicit Filter(Config config = {});
    bool reset(Pose pose, double positionStdDev = 1.5, double headingStdDev = 2.0);
    bool predict(double leftDelta, double rightDelta);
    RangePrediction rangePrediction(SensorMount sensor, Wall wall) const;
    bool correctRange(double inches, SensorMount sensor, Wall wall);
    Pose pose() const;
    const Matrix& covariance() const { return covariance_; }
    const Config& config() const { return config_; }
 private:
    Config config_;
    std::array<double, 3> state_{};
    Matrix covariance_{};
};
struct Estimate {
    Pose pose;
    Matrix covariance{};
    bool enabled = false;
    bool healthy = false;
    unsigned acceptedRanges = 0, rejectedRanges = 0;
    std::uint32_t updatedMs = 0, updateMicros = 0;
};
void start(); // Starts the EKF task at (0,0,0), disabling MCL.
void setEnabled(bool enabled);
void reset(Pose pose); // AFTER chassis resets; resumes EKF and disables MCL.
Estimate getEstimate();
void printEstimate(int firstLine = 1);
} // namespace ekf
