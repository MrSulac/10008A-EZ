#pragma once

#include <cstddef>
#include <cstdint>

// Inches; clockwise degrees from +Y. Field origin is the bottom-left corner.
namespace mcl {
constexpr std::size_t PARTICLE_COUNT = 200;

struct Pose {
    double x = 0;
    double y = 0;
    double theta = 0;
};

struct Comparison {
    Pose encoder;                 // Drive encoders + IMU, without wall corrections.
    Pose localized;               // The 200-particle fused estimate.
    Pose distance;                // Independent wall solution using localized heading.
    bool distanceValid = false;   // Requires at least two nonparallel wall normals.
    bool enabled = false;
    unsigned validSensors = 0;
    double effectiveParticles = PARTICLE_COUNT;
    double residualInches = 0;    // RMS measured-versus-predicted range error.
    double spreadInches = 0;      // Particle position standard deviation.
    std::uint32_t updateMicros = 0;
};

// Call once after chassis.initialize(). Starts at (0, 0, 0).
void start();
// Pause before resetting chassis encoders/IMU/odometry. reset() resumes MCL and
// takes new encoder baselines; call it AFTER those chassis resets finish.
void setEnabled(bool enabled);
void reset(Pose pose, double positionStdDev = 1.5, double headingStdDev = 1.0);
Comparison getComparison();
void printComparison(int firstLine = 1);
} // namespace mcl
