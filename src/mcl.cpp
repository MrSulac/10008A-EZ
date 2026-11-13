#include "mcl.hpp"
#include "ekf.hpp"

#include "bot/config.hpp"
#include "fieldmap.hpp"
#include "subsystems.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <limits>

namespace mcl {
namespace {
// All tuning stays here. No sensor reads, heap allocations, or delays occur
// inside a particle loop. 50 Hz prediction, at most 20 Hz range updates.
constexpr std::uint32_t LOOP_MS = 20;
constexpr std::uint32_t RANGE_MS = 50;
constexpr float PI = 3.14159265358979323846f;
constexpr float RAD = PI / 180.0f;
constexpr float FIELD_SIZE = 144.0f;
constexpr float RANGE_SIGMA = 1.5f;
constexpr float RANGE_MAX_MM = 2000.0f;
constexpr float OUTLIER_FLOOR = 0.08f;
constexpr float RESAMPLE_THRESHOLD = PARTICLE_COUNT * 0.55f;
// Preserve the IMU heading used by EZ-Template's turn controller.
constexpr bool CORRECT_CHASSIS_XY = true;
constexpr std::size_t MAX_SEGMENTS = 64;

struct Particle {
    float x, y, theta, weight;
};
struct RaySensor {
    pros::Distance* device = nullptr;
    float x = 0, y = 0;             // Robot-relative offsets.
    float dx = 0, dy = 1;           // Robot-relative direction.
    float range = 0;
    bool valid = false;
};
struct Edge {
    float x, y, dx, dy;
};
struct Hit {
    float range = std::numeric_limits<float>::infinity();
    int edge = -1;
};

std::array<Particle, PARTICLE_COUNT> particles;
std::array<Particle, PARTICLE_COUNT> scratch;
std::array<RaySensor, 4> sensors;
std::array<Edge, MAX_SEGMENTS> edges;
std::size_t edgeCount = 0;
pros::Mutex mutex;
Comparison comparison;
bool enabled = false;
bool haveBaseline = false;
bool haveObservation = false;
double lastLeft = 0, lastRight = 0, lastHeading = 0;
std::uint32_t lastMotionMs = 0, lastRangeMs = 0;
std::uint32_t randomState = 0x10008a31u;
float sinceRange = 0;

// Xorshift and a bounded, zero-mean, unit-variance Gaussian approximation.
// Avoids random_device, Box-Muller logarithms, and trigonometry for noise.
float uniform() {
    randomState ^= randomState << 13;
    randomState ^= randomState >> 17;
    randomState ^= randomState << 5;
    return static_cast<float>(randomState >> 8) * (1.0f / 16777216.0f);
}
float noise() {
    return (uniform() + uniform() + uniform() - 1.5f) * 2.0f;
}
float wrap(float theta) {
    return std::remainder(theta, 2.0f * PI);
}
bool validReading(double value) {
    return std::isfinite(value) && std::abs(value) < 1.0e8;
}
float cross(float ax, float ay, float bx, float by) {
    return ax * by - ay * bx;
}

void cacheGeometry() {
    // Reuse the existing configuration, but don't mutate the old reset solver.
    const Sensor* configured[] = {&frontSensor, &backSensor, &leftSensor, &rightSensor};
    for (std::size_t i = 0; i < sensors.size(); ++i) {
        const auto& source = *configured[i];
        sensors[i].device = &source.distanceSensor;
        sensors[i].x = source.offset.x;
        sensors[i].y = source.offset.y;
        sensors[i].dx = std::sin(source.angle * RAD);
        sensors[i].dy = std::cos(source.angle * RAD);
    }
    // Refuse a truncated map rather than silently predicting through obstacles.
    edgeCount = localizationMap.size() <= MAX_SEGMENTS ? localizationMap.size() : 0;
    for (std::size_t i = 0; i < edgeCount; ++i) {
        const auto& segment = localizationMap[i];
        edges[i] = {static_cast<float>(segment.A.x), static_cast<float>(segment.A.y),
                    static_cast<float>(segment.B.x - segment.A.x),
                    static_cast<float>(segment.B.y - segment.A.y)};
    }
}

Hit raycast(const Particle& p, const RaySensor& sensor, float s, float c) {
    // Clockwise rotation: at heading 0, forward is +Y; at 90, it is +X.
    const float ox = p.x + c * sensor.x + s * sensor.y;
    const float oy = p.y - s * sensor.x + c * sensor.y;
    const float dx = c * sensor.dx + s * sensor.dy;
    const float dy = -s * sensor.dx + c * sensor.dy;
    Hit nearest;
    for (std::size_t i = 0; i < edgeCount; ++i) {
        const auto& edge = edges[i];
        const float denominator = cross(dx, dy, edge.dx, edge.dy);
        if (std::abs(denominator) < 1.0e-6f) continue;
        const float ax = edge.x - ox, ay = edge.y - oy;
        const float t = cross(ax, ay, edge.dx, edge.dy) / denominator;
        const float u = cross(ax, ay, dx, dy) / denominator;
        if (t >= 0 && u >= 0 && u <= 1 && t < nearest.range) {
            nearest = {t, static_cast<int>(i)};
        }
    }
    return nearest;
}

void captureBaseline(std::uint32_t now) {
    lastLeft = chassis.drive_sensor_left();
    lastRight = chassis.drive_sensor_right();
    lastHeading = chassis.drive_imu_get();
    haveBaseline = validReading(lastLeft) && validReading(lastRight) && validReading(lastHeading);
    lastMotionMs = now;
}

void resetLocked(Pose pose, double positionSigma, double headingSigma) {
    comparison = Comparison{};
    comparison.encoder = comparison.localized = comparison.distance = pose;
    positionSigma = std::abs(positionSigma);
    headingSigma = std::abs(headingSigma);
    for (auto& p : particles) {
        // A symmetric prior also supports the user-specified corner origin.
        // Do not clamp particles to a wall: that biases the initial estimate.
        p = {static_cast<float>(pose.x + positionSigma * noise()),
             static_cast<float>(pose.y + positionSigma * noise()),
             wrap(static_cast<float>((pose.theta + headingSigma * noise()) * RAD)),
             1.0f / PARTICLE_COUNT};
    }
    captureBaseline(pros::millis());
    lastRangeMs = pros::millis() - RANGE_MS;
    sinceRange = 0;
    haveObservation = false;
    enabled = true;
    comparison.enabled = true;
}

bool predict(std::uint32_t now) {
    const double left = chassis.drive_sensor_left();
    const double right = chassis.drive_sensor_right();
    const double heading = chassis.drive_imu_get();
    if (!validReading(left) || !validReading(right) || !validReading(heading)) {
        haveBaseline = false;
        return false;
    }
    if (!haveBaseline) {
        captureBaseline(now);
        return false;
    }
    const float dl = left - lastLeft, dr = right - lastRight;
    const float turn = wrap(static_cast<float>((heading - lastHeading) * RAD));
    // Reject discontinuities (e.g. encoder reset), rather than moving the cloud.
    const float seconds = (now - lastMotionMs) * 0.001f;
    const bool discontinuity = std::abs(dl) > 100.0f * seconds + 0.5f ||
                               std::abs(dr) > 100.0f * seconds + 0.5f ||
                               std::abs(turn) > 15.0f * seconds + 0.02f;
    lastLeft = left;
    lastRight = right;
    lastHeading = heading;
    lastMotionMs = now;
    if (discontinuity) return false;

    const float distance = (dl + dr) * 0.5f;
    // Exact chord length for an arc. Avoid division by a tiny angle.
    const float chord = std::abs(turn) > 1.0e-4f
        ? distance * std::sin(turn * 0.5f) / (turn * 0.5f) : distance;
    const double middle = comparison.encoder.theta * RAD + turn * 0.5;
    comparison.encoder.x += chord * std::sin(middle);
    comparison.encoder.y += chord * std::cos(middle);
    comparison.encoder.theta = wrap(comparison.encoder.theta * RAD + turn) / RAD;

    const float travel = std::abs(distance);
    sinceRange += travel + std::abs(turn) * 6.0f;
    if (travel < 1.0e-6f && std::abs(turn) < 1.0e-6f) return true;
    // Variance is proportional to movement, so noise is independent of loop rate.
    const float forwardSigma = 0.10f * std::sqrt(travel);
    const float sideSigma = 0.04f * std::sqrt(travel) + 0.025f * std::sqrt(std::abs(turn));
    const float turnSigma = 0.012f * std::sqrt(std::abs(turn)) + 0.0015f * std::sqrt(travel);
    for (auto& p : particles) {
        const float rotation = turn + turnSigma * noise();
        const float angle = p.theta + rotation * 0.5f;
        const float s = std::sin(angle), c = std::cos(angle);
        const float forward = chord + forwardSigma * noise();
        const float sideways = sideSigma * noise();
        p.x += s * forward + c * sideways;
        p.y += c * forward - s * sideways;
        p.theta = wrap(p.theta + rotation);
    }
    return true;
}

unsigned readRanges() {
    unsigned count = 0;
    for (auto& sensor : sensors) {
        const auto mm = sensor.device->get();
        sensor.valid = mm > 0 && mm <= RANGE_MAX_MM;
        sensor.range = sensor.valid ? mm / 25.4f : 0;
        count += sensor.valid;
    }
    return count;
}

bool weightParticles() {
    float total = 0;
    for (auto& p : particles) {
        const float s = std::sin(p.theta), c = std::cos(p.theta);
        // Keep the supplied corner origin possible; reject centers outside field.
        float likelihood = p.x >= 0 && p.x <= FIELD_SIZE && p.y >= 0 && p.y <= FIELD_SIZE
            ? 1.0f : 0.0f;
        if (likelihood == 0) {
            p.weight = 0;
            continue;
        }
        for (const auto& sensor : sensors) {
            if (!sensor.valid) continue;
            const Hit hit = raycast(p, sensor, s, c);
            float score = OUTLIER_FLOOR;
            if (hit.edge >= 0) {
                const float error = (sensor.range - hit.range) / RANGE_SIGMA;
                // Robust Cauchy + random-return mixture: a blocked sensor cannot
                // zero every weight. No exp/log calls or probability underflow.
                score += (1.0f - OUTLIER_FLOOR) / (1.0f + error * error);
            }
            likelihood *= score;
        }
        p.weight *= likelihood;
        total += p.weight;
    }
    if (!(total > 1.0e-20f) || !std::isfinite(total)) {
        for (auto& p : particles) p.weight = 1.0f / PARTICLE_COUNT;
        comparison.effectiveParticles = PARTICLE_COUNT;
        return false;
    }
    float sumSquared = 0;
    for (auto& p : particles) {
        p.weight /= total;
        sumSquared += p.weight * p.weight;
    }
    comparison.effectiveParticles = 1.0f / sumSquared;
    return true;
}

void estimate() {
    double x = 0, y = 0, s = 0, c = 0;
    for (const auto& p : particles) {
        x += p.weight * p.x;
        y += p.weight * p.y;
        s += p.weight * std::sin(p.theta);
        c += p.weight * std::cos(p.theta);
    }
    comparison.localized = {x, y, std::atan2(s, c) / RAD};
    double variance = 0;
    for (const auto& p : particles) {
        variance += p.weight * ((p.x - x) * (p.x - x) + (p.y - y) * (p.y - y));
    }
    comparison.spreadInches = std::sqrt(variance);
}

void compareDistances() {
    // Solve measured wall equations independently of the encoder position.
    // Use the fused pose only to choose the visible wall and sensor orientation.
    const auto& pose = comparison.localized;
    const Particle p{static_cast<float>(pose.x), static_cast<float>(pose.y),
                     static_cast<float>(pose.theta * RAD), 1};
    const float s = std::sin(p.theta), c = std::cos(p.theta);
    double xx = 0, xy = 0, yy = 0, bx = 0, by = 0, residual = 0;
    unsigned used = 0;
    for (const auto& sensor : sensors) {
        if (!sensor.valid) continue;
        const auto hit = raycast(p, sensor, s, c);
        if (hit.edge < 0) continue;
        const double error = sensor.range - hit.range;
        residual += error * error;
        ++used;
        // Outlier readings still count in RMS, but not in the independent solve.
        if (std::abs(error) > 6.0 * RANGE_SIGMA) continue;
        const auto& edge = edges[hit.edge];
        const float length = std::hypot(edge.dx, edge.dy);
        if (length < 1.0e-6f) continue;
        const double nx = -edge.dy / length, ny = edge.dx / length;
        const double ox = c * sensor.x + s * sensor.y;
        const double oy = -s * sensor.x + c * sensor.y;
        const double dx = c * sensor.dx + s * sensor.dy;
        const double dy = -s * sensor.dx + c * sensor.dy;
        const double rhs = nx * (edge.x - ox - dx * sensor.range) +
                           ny * (edge.y - oy - dy * sensor.range);
        const double incidence = std::abs(nx * dx + ny * dy);
        const double w = incidence * incidence / (1.0 + error * error / (RANGE_SIGMA * RANGE_SIGMA));
        xx += w * nx * nx;
        xy += w * nx * ny;
        yy += w * ny * ny;
        bx += w * nx * rhs;
        by += w * ny * rhs;
    }
    comparison.residualInches = used ? std::sqrt(residual / used) : 0;
    const double determinant = xx * yy - xy * xy;
    comparison.distanceValid = determinant > 1.0e-5;
    if (comparison.distanceValid) {
        comparison.distance = {(bx * yy - xy * by) / determinant,
                               (xx * by - xy * bx) / determinant, pose.theta};
    }
}

void resample() {
    // Systematic resampling is O(N), with a single random draw and no sorting.
    std::size_t source = 0;
    float cumulative = particles[0].weight;
    const float offset = uniform() / PARTICLE_COUNT;
    for (std::size_t i = 0; i < PARTICLE_COUNT; ++i) {
        const float target = offset + static_cast<float>(i) / PARTICLE_COUNT;
        while (target > cumulative && source + 1 < PARTICLE_COUNT) {
            cumulative += particles[++source].weight;
        }
        scratch[i] = particles[source];
        // Small roughening preserves diversity after a stationary correction.
        scratch[i].x += 0.08f * noise();
        scratch[i].y += 0.08f * noise();
        scratch[i].theta = wrap(scratch[i].theta + 0.08f * RAD * noise());
        scratch[i].weight = 1.0f / PARTICLE_COUNT;
    }
    particles.swap(scratch);
}

void update() {
    if (!mutex.take(0)) return;
    if (!enabled) {
        mutex.give();
        return;
    }
    const auto began = pros::micros();
    const auto now = pros::millis();
    const bool motionValid = predict(now);
    bool corrected = false;
    bool weighted = false;
    if (motionValid && now - lastRangeMs >= RANGE_MS) {
        lastRangeMs = now;
        comparison.validSensors = readRanges();
        comparison.distanceValid = false;
        // Don't repeatedly multiply the same stationary observation into the
        // posterior. Still refresh comparison readings while standing still.
        if (comparison.validSensors && edgeCount &&
            (sinceRange >= 0.05f || !haveObservation)) {
            weighted = weightParticles();
            corrected = weighted;
            if (weighted) {
                sinceRange = 0;
                haveObservation = true;
            }
        }
        estimate();
        compareDistances();
    } else {
        estimate();
    }
    if (corrected && CORRECT_CHASSIS_XY) {
        // Encoder prediction reads hardware, never this corrected pose, so
        // position corrections cannot feed back as fictitious robot motion.
        chassis.odom_xy_set(comparison.localized.x, comparison.localized.y);
    }
    if (weighted && comparison.effectiveParticles < RESAMPLE_THRESHOLD) resample();
    comparison.updateMicros = pros::micros() - began;
    mutex.give();
}

void run(void*) {
    std::uint32_t wake = pros::millis();
    while (true) {
        update();
        pros::Task::delay_until(&wake, LOOP_MS);
    }
}
} // namespace

void start() {
    // Function-local initialization also makes repeated start() calls harmless.
    static const bool initialized = [] {
        if (localizationMap.empty()) initializeLocalizationMap();
        cacheGeometry();
        reset({0, 0, 0});
        return true;
    }();
    (void)initialized;
    static pros::Task task(run, nullptr, TASK_PRIORITY_DEFAULT - 1,
                           TASK_STACK_DEPTH_DEFAULT, "MCL 200");
}

void setEnabled(bool value) {
    if (value) ekf::setEnabled(false);
    mutex.take();
    enabled = value;
    comparison.enabled = value;
    if (value) captureBaseline(pros::millis());
    mutex.give();
}

void reset(Pose pose, double positionStdDev, double headingStdDev) {
    if (!std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.theta) ||
        !std::isfinite(positionStdDev) || !std::isfinite(headingStdDev)) return;
    ekf::setEnabled(false);
    mutex.take();
    resetLocked(pose, positionStdDev, headingStdDev);
    mutex.give();
}

Comparison getComparison() {
    mutex.take();
    const Comparison result = comparison;
    mutex.give();
    return result;
}

void printComparison(int firstLine) {
    const auto result = getComparison();
    char text[150];
    std::snprintf(text, sizeof(text), "Enc x %.1f y %.1f a %.1f", result.encoder.x, result.encoder.y, result.encoder.theta);
    ez::screen_print(text, firstLine);
    std::snprintf(text, sizeof(text), "MCL x %.1f y %.1f a %.1f", result.localized.x, result.localized.y, result.localized.theta);
    ez::screen_print(text, firstLine + 1);
    if (result.distanceValid) {
        std::snprintf(text, sizeof(text), "Dist x %.1f y %.1f", result.distance.x, result.distance.y);
    } else {
        std::snprintf(text, sizeof(text), "Dist: need 2 nonparallel walls");
    }
    ez::screen_print(text, firstLine + 2);
    std::snprintf(text, sizeof(text), "MCL-Enc dx %.1f dy %.1f", result.localized.x - result.encoder.x, result.localized.y - result.encoder.y);
    ez::screen_print(text, firstLine + 3);
    std::snprintf(text, sizeof(text), "Sensors %u ESS %.0f/200 RMS %.1f", result.validSensors, result.effectiveParticles, result.residualInches);
    ez::screen_print(text, firstLine + 4);
    std::snprintf(text, sizeof(text), "Spread %.1f in; update %lu us", result.spreadInches, static_cast<unsigned long>(result.updateMicros));
    ez::screen_print(text, firstLine + 5);
}
} // namespace mcl
