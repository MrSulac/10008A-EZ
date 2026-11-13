#include "ekf.hpp"
#include "bot/config.hpp"
#include "fieldmap.hpp"
#include "mcl.hpp"
#include "subsystems.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

namespace ekf {
namespace {
constexpr double PI = 3.14159265358979323846;
constexpr double RAD = PI / 180.0;
constexpr std::uint32_t LOOP_MS = 20, RANGE_MS = 60;
// All geometry/noise tuning is here and in Config; no matrix library needed.
const Config ROBOT_CONFIG{};
double wrap(double a) { return std::remainder(a, 2 * PI); }
bool finitePose(Pose p) { return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.theta); }
}
Filter::Filter(Config config) : config_(config) { reset({}); }
bool Filter::reset(Pose pose, double positionStdDev, double headingStdDev) {
    if (!finitePose(pose) || !std::isfinite(positionStdDev) || !std::isfinite(headingStdDev) ||
        !(config_.trackWidth > 0) || !std::isfinite(config_.trackWidth) ||
        !(config_.wheelVariancePerInch >= 0) || !std::isfinite(config_.wheelVariancePerInch) ||
        !(config_.slipVariancePerInch >= 0) || !std::isfinite(config_.slipVariancePerInch) ||
        !(config_.rangeStdDev > 0) || !std::isfinite(config_.rangeStdDev) ||
        !(config_.gateSquared > 0) || !std::isfinite(config_.gateSquared)) return false;
    state_ = {pose.x, pose.y, wrap(pose.theta * RAD)};
    covariance_ = {};
    covariance_[0][0] = covariance_[1][1] = positionStdDev * positionStdDev;
    covariance_[2][2] = headingStdDev * headingStdDev * RAD * RAD;
    return true;
}
Pose Filter::pose() const { return {state_[0], state_[1], state_[2] / RAD}; }
bool Filter::predict(double dl, double dr) {
    if (!std::isfinite(dl) || !std::isfinite(dr) || !(config_.trackWidth > 0)) return false;
    const double d = (dl + dr) * .5, a = (dl - dr) / config_.trackWidth;
    const double half = a * .5;
    const double sinc = std::abs(half) < 1e-5 ? 1 - half * half / 6 : std::sin(half) / half;
    const double dsincDa = std::abs(half) < 1e-5 ? -a / 12 :
        .5 * (half * std::cos(half) - std::sin(half)) / (half * half);
    const double middle = state_[2] + half, s = std::sin(middle), c = std::cos(middle);
    const double chord = d * sinc;
    Matrix f{{{1, 0, chord * c}, {0, 1, -chord * s}, {0, 0, 1}}};
    // Jacobian with respect to independent left/right wheel displacements.
    const double gx = d * dsincDa * s + chord * c * .5;
    const double gy = d * dsincDa * c - chord * s * .5;
    double g[3][2] = {{.5 * sinc * s + gx / config_.trackWidth, .5 * sinc * s - gx / config_.trackWidth},
                      {.5 * sinc * c + gy / config_.trackWidth, .5 * sinc * c - gy / config_.trackWidth},
                      {1 / config_.trackWidth, -1 / config_.trackWidth}};
    const double q[2] = {config_.wheelVariancePerInch * std::abs(dl),
                         config_.wheelVariancePerInch * std::abs(dr)};
    Matrix fp{}, next{};
    for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j)
        for (int k = 0; k < 3; ++k) fp[i][j] += f[i][k] * covariance_[k][j];
    for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) {
        for (int k = 0; k < 3; ++k) next[i][j] += fp[i][k] * f[j][k];
        for (int k = 0; k < 2; ++k) next[i][j] += g[i][k] * q[k] * g[j][k];
    }
    const double slip = config_.slipVariancePerInch * (std::abs(dl) + std::abs(dr)) * .5;
    next[0][0] += slip; next[1][1] += slip;
    covariance_ = next;
    state_[0] += chord * s; state_[1] += chord * c; state_[2] = wrap(state_[2] + a);
    return finitePose(pose());
}
RangePrediction Filter::rangePrediction(SensorMount sensor, Wall wall) const {
    RangePrediction result;
    const double t = state_[2], s = std::sin(t), c = std::cos(t);
    const double ox = c * sensor.x + s * sensor.y, oy = -s * sensor.x + c * sensor.y;
    const double dx = std::sin(t + sensor.angle * RAD), dy = std::cos(t + sensor.angle * RAD);
    const double ex = wall.bx - wall.ax, ey = wall.by - wall.ay, length = std::hypot(ex, ey);
    if (!(length > 1e-6)) return result;
    const double nx = -ey / length, ny = ex / length;
    const double denom = nx * dx + ny * dy;
    // Avoid grazing rays and unstable Jacobians near tangent incidence.
    if (std::abs(denom) < .15) return result;
    const double distance = (nx * (wall.ax - state_[0] - ox) + ny * (wall.ay - state_[1] - oy)) / denom;
    const double hx = state_[0] + ox + distance * dx, hy = state_[1] + oy + distance * dy;
    const double u = ((hx - wall.ax) * ex + (hy - wall.ay) * ey) / (length * length);
    if (!(distance > 0) || u < 0 || u > 1 || !std::isfinite(distance)) return result;
    result.valid = true; result.distance = distance;
    result.jacobian = {-nx / denom, -ny / denom,
        (-nx * oy + ny * ox - distance * (nx * dy - ny * dx)) / denom};
    return result;
}
bool Filter::correctRange(double inches, SensorMount sensor, Wall wall) {
    if (!(inches > 0) || !std::isfinite(inches)) return false;
    const auto predicted = rangePrediction(sensor, wall);
    if (!predicted.valid) return false;
    const auto& h = predicted.jacobian;
    const double innovation = inches - predicted.distance, r = config_.rangeStdDev * config_.rangeStdDev;
    std::array<double, 3> ph{}, gain{};
    for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) ph[i] += covariance_[i][j] * h[j];
    double variance = r;
    for (int i = 0; i < 3; ++i) variance += h[i] * ph[i];
    if (!(variance > 0) || !std::isfinite(variance) || innovation * innovation > config_.gateSquared * variance) return false;
    auto nextState = state_;
    Matrix a{}, ap{}, next{};
    for (int i = 0; i < 3; ++i) {
        gain[i] = ph[i] / variance;
        nextState[i] += gain[i] * innovation;
        for (int j = 0; j < 3; ++j) a[i][j] = (i == j ? 1.0 : 0.0) - gain[i] * h[j];
    }
    // Joseph form P=(I-KH)P(I-KH)' + KRK' preserves covariance PSD.
    for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j)
        for (int k = 0; k < 3; ++k) ap[i][j] += a[i][k] * covariance_[k][j];
    for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) {
        for (int k = 0; k < 3; ++k) next[i][j] += ap[i][k] * a[j][k];
        next[i][j] += gain[i] * r * gain[j];
    }
    for (int i = 0; i < 3; ++i) {
        if (!std::isfinite(nextState[i]) || !std::isfinite(next[i][i]) || next[i][i] < 0) return false;
        for (int j = i + 1; j < 3; ++j) next[i][j] = next[j][i] = .5 * (next[i][j] + next[j][i]);
    }
    nextState[2] = wrap(nextState[2]); state_ = nextState; covariance_ = next;
    return true;
}
namespace {
Filter filter(ROBOT_CONFIG);
pros::Mutex mutex;
Estimate estimate;
bool enabled = false, haveBaseline = false;
double lastLeft = 0, lastRight = 0;
std::uint32_t lastMs = 0, lastRanges = 0;
void baseline() {
    lastLeft = chassis.drive_sensor_left(); lastRight = chassis.drive_sensor_right();
    haveBaseline = std::isfinite(lastLeft) && std::isfinite(lastRight) &&
                   std::abs(lastLeft) < 1e8 && std::abs(lastRight) < 1e8;
    lastMs = pros::millis();
}
void update() {
    if (!mutex.take(0)) return;
    if (!enabled) { mutex.give(); return; }
    const auto began = pros::micros(); const auto now = pros::millis();
    const double left = chassis.drive_sensor_left(), right = chassis.drive_sensor_right();
    estimate.healthy = false;
    if (std::isfinite(left) && std::isfinite(right) && std::abs(left) < 1e8 && std::abs(right) < 1e8) {
        const double dl = left - lastLeft, dr = right - lastRight;
        const double limit = 100.0 * (now - lastMs) / 1000.0 + .5;
        if (haveBaseline && std::abs(dl) <= limit && std::abs(dr) <= limit) {
            estimate.healthy = filter.predict(dl, dr);
        }
        lastLeft = left; lastRight = right; lastMs = now; haveBaseline = true;
    } else { haveBaseline = false; }
    if (estimate.healthy && now - lastRanges >= RANGE_MS) {
        lastRanges = now;
        estimate.acceptedRanges = estimate.rejectedRanges = 0;
        Sensor* sensors[] = {&frontSensor, &backSensor, &leftSensor, &rightSensor};
        for (const auto sensor : sensors) {
            const auto mm = sensor->distanceSensor.get();
            if (mm <= 0 || mm > 2000) { ++estimate.rejectedRanges; continue; }
            const SensorMount mount{sensor->offset.x, sensor->offset.y, sensor->angle};
            // Nearest actual intersection, including grazing occluders. A nearer
            // grazing wall must not expose a farther, easier-to-match wall.
            double nearest = std::numeric_limits<double>::infinity();
            const Segment* match = nullptr;
            const auto pose = filter.pose(); const double t = pose.theta * RAD;
            const double ox = pose.x + std::cos(t) * mount.x + std::sin(t) * mount.y;
            const double oy = pose.y - std::sin(t) * mount.x + std::cos(t) * mount.y;
            const double dx = std::sin(t + mount.angle * RAD), dy = std::cos(t + mount.angle * RAD);
            for (const auto& segment : localizationMap) {
                const double ex = segment.B.x - segment.A.x, ey = segment.B.y - segment.A.y;
                const double denominator = dx * ey - dy * ex;
                if (std::abs(denominator) < 1e-8) continue;
                const double ax = segment.A.x - ox, ay = segment.A.y - oy;
                const double distance = (ax * ey - ay * ex) / denominator;
                const double u = (ax * dy - ay * dx) / denominator;
                if (distance > 0 && u >= 0 && u <= 1 && distance < nearest) { nearest = distance; match = &segment; }
            }
            if (match && filter.correctRange(mm / 25.4, mount,
                {match->A.x, match->A.y, match->B.x, match->B.y})) ++estimate.acceptedRanges;
            else ++estimate.rejectedRanges;
        }
    }
    estimate.pose = filter.pose(); estimate.covariance = filter.covariance();
    estimate.updatedMs = now; estimate.updateMicros = pros::micros() - began;
    // This is a display/legacy-odometry correction only; prediction reads raw
    // encoder inches and the profile controller reads the complete EKF pose.
    if (estimate.healthy) chassis.odom_xy_set(estimate.pose.x, estimate.pose.y);
    mutex.give();
}
void run(void*) {
    auto wake = pros::millis();
    while (true) { update(); pros::Task::delay_until(&wake, LOOP_MS); }
}
}
void start() {
    static const bool once = [] { if (localizationMap.empty()) initializeLocalizationMap(); reset({}); return true; }();
    (void)once;
    static pros::Task task(run, nullptr, TASK_PRIORITY_DEFAULT - 1, TASK_STACK_DEPTH_DEFAULT, "Encoder Range EKF");
}
void setEnabled(bool value) {
    if (value) mcl::setEnabled(false);
    mutex.take(); enabled = value; estimate.enabled = value;
    if (value) baseline(); else estimate.healthy = false;
    mutex.give();
}
void reset(Pose pose) {
    if (!finitePose(pose)) return;
    mcl::setEnabled(false);
    mutex.take();
    if (filter.reset(pose)) {
        baseline(); lastRanges = pros::millis() - RANGE_MS;
        enabled = true; estimate = {}; estimate.enabled = true;
        estimate.pose = filter.pose(); estimate.covariance = filter.covariance();
        estimate.updatedMs = pros::millis();
    }
    mutex.give();
}
Estimate getEstimate() { mutex.take(); const auto copy = estimate; mutex.give(); return copy; }
void printEstimate(int line) {
    const auto e = getEstimate(); char text[130];
    std::snprintf(text, sizeof(text), "EKF x %.1f y %.1f a %.1f", e.pose.x, e.pose.y, e.pose.theta); ez::screen_print(text, line);
    std::snprintf(text, sizeof(text), "SD x %.2f y %.2f a %.1f", std::sqrt(e.covariance[0][0]), std::sqrt(e.covariance[1][1]), std::sqrt(e.covariance[2][2]) / RAD); ez::screen_print(text, line + 1);
    std::snprintf(text, sizeof(text), "Ranges used %u rejected %u", e.acceptedRanges, e.rejectedRanges); ez::screen_print(text, line + 2);
    std::snprintf(text, sizeof(text), "EKF %s; update %lu us", e.enabled && e.healthy ? "ready" : "paused/not ready", static_cast<unsigned long>(e.updateMicros)); ez::screen_print(text, line + 3);
}
} // namespace ekf
