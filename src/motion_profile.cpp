#include "motion_profile.hpp"
#include "ramsete.hpp"
#include "subsystems.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <limits>

namespace motion {
namespace {
constexpr double PI = 3.14159265358979323846, RAD = PI / 180;
double wrap(double a) { return std::remainder(a, 2 * PI); }
double clamp(double a, double limit) { return std::max(-limit, std::min(a, limit)); }
bool positive(double value) { return std::isfinite(value) && value > 0; }
std::atomic<std::uint32_t> generation{0};
}
bool Path::build(std::initializer_list<Waypoint> points, Limits limits) {
    return build(points.begin(), points.size(), limits);
}
bool Path::build(const Waypoint* points, std::size_t pointCount, Limits limits) {
    count_ = 0; limits_ = limits;
    if (points == nullptr || pointCount < 2 || pointCount > CAPACITY || !positive(limits.maxVelocity) || !positive(limits.maxAcceleration) ||
        !positive(limits.maxWheelVelocity) || !positive(limits.maxWheelAcceleration) ||
        !positive(limits.maxLateralAcceleration) || !positive(limits.maxAngularVelocity) ||
        !positive(limits.trackWidth) || !positive(limits.sampleSpacing)) return false;
    for (std::size_t i = 0; i < pointCount; ++i) if (!std::isfinite(points[i].x) || !std::isfinite(points[i].y) || !std::isfinite(points[i].heading)) return false;
    auto fail = [this] { count_ = 0; return false; };
    const auto* begin = points;
    for (std::size_t segment = 0; segment + 1 < pointCount; ++segment) {
        const auto& a = begin[segment]; const auto& b = begin[segment + 1];
        const double chord = std::hypot(b.x - a.x, b.y - a.y);
        if (!(chord > .01)) return fail(); // Pure turns are not path segments.
        const double ax = chord * std::sin(a.heading * RAD), ay = chord * std::cos(a.heading * RAD);
        const double bx = chord * std::sin(b.heading * RAD), by = chord * std::cos(b.heading * RAD);
        const double polygon = 2 * chord / 3 + std::hypot(b.x - bx / 3 - a.x - ax / 3, b.y - by / 3 - a.y - ay / 3);
        const double required = std::ceil(polygon / limits.sampleSpacing);
        if (!std::isfinite(required) || required > CAPACITY - 1) return fail();
        const std::size_t steps = std::max<std::size_t>(2, static_cast<std::size_t>(required));
        if (count_ + steps + (segment == 0 ? 1 : 0) > CAPACITY) return fail();
        for (std::size_t i = segment == 0 ? 0 : 1; i <= steps; ++i) {
            const double t = static_cast<double>(i) / steps, t2 = t * t, t3 = t2 * t;
            const double h00 = 2*t3-3*t2+1, h10 = t3-2*t2+t, h01 = -2*t3+3*t2, h11 = t3-t2;
            const double x = h00*a.x+h10*ax+h01*b.x+h11*bx, y = h00*a.y+h10*ay+h01*b.y+h11*by;
            const double dx = (6*t2-6*t)*a.x+(3*t2-4*t+1)*ax+(-6*t2+6*t)*b.x+(3*t2-2*t)*bx;
            const double dy = (6*t2-6*t)*a.y+(3*t2-4*t+1)*ay+(-6*t2+6*t)*b.y+(3*t2-2*t)*by;
            const double ddx = (12*t-6)*a.x+(6*t-4)*ax+(-12*t+6)*b.x+(6*t-2)*bx;
            const double ddy = (12*t-6)*a.y+(6*t-4)*ay+(-12*t+6)*b.y+(6*t-2)*by;
            const double norm = std::hypot(dx, dy);
            if (!(norm > 1e-6)) return fail();
            const double curvature = (dy*ddx-dx*ddy)/(norm*norm*norm);
            double distance = count_ ? nodes_[count_-1].distance + std::hypot(x-nodes_[count_-1].x, y-nodes_[count_-1].y) : 0;
            if (count_ && distance - nodes_[count_-1].distance < 1e-7) return fail();
            double velocity = std::min(limits.maxVelocity, limits.maxWheelVelocity / (1+std::abs(curvature)*limits.trackWidth*.5));
            if (std::abs(curvature) > 1e-8) velocity = std::min({velocity,
                std::sqrt(limits.maxLateralAcceleration/std::abs(curvature)), limits.maxAngularVelocity/std::abs(curvature)});
            nodes_[count_++] = {x,y,std::atan2(dx,dy),curvature,distance,velocity,0};
        }
    }
    if (count_ < 2) return fail();
    // Use each interval's worst curvature at BOTH ends. This guarantees the
    // interpolated reference also respects wheel/yaw/lateral speed limits.
    for (std::size_t i=1; i<count_; ++i) {
        const double k=std::max(std::abs(nodes_[i-1].curvature),std::abs(nodes_[i].curvature));
        double cap=std::min(limits.maxVelocity,limits.maxWheelVelocity/(1+k*limits.trackWidth*.5));
        if (k>1e-8) cap=std::min({cap,std::sqrt(limits.maxLateralAcceleration/k),limits.maxAngularVelocity/k});
        nodes_[i-1].velocity=std::min(nodes_[i-1].velocity,cap);
        nodes_[i].velocity=std::min(nodes_[i].velocity,cap);
    }
    nodes_[0].velocity = nodes_[count_-1].velocity = 0;
    for (std::size_t i=1; i<count_; ++i) {
        const double ds = nodes_[i].distance-nodes_[i-1].distance;
        nodes_[i].velocity = std::min(nodes_[i].velocity, std::sqrt(nodes_[i-1].velocity*nodes_[i-1].velocity+2*limits.maxAcceleration*ds));
    }
    for (std::size_t i=count_-1; i>0; --i) {
        const double ds = nodes_[i].distance-nodes_[i-1].distance;
        nodes_[i-1].velocity = std::min(nodes_[i-1].velocity, std::sqrt(nodes_[i].velocity*nodes_[i].velocity+2*limits.maxAcceleration*ds));
    }
    // Curvature changes accelerate the individual wheels too. Bound both
    // endpoints of each interval's analytic wheel acceleration, then slow the
    // whole profile uniformly (acceleration scales with speed squared).
    double peakWheelAcceleration = 0;
    for (std::size_t i=1; i<count_; ++i) {
        const auto& a=nodes_[i-1]; const auto& b=nodes_[i];
        const double ds=b.distance-a.distance, acceleration=(b.velocity*b.velocity-a.velocity*a.velocity)/(2*ds);
        for (double sign : {-1.0,1.0}) {
            const double qa=1+sign*limits.trackWidth*.5*a.curvature, qb=1+sign*limits.trackWidth*.5*b.curvature;
            const double derivative=(qb-qa)/ds;
            peakWheelAcceleration=std::max({peakWheelAcceleration,
                std::abs(qa*acceleration+derivative*a.velocity*a.velocity),
                std::abs(qb*acceleration+derivative*b.velocity*b.velocity)});
        }
    }
    const double scale=peakWheelAcceleration>limits.maxWheelAcceleration ? std::sqrt(limits.maxWheelAcceleration/peakWheelAcceleration) : 1;
    for (auto& node : nodes_) { node.velocity *= scale; } // Fixed, bounded array.
    for (std::size_t i=1; i<count_; ++i) {
        const double sum=nodes_[i-1].velocity+nodes_[i].velocity;
        if (!(sum > 1e-8)) return fail();
        nodes_[i].time=nodes_[i-1].time+2*(nodes_[i].distance-nodes_[i-1].distance)/sum;
    }
    return std::isfinite(duration()) && duration() > 0 ? true : fail();
}
Sample Path::sample(double seconds) const {
    Sample out;
    if (!valid() || !std::isfinite(seconds)) return out;
    if (seconds >= duration()) {
        const auto& n=nodes_[count_-1]; out.pose={n.x,n.y,n.heading/RAD}; out.curvature=n.curvature; out.distance=n.distance; return out;
    }
    seconds=std::max(0.0,seconds);
    std::size_t low=0,high=count_-1;
    while (high-low>1) { const auto mid=(low+high)/2; if(nodes_[mid].time<=seconds) low=mid; else high=mid; }
    const auto& a=nodes_[low]; const auto& b=nodes_[high];
    const double dt=b.time-a.time, t=seconds-a.time, acceleration=(b.velocity-a.velocity)/dt;
    const double ds=a.velocity*t+.5*acceleration*t*t;
    const double u=std::max(0.0,std::min(1.0,ds/(b.distance-a.distance)));
    out.pose={a.x+(b.x-a.x)*u,a.y+(b.y-a.y)*u,(a.heading+wrap(b.heading-a.heading)*u)/RAD};
    out.velocity=a.velocity+acceleration*t; out.acceleration=acceleration;
    out.curvature=a.curvature+(b.curvature-a.curvature)*u; out.angularVelocity=out.curvature*out.velocity;
    out.distance=a.distance+ds;
    return out;
}
ChassisVelocity feedback(ekf::Pose actual, Sample reference) {
    // Preserve the original API and default tuning through one shared formula.
    return ramsete::feedback(actual, reference);
}

void cancel() { ++generation; chassis.drive_set(0,0); }
const char* resultName(Result r) {
    switch(r) {
        case Result::Settled:return "settled"; case Result::Timeout:return "timeout";
        case Result::Cancelled:return "cancelled"; case Result::InvalidPath:return "invalid path";
        default:return "localization unavailable";
    }
}
namespace {
TrackingCommand defaultController(const Path& path, ekf::Pose actual,
    const Sample& reference, double, void*) {
    return {feedback(actual, reference), reference.distance >= path.length()};
}
}
Result follow(const Path& path, Gains gains, std::uint32_t timeoutMs) {
    return followWithController(path, defaultController, nullptr, gains, timeoutMs);
}
Result followWithController(const Path& path, TrackingController controller,
    void* context, Gains gains, std::uint32_t timeoutMs) {
    const auto token=++generation;
    struct Stop {
        std::uint32_t token;
        ~Stop() { if (generation.load()==token) chassis.drive_set(0,0); }
    } stop{token};
    if (!path.valid() || controller == nullptr || !positive(gains.kV) || !std::isfinite(gains.kS) || gains.kS<0 ||
        !std::isfinite(gains.kA) || gains.kA<0 || !std::isfinite(gains.velocityKp) || gains.velocityKp<0 ||
        !positive(gains.positionTolerance) || !positive(gains.headingTolerance) ||
        !positive(gains.maxCommand) || gains.maxCommand>127 ||
        std::abs(path.limits().trackWidth-ekf::Config{}.trackWidth)>1e-6) return Result::InvalidPath;
    if (!timeoutMs) timeoutMs=static_cast<std::uint32_t>(std::min(600000.0,path.duration()*2000+3000));
    const auto began=pros::millis(); auto lastMs=began, wake=began;
    double lastLeft=chassis.drive_sensor_left(), lastRight=chassis.drive_sensor_right();
    double previousLeft=0,previousRight=0;
    std::uint32_t settled=0;
    chassis.drive_set(0,0); // Cancels EZ-Template PID before taking motor control.
    while (true) {
        const auto now=pros::millis();
        if (generation.load()!=token || (pros::competition::is_connected() &&
            (pros::competition::is_disabled() || !pros::competition::is_autonomous()))) return Result::Cancelled;
        if (now-began>=timeoutMs) return Result::Timeout;
        const auto estimate=ekf::getEstimate();
        if (!estimate.enabled || (now-estimate.updatedMs>200) ||
            (!estimate.healthy && now-began>200)) return Result::LocalizationUnavailable;
        if (!estimate.healthy) { pros::Task::delay_until(&wake,20); continue; }
        const double dt=std::max(.001,(now-lastMs)/1000.0);
        const double left=chassis.drive_sensor_left(),right=chassis.drive_sensor_right();
        if (!std::isfinite(left) || !std::isfinite(right) || std::abs(left)>1e8 || std::abs(right)>1e8 ||
            std::abs(left-lastLeft)>100*dt+.5 || std::abs(right-lastRight)>100*dt+.5) return Result::LocalizationUnavailable;
        const double leftVelocity=(left-lastLeft)/dt, rightVelocity=(right-lastRight)/dt;
        lastLeft=left; lastRight=right; lastMs=now;
        const double elapsed=(now-began)/1000.0;
        const auto desired=path.sample(elapsed);
        const auto tracking=controller(path,estimate.pose,desired,(leftVelocity+rightVelocity)*.5,context);
        auto command=tracking.velocity;
        if (!std::isfinite(command.forward) || !std::isfinite(command.angular)) return Result::InvalidPath;
        if (tracking.atEnd) {
            const auto goal=path.sample(path.duration()).pose;
            const double dx=goal.x-estimate.pose.x,dy=goal.y-estimate.pose.y,distance=std::hypot(dx,dy);
            const double headingError=wrap((goal.theta-estimate.pose.theta)*RAD);
            if (distance>gains.positionTolerance) {
                const double bearing=std::atan2(dx,dy), angle=wrap(bearing-estimate.pose.theta*RAD);
                command={std::min(6.0,2*distance)*std::max(0.0,std::cos(angle)),3*angle};
            } else command={0,3*headingError};
            if (distance<=gains.positionTolerance && std::abs(headingError)<=gains.headingTolerance*RAD &&
                std::abs(leftVelocity)<1 && std::abs(rightVelocity)<1) {
                if (!settled) settled=now;
                if (now-settled>=gains.settleMs) return Result::Settled;
            } else settled=0;
        }
        command.forward=clamp(command.forward,path.limits().maxVelocity);
        command.angular=clamp(command.angular,path.limits().maxAngularVelocity);
        const double half=path.limits().trackWidth*.5;
        double wantedLeft=command.forward+command.angular*half, wantedRight=command.forward-command.angular*half;
        const double peak=std::max(std::abs(wantedLeft),std::abs(wantedRight));
        if (peak>path.limits().maxWheelVelocity) {
            wantedLeft*=path.limits().maxWheelVelocity/peak; wantedRight*=path.limits().maxWheelVelocity/peak;
        }
        const double maxChange=path.limits().maxWheelAcceleration*dt;
        wantedLeft=previousLeft+clamp(wantedLeft-previousLeft,maxChange);
        wantedRight=previousRight+clamp(wantedRight-previousRight,maxChange);
        auto voltage=[&](double wanted,double previous,double measured) {
            const double ff=(std::abs(wanted)>.05 ? std::copysign(gains.kS,wanted) : 0)+
                gains.kV*wanted+gains.kA*(wanted-previous)/dt;
            return clamp(ff+gains.velocityKp*(wanted-measured),gains.maxCommand);
        };
        const double leftCommand=voltage(wantedLeft,previousLeft,leftVelocity);
        const double rightCommand=voltage(wantedRight,previousRight,rightVelocity);
        previousLeft=wantedLeft; previousRight=wantedRight;
        if (generation.load()!=token) return Result::Cancelled;
        chassis.drive_set(static_cast<int>(std::lround(leftCommand)),static_cast<int>(std::lround(rightCommand)));
        pros::Task::delay_until(&wake,20);
    }
}
} // namespace motion
