#include "autons/stanley-test.hpp"
#include "stanley.hpp"
#include "subsystems.hpp"

#include <cstdio>

void stanley_profile_test() {
    // autonomous() has already reset chassis and EKF to (0,0,0).
    static motion::Path path; // Keep the path buffer off the PROS task stack.
    motion::Limits limits;
    limits.maxVelocity = 18;
    limits.maxAcceleration = 24;
    limits.maxWheelVelocity = 24;
    limits.maxWheelAcceleration = 36;
    // Same route and limits as EKF 2D Profile for a direct controller comparison.
    if (!path.build({{0,0,0}, {0,12,0}, {12,24,90}, {24,24,90}}, limits)) {
        motion::cancel();
        ez::screen_print("Stanley: path build failed", 1);
        return;
    }
    const auto result = stanley::follow(path, {}, {}, 14000);
    const auto final = ekf::getEstimate();
    char text[120];
    std::snprintf(text, sizeof(text), "Stanley %s: x %.1f y %.1f a %.1f",
        motion::resultName(result), final.pose.x, final.pose.y, final.pose.theta);
    ez::screen_print(text, 1);
    std::printf("%s\n", text);
}
