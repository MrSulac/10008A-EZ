#include "autons/profile-test.hpp"
#include "ekf.hpp"
#include "motion_profile.hpp"
#include "subsystems.hpp"

#include <cstdio>

void ekf_profile_test() {
    // autonomous() has already reset chassis and EKF to the requested (0,0,0).
    // Static storage avoids putting the fixed path buffer on a PROS task stack.
    static motion::Path path;
    motion::Limits limits;
    limits.maxVelocity=18;
    limits.maxAcceleration=24;
    limits.maxWheelVelocity=24;
    limits.maxWheelAcceleration=36;
    // Forward 12 inches, curve right, then finish facing +X at (24,24).
    if (!path.build({{0,0,0},{0,12,0},{12,24,90},{24,24,90}},limits)) {
        chassis.drive_set(0,0);
        ez::screen_print("EKF profile: path build failed",1);
        return;
    }
    const auto result=motion::follow(path, {}, 14000);
    const auto final=ekf::getEstimate();
    char text[120];
    std::snprintf(text,sizeof(text),"Profile %s: x %.1f y %.1f a %.1f",motion::resultName(result),final.pose.x,final.pose.y,final.pose.theta);
    ez::screen_print(text,1);
    std::printf("%s\n",text);
}
