# Ramsete controller

Select **Ramsete Profile** to run the same `(0,0,0)` → `(0,12,0)` →
`(12,24,90)` → `(24,24,90)` route as the other controller tests. It has a
14-second timeout and uses the EKF pose, shared motion limits and motor feedback.

The controller is isolated in `include/ramsete.hpp` and `src/ramsete.cpp`, with
the test in `src/autons/ramsete-test.cpp`. The original `motion::feedback()`
already contained this law; it now delegates to this module with the same default
gains. `motion::follow()` remains compatible. Stanley is unchanged.

```cpp
#include "ramsete.hpp"
static motion::Path path;
if (path.build({{0,0,0}, {0,12,0}, {12,24,90}, {24,24,90}})) {
    ramsete::Config config;
    config.b = 0.015;
    config.zeta = 0.7;
    const auto result = ramsete::follow(path, config);
}
```

Ramsete tracks a timed trajectory, using body-frame forward, lateral and heading
errors. The controller returns corrected center velocity and yaw rate:

```
k = 2*zeta*sqrt(omega_ref^2 + b*v_ref^2)
v = v_ref*cos(e_theta) + k*e_forward
omega = omega_ref + k*e_theta + b*v_ref*sinc(e_theta)*e_right
```

The field uses inches and clockwise degrees from +Y. Internal angles and yaw
rates use radians. `sinc` has a finite small-angle limit. The formulation follows
the standard Ramsete equations, expressed in this robot's coordinate convention;
see the [WPILib controller source](https://github.wpilib.org/allwpilib/docs/release/java/src-html/edu/wpi/first/math/controller/RamseteController.html).

`b` must be positive and is expressed in inverse-square inches with radian
angles; `zeta` must be strictly between 0 and 1. The defaults retain this
project's existing profile tuning. If converting a gain specified per square
meter, multiply it by `0.0254²`; a metric `b=2` becomes approximately `0.00129032`
here. Do not copy a metric gain directly into this inch-based controller.

Ramsete does not move toward a stationary reference when both reference speeds
are zero. The shared terminal controller handles final position/heading settling.
The shared loop also enforces output and wheel-target limits, detects stale EKF
state, and stops on timeout, disable or `motion::cancel()`.

The nominal robot build, host simulations and sanitizer checks pass, including
nondefault gains and initial lateral/heading errors. The actual selectable test
settles in about 10.4 seconds in the idealized simulation. Track width is still
the unmeasured 12-inch placeholder; tune it and the motor gains on the robot.
