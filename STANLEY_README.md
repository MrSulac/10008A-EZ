# Stanley path controller

The Stanley controller is an additional path follower. The original 2D profile
controller and EKF remain available. Select **Stanley Profile** in the autonomous
selector to compare it with **EKF 2D Profile** on the same route and limits.

## Files and usage

- `include/stanley.hpp`, `src/stanley.cpp`: Stanley feedback and path projection.
- `include/autons/stanley-test.hpp`, `src/autons/stanley-test.cpp`: selectable test.
- `motion::followWithController()`: small shared-controller entry point in the
  existing motion module. Motor output, health checks, limits, settling and
  cancellation are shared with the original follower.

```cpp
#include "stanley.hpp"

static motion::Path path;  // Keep the path buffer off the PROS task stack.
if (path.build({{0,0,0}, {0,12,0}, {12,24,90}, {24,24,90}})) {
    const auto result = stanley::follow(path);
}
```

The existing autonomous entry point resets the EKF and robot to `(0,0,0)`.
The test drives forward, curves right and finishes near `(24,24,90 degrees)`.
It uses the same 18 in/s center-speed, 24 in/s² center-acceleration, 24 in/s
wheel-speed and 36 in/s² wheel-acceleration settings as the profile test, plus
a 14-second timeout. Final settling uses the shared 0.75-inch position and
3-degree heading tolerances. A timeout is reported separately from settling.

As with the other controller, poses are robot-center coordinates in inches,
with +X right, +Y forward/up the field and headings clockwise from +Y. The
controller reads EKF pose; that estimator fuses drive encoders and distances.
The drive width remains the shared **12-inch temporary value** until measured.

## Control law

The original Stanley law combines heading error with an arctangent of lateral
error divided by speed. This implementation adapts that structure to a tank
robot, which controls yaw rate through its two wheels:

```
steering = headingError + atan2(crossTrackGain * crossTrackError,
                               abs(measuredForwardSpeed) + softeningSpeed)
turnRate = forwardSpeed * curvature + headingGain * wrap(steering)
```

The nearest path segment supplies signed lateral error, tangent heading and
curvature. Positive turn rate is clockwise. At the default zero control-point
offset, the tracking point is the robot's drive-axle/pose center. This is a
Stanley-style differential-drive adaptation, rather than a bicycle-model
steering-servo command or a claim of the original vehicle's stability guarantees.

The algorithm reference is section 9.2 of Thrun et al.,
[Stanley: The Robot that Won the DARPA Grand Challenge](https://robots.stanford.edu/papers/thrun.stanley05.pdf).
Curvature feedforward and heading-to-turn-rate conversion above are the
adaptation used here.

## Profile and runtime behavior

The follower projects onto line segments between the existing Hermite path
samples. It makes one full nearest-segment search initially, then searches a
bounded forward window with a small backward allowance. Stored arc-length
progress never decreases. Start near the intended beginning of the path;
self-intersecting paths can have ambiguous nearest points at initialization.

Speed follows the profile envelope at actual projected progress, rather than
elapsed trajectory time. A small launch target leaves the profile's zero-speed
start; the shared wheel acceleration limiter ramps motor targets. Heading
misalignment reduces forward speed, and softening keeps steering finite at rest.
Curvature limits cap the speed target. The shared loop additionally bounds
forward/yaw requests, individual wheel speeds/target accelerations and motor
commands. Feedback and physical acceleration depend on the drive tuning; this
is not a promise that the physical robot follows the nominal profile exactly.

Once center progress reaches the endpoint region, the existing terminal
controller settles position and heading. Spatial completion prevents a slow
robot from skipping the remainder of the curve just because nominal profile
time has elapsed. The default overall timeout remains twice nominal duration
plus three seconds; the selectable test overrides it to 14 seconds.

`motion::cancel()` stops either follower. Competition disable, entry to driver
control, stale/unhealthy EKF, encoder discontinuities, invalid configuration and
timeout also stop output. Use one motor controller at a time; don't mutate the
path while following it. Forward paths are supported; reverse paths are not.

All Stanley state is small and local to each follow call. Projection and feedback
perform no heap allocations, and the EKF/sensor task is reused without creating
another localization task or rereading all distance sensors in this controller.

## Tuning

`stanley::Config` in `include/stanley.hpp` provides:

| Setting | Default | Effect |
| --- | ---: | --- |
| `crossTrackGain` | 2.0 /s | Strength of lateral correction |
| `headingGain` | 3.0 /s | Heading/steering error to yaw rate |
| `softeningSpeed` | 3.0 in/s | Reduces abrupt steering at low speed |
| `controlPointOffset` | 0 in | Tracking point ahead of the pose center |
| `searchAhead` | 12 in | Forward projection window after initialization |
| `launchSpeed` | 3.0 in/s | Startup target over the first inch |

For lateral oscillation, reduce cross-track gain or increase softening. For slow
heading response, adjust heading gain. Keep `searchAhead` greater than the
control-point offset. Shared `motion::Gains` still controls motor feedforward,
velocity feedback and endpoint tolerance. Measure effective track width and
tune the shared drive gains before increasing speed.

## Verification

The full PROS ARM build and host suite are checked with the new controller.
The host suite covers the actual selectable Stanley auton with simulated motors,
encoders and distance sensors; left/right steering signs; wrapped headings;
rest/curvature behavior; recovery from a 2-inch lateral and 8-degree heading
offset; monotonic progress; invalid configuration; and cancel, stale localization,
timeout and competition-disable stops. Existing EKF and original follower tests
are retained. AddressSanitizer and UndefinedBehaviorSanitizer are also run.

From the project directory:

```sh
tests/ekf_profile/run.sh
SANITIZE=1 tests/ekf_profile/run.sh
```

These checks use an idealized simulated robot. Real hardware timing, effective
track width, sensor geometry/noise and Stanley gains still need on-robot tuning.
