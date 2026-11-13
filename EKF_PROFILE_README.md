# Encoder/distance EKF and 2D motion profiling

**Ramsete Profile** exposes the existing default feedback with configurable gains;
see `RAMSETE_README.md`. Open `tools/auton_visualizer/index.html` for all auton
previews and match-log replay. SD position logging now covers autonomous and
driver control; see `AUTON_VISUALIZER_README.md`.

An additional Stanley follower is available as **Stanley Profile** in the selector.
See `STANLEY_README.md` for its API, gains and test. It shares the motor-control
loop and stop handling; **EKF 2D Profile** retains its original feedback law.

The robot now starts the EKF at the requested `(0, 0, 0)` instead of starting
MCL. The 200-particle MCL implementation remains available. Resetting or enabling
either estimator pauses the other before it takes ownership of position updates.

## Files

- `include/ekf.hpp`, `src/ekf.cpp`: filter math and the background hardware task.
- `include/motion_profile.hpp`, `src/motion_profile.cpp`: path generation,
  time parameterization, feedback and motor control.
- `include/autons/profile-test.hpp`, `src/autons/profile-test.cpp`: test auton.
- `tests/ekf_profile/`: repeatable host simulations, including the actual test
  auton, using the production estimator and controller code.

Existing source changes are limited to `src/main.cpp` (startup, reset handling,
selector, screen, cancellation) and `src/mcl.cpp` (mutual estimator exclusion).
Sensor wiring stays front 13, back 14, left 15, right 16.

## Before driving

`ekf::Config::trackWidth` in `include/ekf.hpp` is **12 inches as a temporary
starting value**, not a measurement of this robot. Set it to the effective
left/right drive width. The estimator and controller share this setting.
Measure effective width using wheel travel during a measured turn:

```
width = (left travel - right travel) / clockwise angle in radians
```

The robot's IMU can supply the measured angle during calibration; this EKF
itself uses drive encoders and distance sensors. Average several controlled
turns in both directions. Verify encoder signs: forward motion increases both;
a clockwise turn increases the left travel relative to the right travel.

Tune `motion::Gains` in `include/motion_profile.hpp` on the robot. The initial
`kV` comes from the configured 3.25-inch wheels and 360 wheel RPM. Static
friction, acceleration feedforward and velocity feedback are initial values.
Start with clear space and the short test route, then adjust them using actual
motion. Keep the existing distance-sensor offsets and map accurate.

## Test autonomous

Select **EKF 2D Profile** in the autonomous selector. The normal autonomous
entry point pauses the EKF, resets encoders/IMU/chassis odometry, then seeds the
EKF at `(0, 0, 0)`.

The final test route is:

```
(0, 0, 0 degrees)
    -> (0, 12, 0 degrees)
    -> (12, 24, 90 degrees)
    -> (24, 24, 90 degrees)
```

It moves forward, follows a right-hand curve, then finishes facing +X. Limits
are 18 in/s center speed, 24 in/s² center acceleration, 24 in/s wheel speed and
36 in/s² wheel acceleration. The entire follow call has a 14-second timeout.
It prints the result and final EKF pose, and leaves both drive outputs at zero.
The fourth blank Brain screen page displays EKF pose, standard deviations,
accepted/rejected ranges and update time while off the competition switch.

Coordinates are inches, +X right, +Y forward/up the field, with clockwise
headings from +Y. The origin is the field's bottom-left corner and poses refer
to the robot center. At the exact corner, some sensor origins extend outside
the field; missing/no-hit readings are ignored. Encoder prediction continues
until valid mapped wall observations become available.

## EKF details

The state is `[x, y, theta]`, with radians internally. Left/right encoder travel
predicts the exact differential-drive arc. The process Jacobians propagate a
fixed 3×3 covariance with independent wheel travel noise and translational slip.
There is no IMU fusion or matrix-library dependency.

Each range update chooses the nearest forward map intersection, including
occlusion, and uses an analytic range Jacobian with the sensor's mounting
offset. Grazing rays and innovations outside a 3-sigma gate are rejected.
Scalar updates use the Joseph covariance form and restore matrix symmetry.
Invalid sensor values, no-return 9999 and readings above 2000 mm are skipped.

Prediction runs every 20 ms; each distance sensor is read once every 60 ms.
The filter uses fixed-size matrices and performs no update-time allocations.
It exposes a thread-safe `ekf::getEstimate()` snapshot with pose, covariance,
health and timing. It also publishes chassis X/Y for legacy odometry/display;
it leaves EZ-Template's IMU heading and turn-controller targets alone. The
profile controller uses the complete EKF pose, including encoder/range heading.
Do not simultaneously run the old `resets::runLocalization()` correction task.

This is local estimation around a known pose. Large unexpected encoder jumps
are rejected, and the next valid read rebaselines the filter. Before intentionally
resetting encoders or changing frames, use the same sequence as autonomous:

```cpp
ekf::setEnabled(false);
// Reset chassis sensors and set chassis odometry in the desired field frame.
ekf::reset({0, 0, 0});
```

Use that sequence in the older offset-measurement/interference examples if you
run them with the EKF. Noise and track width require calibration; a wrong map,
large unmodeled slip or carrying the robot can require a reset to a known pose.

## Controller details and reuse

Paths are tangent-continuous cubic Hermite curves through `(x,y,heading)`
waypoints, sampled into a fixed 768-node buffer (about 43 KB). Store a `Path`
with static/global storage on the robot rather than on a small PROS task stack.
Paths must remain unchanged and alive while `follow()` runs. Pure turn-only
segments, invalid limits and paths exceeding capacity fail without driving.
Paths are forward travel; reverse path generation is not implemented.

Forward/backward speed passes enforce tangential acceleration and curvature
limits for wheel speed, angular velocity and lateral acceleration. A uniform
time scaling step also enforces individual wheel acceleration, including
changing curvature. Geometry is precomputed once; runtime sampling uses binary
search and constant-acceleration interpolation. The controller uses nonlinear
trajectory feedback, wheel velocity feedback, kS/kV/kA feedforward, bounded
commands and wheel-target acceleration limits. A terminal position/heading
controller handles final settling even when reference speed reaches zero.

```cpp
static motion::Path path;
if (path.build({{0, 0, 0}, {0, 12, 0}, {12, 24, 90}, {24, 24, 90}})) {
    motion::Result result = motion::follow(path);
}
```

`follow()` blocks the calling autonomous task and calls `drive_set()` to cancel
EZ-Template PID before controlling the motors. Don't run another motor controller
at the same time. Cancel, timeout, invalid paths, stale/unhealthy localization,
and competition disable all stop drive output. `disabled()` and `opcontrol()`
also call `motion::cancel()` to cover competition task termination. The profile
time limit bounds a stalled route; reaching a timed-out route is not reported
as successful settling.

## Validation

Full PROS ARM compilation/linking and host tests were run. The host suite checks
encoder arc prediction, the range Jacobian against finite differences, gating,
10,000 noisy covariance updates for symmetry/positive semidefiniteness, distance
correction, continuous profile limits, exclusive estimator switching, invalid
readings, motor/encoder/sensor closed-loop simulation of the actual test auton,
and cancel/stale/timeout/disable stops. The sanitizer run uses AddressSanitizer
and UndefinedBehaviorSanitizer.

Run from the project directory on a computer with a C++17 compiler:

```sh
tests/ekf_profile/run.sh
SANITIZE=1 tests/ekf_profile/run.sh
```

These are build and simulation checks. Physical accuracy, drive tuning,
effective track width and real task timing remain to be validated on the robot.
