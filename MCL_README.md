# 200-particle Monte Carlo localization

The project now starts the EKF by default; see `EKF_PROFILE_README.md`. MCL is
retained as an alternative. Enabling/resetting either estimator pauses the other.
The motion-profile test requires the EKF.

The implementation is in `src/mcl.cpp`, with its public interface in
`include/mcl.hpp`. The only existing source files changed are `src/main.cpp`
(startup, autonomous reset, comparison screen) and `src/bot/config.cpp` (ports).
The existing distance-reset solver is left unchanged and is not started by MCL.

## Hardware and coordinates

| Sensor | Smart port | Offset from robot center (inches) | Direction at heading 0 |
| --- | ---: | --- | --- |
| Front | 13 | (0, 5.75) | +Y |
| Back | 14 | (0, -5.75) | -Y |
| Left | 15 | (-5.75, 0) | -X |
| Right | 16 | (5.75, 0) | +X |

Connect the distance sensors to these ports. Motors use 1–6, the IMU uses 7,
the lift uses 8–10, the claw sensor uses 11, and the intake uses 12.

The requested start is `(0, 0, 0)`, with the field's bottom-left corner as the
origin. Coordinates are inches; +X points right, +Y points up the field, and
heading is clockwise from +Y. Robot and sensor poses use the robot center as
their reference. At the exact corner, negative sensor offsets extend outside
the perimeter, so some rays may have no valid wall return. Invalid readings
are ignored and the system continues predicting from encoders and the IMU.
Sensor offsets and the 144-inch field map should match the physical robot and
field for useful absolute corrections.

## Operation

`mcl::start()` runs one background task after chassis initialization. It starts
at `(0, 0, 0)` and keeps exactly 200 particles. It uses left/right drive encoder
travel in inches and IMU heading changes to predict their motion. It scores
particles by comparing measured distances with the nearest forward wall or
mapped obstacle. It uses the sensor mounting offsets and clockwise heading
convention directly, rather than the old reset solver's ray calculations.

The weighted particle mean corrects EZ-Template's X/Y position. The chassis
heading and turn-controller targets remain controlled by the IMU. MCL never
uses corrected chassis X/Y as encoder movement, so a wall correction cannot
feed back into its next prediction. Do not simultaneously run the old
`resets::runLocalization()` position-correction task.

`mcl::printComparison()` displays the following values if connected to a screen
page. The fourth blank page currently displays the EKF instead:

- **Enc**: encoder/IMU position with no distance corrections.
- **MCL**: fused particle position and heading.
- **Dist**: position solved from sensor ranges, using the fused heading and
  visible-wall associations. This is available only when at least two
  nonparallel wall constraints are valid; it is not an independent heading.
- **MCL-Enc**: the difference between fused and encoder-only X/Y.
- Valid sensor count, effective particle count before resampling, range RMS
  error, particle position spread, and the most recent update time.

The same values are available as a thread-safe copy from
`mcl::getComparison()`; encoder, localized, and distance positions share the
same coordinate frame. `distanceValid` marks whether the distance solution
exists. The last distance pose is retained when invalid; check that flag.

## Efficiency and tuning

All tuning constants are at the top of `src/mcl.cpp`.

- Fixed-size particle and scratch arrays; no per-update heap allocations.
- Motion prediction every 20 ms. Ranges are read at most every 50 ms
  (normally every 60 ms with the 20 ms task period).
- Each sensor is read once per range update, outside the particle loops.
- Readings of 0, errors, 9999/no-return, and readings above 2000 mm are ignored.
- Sensor geometry and map edges are cached at startup.
- Ray casts use the nearest positive segment intersection, including occlusion.
- A robust Cauchy/outlier-mixture likelihood avoids exponentials and logarithms.
- Linear-time systematic resampling runs only below 110 effective particles.
- Small resampling perturbations preserve particle diversity. Stationary
  measurements are refreshed for comparison but aren't repeatedly multiplied
  into the posterior until the robot moves again.
- The map cache holds up to 64 segments. A larger map disables range fusion
  instead of silently using a truncated map; raise `MAX_SEGMENTS` if needed.
- Set `CORRECT_CHASSIS_XY` to `false` for comparison-only operation.

This is localization around a known starting pose. It does not perform a
whole-field search or automatically recover from physically carrying the robot
to a distant location. Use `reset()` with the new known pose in that case.

## Resetting

The autonomous entry point now pauses and resets the EKF. When using MCL,
apply the sequence below around chassis resets. If another routine resets drive
encoders, the IMU, or the coordinate frame, use the same sequence:

```cpp
mcl::setEnabled(false);
// Reset chassis encoders/IMU/odometry here.
// Make the chassis pose and the MCL seed use the same field coordinates.
mcl::reset({0, 0, 0});  // Captures new baselines and resumes MCL.
```

Large unexpected encoder/IMU jumps are rejected as discontinuities, but the
explicit sequence is needed to handle small resets reliably. This also applies
to the existing offset-measurement and interference example routines if used
while localization is running. Do not call `mcl::reset()` every loop.

## Validation

- Full PROS ARM build (`make -j4 quick`) passed, including hot-image linking.
- Host simulations passed geometry, nearest-hit occlusion, clockwise turns,
  heading wrap, encoder-only fallback, millimeter conversion, the independent
  distance solution, encoder-drift correction without feedback, blocked and
  invalid sensors, parallel-wall rejection, reset handling, bounded sensor
  reads, and stationary-observation handling.
- In the drift simulation, 20% excess encoder travel produced 4.00 inches of
  encoder-only error; the fused estimate reduced that to approximately 1.08
  inches.
- AddressSanitizer and UndefinedBehaviorSanitizer runs passed, including 10,000
  measurement updates.

These are build and simulation results. On-robot accuracy, actual task timing,
and noise constants still need checking with the real sensors and field.
