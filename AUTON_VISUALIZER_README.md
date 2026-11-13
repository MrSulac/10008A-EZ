# Autonomous visualizer and match position logging

Open `tools/auton_visualizer/index.html` in a browser. It works offline, without
installation, a server or network access. It currently lists all 18 registered
autons and supports EZ-Template, the shared profiler, Stanley and Ramsete.

The viewer includes field playback, a timeline scrubber, speed chart, adjustable
robot footprint, optional predicted sensor rays, JSON/PNG export and recorded-run
overlays. The displayed footprint defaults to 12×12 inches for illustration;
change it to match the robot. Predicted rays use the map and are not actual
sensor observations. The field map comes from `src/fieldmap.cpp`.

## Preview accuracy

- The three profile-based tests use the actual C++ `motion::Path` generator and
  time parameterization. No separate JavaScript curve implementation is used.
- EZ drive, turn and swing routines show **command geometry guides**. Their
  timings and swing arcs are estimates, not EZ PID simulations. Pursuit and
  boomerang previews connect requested waypoints; they do not reproduce EZ's
  internal generated pursuit curve or chaining dynamics.
- Conditional interference and offset-calibration routines are marked
  **Recorded run needed**. Arbitrary C++ conditions/loops are not executed or
  guessed by the source extractor. Other unsupported expressions are identified
  instead of inventing a route.

Actual recording below is independent of the motion controller and covers the
real executed branch of any auton. The position is a localization estimate,
not external ground-truth tracking.

## Saving position during a match

The new `auton_trace` module starts a low-priority background task during robot
initialization. The autonomous entry point names and begins a trace; driver
control begins a separate `Driver-Control` trace. Normal completion or disable
ends the current phase. Practice runs are recorded too while those modes run.

With an SD card installed, files are saved on the robot as:

```
/usd/match-<routine-or-phase>-<clock-ms>-<sequence>.csv
```

After the match, copy these files from the SD card and load them using **Replay
a match or import a path** in the viewer. Routine names automatically select the
matching planned route; driver-control files get a separate entry. The included
`examples/ramsete-simulated.csv` demonstrates the same logger and import format
using the host simulation. It is not a measured hardware run.

Position is sampled at about 50 Hz. CSV writes and approximately one-second
flushes happen only in the background task, never in the controller loop. Phase
ends close the file. This persists data during the match rather than waiting
until autonomous ends. The final buffered tail can be lost if power is removed;
card/filesystem latency can also affect durability. An SD-card hardware test is
still needed on the actual Brain.

The logger stores X/Y/heading, EZ odometry, left/right encoder positions and the
pose source. A healthy EKF estimate is preferred; otherwise it falls back to the
chassis odometry, so EZ, MCL and custom profilers that publish chassis odometry
can also be recorded. It does not automatically restore an old pose at startup.

Logging is bounded: a fixed 512-entry queue reserves space for phase boundaries.
If the card is too slow and the queue fills, position samples are dropped rather
than allocating memory or delaying motor output. `auton_trace::status()` exposes
SD availability, write errors, captured samples, dropped entries, completed files
and the last opened filename. Missing SD cards do not stop the robot. Insert a
working card before a match; writes may retry if a card becomes available.

The controller-independent API is in `include/auton_trace.hpp`:

```cpp
auton_trace::start();          // Already called in initialize().
auton_trace::begin("My Run"); // Already wrapped around selected autons.
// Any controller can run here.
auton_trace::end();
const auto logStatus = auton_trace::status();
```

## CSV/JSON compatibility

For other profiler exports, use this CSV schema:

```csv
t,x,y,theta
0.00,0,0,0
0.02,0,0.1,0
```

Time is seconds. X/Y default to inches, heading defaults to clockwise degrees
from +Y. Select meters/millimeters or radians in the import controls if needed.
`time_ms` is also accepted. Heading may be omitted and inferred from the path
geometry. JSON may be an array of samples or an object with `name`, `units`,
`headingUnit`, and `samples`. For example:

```json
{"name":"My Run","units":"inches","headingUnit":"degrees","samples":[{"t":0,"x":0,"y":0,"theta":0}]}
```

Imports use the browser's local FileReader. No file is uploaded to a server.
Recorded overlays can be compared against any selected planned route. Choose
**New planned path** for a standalone profiler reference. Invalid/nonfinite
values and backwards timestamps are rejected. Duplicate timestamps keep the
last pose. A partial final CSV row after interruption can be ignored while
retaining valid earlier samples. Field plots can show positions outside 0–144
inches; coordinate frames are not silently shifted or clamped.

## Refreshing after code changes

From the project directory:

```sh
python3 tools/auton_visualizer/refresh.py
```

Refreshing requires Python 3 and a C++17 host compiler (`c++`, or set `CXX`). It
reads selector registrations, safe literal/arithmetic arguments and literal
shared-Path builds inside auton functions. It compiles the production path math
with inert test hardware stubs; it never runs robot auton functions. Unsupported
custom construction can still be shown by importing its reference or match log.
The already-generated HTML needs no compiler to open.

To check everything:

```sh
SANITIZE=1 tests/ekf_profile/run.sh
node tools/auton_visualizer/test_core.js
python3 tools/auton_visualizer/test_generator.py
```

The full ARM build, controller regressions, CSV handoff/flushing/fallback tests,
viewer data tests and a browser replay check have passed. The browser check used
521 simulated logged poses over the production profile. Physical motion
accuracy, track-width calibration and real SD write performance remain untested.
