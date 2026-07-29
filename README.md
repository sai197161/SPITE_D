# SPITE-D

Perception-driven dynamic validity certification for roadmap-based motion
planning.

A robot that plans with a **roadmap** (a precomputed graph of collision-free
configurations, with edges for the motions between them) can answer planning
queries quickly — but only while the world matches the one the roadmap was
built in. When obstacles move, some edges become unsafe, and finding out which
ones is expensive: it means re-checking robot geometry against obstacle
geometry many times per second.

This package extends [SPITE / RGG](https://github.com/parasollab/open-spite),
which handles obstacles that *jump* between discrete positions, to obstacles
that **move continuously and whose future motion is predicted**.

## How it works

The runtime loop has five stages:

1. **Perception** — a depth image is turned into tracked obstacle boxes
   (position, size, velocity, and an uncertainty estimate) using a U-map
   detector and a Kalman filter.
2. **Prediction** — each track is extrapolated forward over a time *horizon*.
   Uncertainty (σ) grows the further ahead you predict.
3. **Certification** — the predicted trajectory becomes two geometric
   approximations: an *over*-approximation (a box enclosing everywhere the
   obstacle might plausibly be, inflated by `k·σ`) and an *under*-approximation
   (spheres guaranteed to lie inside it, deflated by `k·σ`). Comparing these
   against the roadmap labels every edge:
   - **Green** — over-approximations don't touch ⇒ provably safe
   - **Red** — under-approximations do touch ⇒ provably blocked
   - **Gray** — neither ⇒ undecided
4. **Planning** — shortest path over the non-Red edges, replanned whenever the
   current path becomes blocked.
5. **Spans** *(the contribution)* — instead of redoing step 3 every frame, one
   prediction is **frozen** over its horizon. Each frame then costs only a
   *conformance check*: is the obstacle still inside the envelope its
   certification was computed against? While it is, every edge label stays
   valid for free. The expensive work reruns only when the obstacle breaks its
   prediction, the horizon runs out, or a new obstacle appears.

Spans can optionally be cut into `k` **time slices**, so slices the clock has
passed are dropped by bookkeeping alone — letting edges reopen *behind* a
departing obstacle without rebuilding anything.

## Repository layout

```
include/           headers, mirrored by src/
  common/          shared data types (no ROS, no Eigen)
  perception/      U-map depth detector, box tracker
  trajectory/      predictor interface, constant-velocity predictor,
                   Span + SpanPipeline (the contribution)
  spite/           ValidityServer: predictions -> geometry -> edge labels
  planner/         Dijkstra replanner, roadmap file format
src/nodes/         thin ROS 2 wrappers (built only under colcon)
tools/             offline roadmap builders, terminal ablation demo
tests/             unit + integration tests
gazebo/worlds/     simulation worlds for depth data collection
launch/            ROS 2 launch files
```

Everything except `src/nodes/` is plain C++17 with **no ROS dependency**, so
the whole pipeline builds and runs on a laptop without ROS. The build detects
which mode it is in automatically:

- **core mode** — plain CMake; libraries, offline tools, and tests.
- **ROS mode** — under `colcon`; adds ROS message types and node executables.

## Prerequisites

This package uses [open-spite](https://github.com/parasol-lab/open-spite) as a
library. **Clone it first** — you pass its path to CMake:

```bash
git clone https://github.com/parasol-lab/open-spite.git ~/open-spite
```

open-spite needs CGAL, Boost, GMP, MPFR, Eigen, FCL and nlohmann_json. OMPL is
needed for the roadmap builders, OpenCV for perception.

### Linux (Ubuntu, ROS 2 Jazzy)

```bash
sudo apt install libcgal-dev libompl-dev libeigen3-dev libfcl-dev \
                 nlohmann-json3-dev libopencv-dev
```

### macOS

Dependencies come from Conan. **CGAL 5.6 or newer is required** — 5.5.x does
not compile under recent Apple Clang.

```bash
brew install cmake ninja opencv ompl
pip install conan && conan profile detect

cd ~/open-spite
export CMAKE_POLICY_VERSION_MINIMUM=3.5     # CMake 4.x vs. older recipes
conan install . --output-folder=build --build=missing \
      -s build_type=Release -s compiler.cppstd=gnu17
```

## Building

### Core mode (no ROS)

```bash
cd ~/spited_ws/src/SPITE_D
export CMAKE_POLICY_VERSION_MINIMUM=3.5           # macOS only

cmake -B build-core -G Ninja -DCMAKE_BUILD_TYPE=Release \
      -DSPITE_D_OPEN_SPITE_DIR=$HOME/open-spite \
      -DCMAKE_TOOLCHAIN_FILE=$HOME/open-spite/build/conan_toolchain.cmake
                                                   # ^ macOS only, omit on Linux
cmake --build build-core
```

`SPITE_D_OPEN_SPITE_DIR` must point at open-spite's **repository root** — the
directory containing its `CMakeLists.txt`.

Targets are skipped gracefully when a dependency is missing: no OpenCV means no
perception library, no OMPL means no roadmap builders, no
`SPITE_D_OPEN_SPITE_DIR` means no certification code. **If an expected binary
is missing, search the configure output for `skipped`** — that is almost always
the reason.

### ROS mode (ROS 2 Jazzy)

```bash
cd ~/spited_ws
colcon build --packages-select spite_d \
      --cmake-args -DSPITE_D_OPEN_SPITE_DIR=$HOME/open-spite
source install/setup.bash
```

To avoid repeating the path, put it in `~/.colcon/defaults.yaml`:

```yaml
build:
  cmake-args:
    - -DSPITE_D_OPEN_SPITE_DIR=/home/YOU/open-spite
```

> `colcon build --cmake-clean-cache` erases **all** cached `-D` settings. If you
> use it, repeat every `-D` you need on the same command line, or the
> certification targets silently vanish from the build.

## Running the tests

```bash
cd ~/spited_ws/src/SPITE_D
ctest --test-dir build-core --output-on-failure
```

Every line should read `Passed`:

```
1/8 Test #1: test_open_spite ..................   Passed
2/8 Test #2: test_predictor ...................   Passed
...
100% tests passed, 0 tests failed out of 8
```

The tests are plain C++ programs built on `assert`, so **exit code 0 means
every assertion held**. There is no test framework and no partial credit: a
test either passes silently or aborts at the first failed assertion, printing
the file and line number. That line *is* the specification that was violated,
which usually makes it the fastest way to understand a regression.

Run one test alone while debugging:

```bash
./build-core/tests/test_span_pipeline && echo PASS
```

If `test_open_spite` shows **Not Run**, it just hasn't been compiled — it is
not in the default build target:

```bash
cmake --build build-core --target test_open_spite
```

### What each test verifies

| test | what it checks | why it matters |
| --- | --- | --- |
| `test_predictor` | A constant-velocity prediction lands where the arithmetic says, and uncertainty grows with lookahead. A zero growth rate reproduces the fixed-horizon scheme exactly. | This is the input contract for everything downstream; if it fails, no later result means anything. |
| `test_replanner` | Shortest path over a small graph; blocking one edge forces the detour; a fully blocked graph yields no path. | Confirms the planner actually honours edge validity, in both directions of travel. |
| `test_validity_server` | On a two-corridor roadmap: an obstacle crossing corridor A turns it **Red** while corridor B stays **Green**, the planner detours, and dropping the obstacle reopens the corridor. Also that obstacles vanishing from a frame stop blocking, and that many short-lived track IDs leave no residue. | The core certification loop — most geometry regressions surface here first. |
| `test_roadmap_roundtrip` | A roadmap written to disk and read back yields identical graph data *and identical edge labels*. | The offline builder and the runtime are separate programs; this is the guarantee they agree. |
| `test_perception` | The tracker runs against synthetically rendered depth images of a moving box and must recover position within 15 cm and velocity within 0.15 m/s while holding one stable track ID; two boxes must produce two distinct IDs. | Ground truth is known exactly here, which real sensor data can't offer. The tolerances reflect the U-map's depth quantization, not sloppiness. |
| `test_span` | Envelope arithmetic, conformance decisions right at the boundary, horizon expiry, and trajectory slicing (including that neighbouring slices share a sample, so no gap opens between them). | The mathematical core of the contribution. |
| `test_span_pipeline` | Over 30 frames with a conforming obstacle the expensive geometry path runs **twice**, not 30 times — while edge labels stay identical to the frame-by-frame result. An obstacle that deviates is caught and triggers a rebuild; with `k=4` slices a corridor reopens behind the obstacle using one geometry build and three expiries. | The contribution's central claim — cheaper *and* equally correct — written as assertions. |
| `test_open_spite` | The upstream library's own suite. | Separates "we broke something" from "the library changed under us". |

## Offline tools

A roadmap is built once, saved, and reused at runtime. Both builders write
`roadmap_graph.txt` (vertices, edges, costs) and `roadmap_geoms.txt` (the
geometric approximations used for certification).

```bash
# Free-flying box robot in 3D
./build-core/build_roadmap --out ~/rm --grow 1.0

# UR5 arm: 6-DOF joint-space roadmap with per-link geometry
./build-core/build_roadmap_ur5 --out ~/rm_ur5 --grow 1.0
```

`--grow` is how many seconds to spend sampling — longer gives a denser
roadmap. Static obstacles default to two pillars and can be overridden with
`--obstacles FILE`, one axis-aligned box per line as
`xmin ymin zmin xmax ymax zmax`.

### The ablation demo

The quickest way to see what this package does. It drives a simulated obstacle
across a planned path twice — once certifying every frame, once with spans —
and compares them:

```bash
./build-core/demo_dynamic --roadmap ~/rm
```

```
mode         frames geom-passes  expiries  replans     update ms mean/med/max
baseline         40          40         0        1        1.385 /  1.423 /  4.008
spans            40           3         0        1        0.141 /  0.000 /  2.057

span speedup: 9.8x mean update (conforming frames are ~free: median 0.0001 ms),
              40 -> 3 expensive geometry passes
identical replan behavior in both modes (same path sizes every frame)
```

**Reading the output:**

- **geom-passes** — how many times the expensive work (build obstacle
  geometry, query the roadmap) actually ran. Baseline does it every frame by
  definition; spans should show a small number. *This is the headline result.*
- **update ms** — wall-clock per frame. The span **median** is essentially zero
  because a conforming frame is a handful of floating-point comparisons. The
  **max** is similar for both, because one span rebuild costs about the same as
  one baseline frame — spans reduce average cost, not worst case.
- **replans** — how many times the path had to be recomputed.
- **The last line is the correctness check.** Both modes must produce the same
  path on every frame. If it instead prints `NOTE: modes diverged`, spans
  changed a planning decision and something is wrong — the speedup means
  nothing without this line.

Variations:

```bash
./build-core/demo_dynamic --roadmap ~/rm --trace          # per-frame table
./build-core/demo_dynamic --roadmap ~/rm --slices 4       # time-sliced spans
./build-core/demo_dynamic --roadmap ~/rm --mode baseline  # single mode
```

With `--slices 4` expect *more* geometry passes but a non-zero **expiries**
count: slicing costs more per rebuild and buys the ability to release the
obstacle's past for free. That trade-off is what the `k` parameter controls.

`--dump-json FILE` writes per-frame state for external plotting;

## Running the full pipeline (ROS 2)

```bash
ros2 launch spite_d pipeline.launch.py \
    world:=depth_single_cross \
    roadmap_graph:=$HOME/rm/roadmap_graph.txt \
    roadmap_geoms:=$HOME/rm/roadmap_geoms.txt
```

Starts Gazebo, the Gazebo↔ROS bridge, and the three nodes. Add
`headless:=false` to open the Gazebo window; the default runs without a
display, which is what a remote machine needs.

Worlds in `gazebo/worlds/`: `depth_single_cross` has one actor walking across
the camera's view at constant speed (clean ground truth); `depth_multi_cross`
has three, including two that cross each other in the image.

Check data is flowing, from a second terminal:

```bash
source install/setup.bash          # required in EVERY terminal
ros2 topic hz   /camera/depth_image                   # sensor data arriving
ros2 topic echo /spite_d_perception/obstacles --once  # obstacles detected
ros2 topic echo /spite_d_validity/path --once         # planned path
```

If `ros2 topic echo` says *"message type is invalid"*, that terminal hasn't
sourced `install/setup.bash`.

`tools/setup_remote_gui.sh` sets up browser-based access to Gazebo and RViz on
a machine with no monitor.

## Status and known gaps

Validated: the perception → prediction → certification → replanning pipeline
runs end to end in Gazebo, and the span mechanism is verified by the test suite
and measured on both a free-flying-robot roadmap and a UR5 arm roadmap.

Open items, roughly in priority order:

1. **Spans are not yet used by the ROS node** — `validity_node` still
   recertifies every frame. Span code is exercised by the tests and the offline
   demo only.
2. **Gray edges** are treated as blocked — safe but conservative. Resolving
   them with a fine-grained collision check is unimplemented.
3. **Curved and multi-modal predictions** — spans assume a locally straight
   prediction; curved motion makes the single enclosing box loose.
4. **Only constant-velocity prediction exists.** Better predictors plug into
   the same `Predictor` interface.
5. **Tracker robustness** — no continuity filter, so track IDs churn on real
   data; obstacle boxes are axis-aligned rather than oriented.
