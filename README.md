# SPITE-D

Perception-driven dynamic validity certification for roadmap-based motion
planning. Extends [SPITE / RGG](https://parasollab.web.illinois.edu/research/spite/)
from discrete obstacle relocations to continuously moving obstacles with
predicted trajectories.

An RGB-D front end tracks obstacles; a predictor extrapolates each track over
a horizon; the predicted trajectory is turned into SPITE certification
geometry (over-approximation inflated by `+kσ`, under-approximation deflated
by `−kσ`) and queried against the roadmap's precomputed swept-volume trees to
label edges **Red** (certified invalid), **Green** (certified valid), or
**Gray** (undecided). A planner replans over the surviving subgraph.

**Spans** are the contribution: rather than rebuilding that geometry every
frame, one prediction is frozen over the horizon (optionally cut into `k` time
slices). Each frame then costs a single *conformance check* — is the observed
obstacle still inside the prediction's inflated envelope at this instant? If
so, all labels remain certified and lapsed slices expire by bookkeeping alone.
The expensive path runs only on conformance violation, horizon exhaustion, or
a new track.

## Layout

```
include/spite_d/, src/
  common/        shared plain-struct data contracts (no ROS, no Eigen)
  perception/    U-map depth detector + box tracker (port of map_manager)
  trajectory/    Predictor interface + constant-velocity implementation
  spite/         ValidityServer: predictions -> SPITE geometry -> RGG labels
  dynamic_map/   Span, SpanPipeline  (the contribution)
  planner/       Dijkstra replanner + roadmap graph persistence
  nodes/         thin rclcpp shims (ROS build only)
tools/           offline roadmap builders, terminal ablation demo
tests/           unit + integration tests (the executable specification)
gazebo/worlds/   depth-collection simulation worlds
launch/          pipeline launch file
```

Everything except `src/nodes/` is ROS-free and unit-testable without a ROS
installation. The build has two modes:

- **core mode** — plain CMake. Builds the libraries, offline tools, and tests.
  Used for development on machines without ROS (e.g. macOS).
- **ROS mode** — `colcon` / `ament_cmake` detected automatically. Additionally
  builds the message interfaces and node executables.

## Dependencies

`spite_d` links [open-spite](https://github.com/parasol-lab/open-spite) as a
CMake subproject; its path is supplied at configure time via
`SPITE_D_OPEN_SPITE_DIR` (pointing at the repo root — the directory containing
open-spite's own `CMakeLists.txt`). open-spite in turn needs CGAL, Boost, GMP,
MPFR, Eigen, FCL and nlohmann_json. OMPL is required for the roadmap builders.

### Linux (ROS mode)

```bash
sudo apt install libcgal-dev libompl-dev libeigen3-dev libfcl-dev \
                 nlohmann-json3-dev
```

### macOS (core mode)

CGAL and friends come from Conan; note **CGAL ≥ 5.6** is required — 5.5.x does
not compile under recent Apple Clang.

```bash
pip install conan && conan profile detect
cd /path/to/open-spite
export CMAKE_POLICY_VERSION_MINIMUM=3.5      # CMake 4.x vs. older recipes
conan install . --output-folder=build --build=missing \
      -s build_type=Release -s compiler.cppstd=gnu17
```

OpenCV (for the perception module) and OMPL come from Homebrew:

```bash
brew install opencv ompl
```

## Building

### Core mode

```bash
cd src/SPITE-D
export CMAKE_POLICY_VERSION_MINIMUM=3.5
cmake -B build-core -G Ninja -DCMAKE_BUILD_TYPE=Release \
      -DSPITE_D_OPEN_SPITE_DIR=/path/to/open-spite \
      -DCMAKE_TOOLCHAIN_FILE=/path/to/open-spite/build/conan_toolchain.cmake
cmake --build build-core
ctest --test-dir build-core --output-on-failure
```

Targets degrade gracefully: without OpenCV the perception library is skipped,
without OMPL the roadmap builders are skipped, without
`SPITE_D_OPEN_SPITE_DIR` the validity and span-pipeline targets are skipped.
Watch the configure output for `skipped` lines if an expected binary is
missing.

> Conan generates a single-configuration toolchain, so a `Debug` build fails to
> find CGAL. For a debuggable build, keep `CMAKE_BUILD_TYPE=Release` and add
> `-DCMAKE_CXX_FLAGS_RELEASE="-g -O0"` in a separate build directory.

### ROS mode (ROS 2 Jazzy)

```bash
cd ~/ros2_ws          # workspace containing src/SPITE-D
colcon build --packages-select spite_d \
      --cmake-args -DSPITE_D_OPEN_SPITE_DIR=/path/to/open-spite
source install/setup.bash
```

To avoid re-passing the path, pin it once in `~/.colcon/defaults.yaml`:

```yaml
build:
  cmake-args:
    - -DSPITE_D_OPEN_SPITE_DIR=/home/you/open-spite
```

`--cmake-clean-cache` erases *all* cached `-D` settings, so any time you use it
every `-D` you care about must be on the same command line — an empty
`SPITE_D_OPEN_SPITE_DIR` silently drops the validity node and roadmap
builders.

## Tests

`ctest` runs eight tests; they double as the specification for each module.

| test | covers |
| --- | --- |
| `test_predictor` | constant-velocity extrapolation, σ growth, `k=0` ⇒ fixed horizon |
| `test_replanner` | Dijkstra over the valid subgraph, blocked-path detection |
| `test_validity_server` | full loop: obstacle crosses corridor → Red → detour → `Forget` → heal |
| `test_roadmap_roundtrip` | graph + geometry serialization; reloaded roadmap reproduces labels |
| `test_perception` | tracker vs. analytically rendered depth frames (position, velocity, stable IDs) |
| `test_span` | envelope, conformance, refresh, trajectory slicing |
| `test_span_pipeline` | soundness between rebuilds; amortization counts; slice expiry |
| `test_open_spite` | upstream library's own suite |

## Offline tools

Build a roadmap once, then reuse it at runtime. Both builders write
`roadmap_graph.txt` (vertices, edges, costs, joint configurations) and
`roadmap_geoms.txt` (per-element OBB and spline approximations).

```bash
# Rigid box robot in SE(3)
./build-core/build_roadmap --out ~/rm --grow 1.0

# UR5: 6-DOF joint-space PRM with per-link swept volumes
./build-core/build_roadmap_ur5 --out ~/rm_ur5 --grow 1.0
```

Static obstacles default to two pillars; override with `--obstacles FILE`
(one axis-aligned box per line: `xmin ymin zmin xmax ymax zmax`).

### Ablation demo

Runs the same simulated obstacle through frame-by-frame and span-based
certification and reports geometry passes, update timings, and whether the two
modes made identical replanning decisions.

```bash
./build-core/demo_dynamic --roadmap ~/rm_ur5              # both modes
./build-core/demo_dynamic --roadmap ~/rm --slices 4       # time-sliced spans
./build-core/demo_dynamic --roadmap ~/rm --trace          # per-frame table
```

`--dump-json FILE` additionally writes per-frame state (roadmap, labels,
envelope, path) for external plotting; `tools/make_viewer.py` renders it as a
standalone HTML animation.

## Running the pipeline

```bash
ros2 launch spite_d pipeline.launch.py \
    world:=depth_single_cross \
    roadmap_graph:=$HOME/rm/roadmap_graph.txt \
    roadmap_geoms:=$HOME/rm/roadmap_geoms.txt
```

Starts Gazebo, the `ros_gz_bridge`, and the three nodes. `headless:=false`
opens the Gazebo GUI (default is server-only with offscreen rendering, which
is what a machine without a display needs). Worlds live in `gazebo/worlds/`:
`depth_single_cross` (one actor crossing at constant speed — clean ground
truth) and `depth_multi_cross` (three actors, association stress).

Topics:

| topic | type |
| --- | --- |
| `/camera/depth_image`, `/camera/camera_info` | sensor input (bridged from Gazebo) |
| `/spite_d_perception/obstacles` | `spite_d/ObstacleArray` |
| `/spite_d_prediction/predictions` | `spite_d/PredictedTrajectoryArray` |
| `/spite_d_validity/path` | `nav_msgs/Path` |

Every terminal that inspects custom messages needs `source install/setup.bash`,
otherwise `ros2 topic echo` reports the message type as invalid.

`tools/setup_remote_gui.sh` sets up browser-based VNC access (RViz / Gazebo
over the network) on a headless development machine.

## Status

Validated: the perception → prediction → validity → replanning pipeline runs
end-to-end in Gazebo; the span mechanism is validated by unit tests
(soundness, amortization, slice expiry) and measured in controlled kinematic
ablations on both the box and UR5 roadmaps.

Known gaps, in current priority order:

1. **Gazebo-in-the-loop spans** — `validity_node` still calls `ValidityServer`
   directly every frame; it does not yet use `SpanPipeline`.
2. **Gray edges** — treated as blocked in the deployed node and as traversable
   in `demo_dynamic`; the two should become one explicit policy, and
   fine-grained (FCL) resolution of Gray is unimplemented (`resolveGray` is a
   stub).
3. **Nonlinear trajectories** — spans assume a locally linear prediction;
   curved and multimodal predictions need per-slice tightening and an oriented
   (along-track / cross-track) envelope.
4. **Confidence-based prediction** — only constant velocity exists; the
   `Predictor` interface is the intended drop-in point.
5. **Tracker robustness** — no continuity filter (map_manager's vote-history
   gate was not ported), which shows up as track-ID churn on walking actors;
   obstacle boxes are axis-aligned rather than oriented.
