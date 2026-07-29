// Offline roadmap builder.
//
// Grows an OMPL PRM for a rigid box robot in SE(3), then constructs
// open-spite over/under approximations for every vertex and edge and
// serializes everything the runtime needs:
//
//   <out>/roadmap_graph.txt  -- vertex states + edges with costs
//                               (spite_d format, see roadmap_io.hpp)
//   <out>/roadmap_geoms.txt  -- OBB/spline approximations
//                               (open-spite WriteGeometries format,
//                                reloaded via ReadGeometriesFromFile)
//
// The roadmap is built against STATIC obstacles only; dynamic obstacles
// are handled online by the validity server. Static obstacles are
// axis-aligned boxes, either the built-in defaults or one per line
// ("xmin ymin zmin xmax ymax zmax") in the --obstacles file.
//
// Usage:
//   build_roadmap --out DIR [--grow SECONDS] [--env-lo F] [--env-hi F]
//                 [--edge-samples N] [--obstacles FILE]
//
// The roadmap graph is extracted through ompl::base::PlannerData rather
// than PRM's boost graph so no Boost.Graph templates are instantiated
// here (keeps us independent of the Boost version OMPL was built with).

#include "planner/roadmap_io.hpp"

#include "DynamicRoadmapTool.h"

#include <ompl/base/PlannerData.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/geometric/planners/prm/PRM.h>

#include <array>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace {

// Rigid box robot, matching the Gazebo model: half-extents + inscribed
// sphere at the body origin.
constexpr std::array<double, 3> kBoxHalf = {0.10, 0.10, 0.20};
constexpr double kInscribedRadius = 0.10;
const double kCircumscribedRadius =
    std::sqrt(kBoxHalf[0] * kBoxHalf[0] + kBoxHalf[1] * kBoxHalf[1] +
              kBoxHalf[2] * kBoxHalf[2]);

struct AxisAlignedBox {
  double xmin, ymin, zmin, xmax, ymax, zmax;
};

struct Options {
  std::string outDir;
  std::string obstaclesFile;
  double growSeconds = 2.0;
  double envLo = 0.0;
  double envHi = 10.0;
  int edgeSamples = 8;
};

bool ParseArgs(int argc, char** argv, Options& opt) {
  for (int i = 1; i < argc; ++i) {
    auto next = [&](double& v) { return ++i < argc && (v = std::atof(argv[i]), true); };
    if (!std::strcmp(argv[i], "--out") && ++i < argc) opt.outDir = argv[i];
    else if (!std::strcmp(argv[i], "--obstacles") && ++i < argc) opt.obstaclesFile = argv[i];
    else if (!std::strcmp(argv[i], "--grow")) { if (!next(opt.growSeconds)) return false; }
    else if (!std::strcmp(argv[i], "--env-lo")) { if (!next(opt.envLo)) return false; }
    else if (!std::strcmp(argv[i], "--env-hi")) { if (!next(opt.envHi)) return false; }
    else if (!std::strcmp(argv[i], "--edge-samples") && ++i < argc) opt.edgeSamples = std::atoi(argv[i]);
    else { std::cerr << "unknown/incomplete arg: " << argv[i] << "\n"; return false; }
  }
  return !opt.outDir.empty();
}

std::vector<AxisAlignedBox> LoadObstacles(const Options& opt) {
  if (opt.obstaclesFile.empty()) {
    // Default: two pillars, matching the gazebo test worlds.
    return {{2.0, 2.0, 0.0, 3.0, 3.0, 10.0}, {6.0, 6.0, 0.0, 7.0, 7.0, 10.0}};
  }
  std::vector<AxisAlignedBox> boxes;
  std::ifstream in(opt.obstaclesFile);
  AxisAlignedBox b;
  while (in >> b.xmin >> b.ymin >> b.zmin >> b.xmax >> b.ymax >> b.zmax)
    boxes.push_back(b);
  return boxes;
}

// Conservative static-obstacle check: the robot's circumscribed sphere
// against inflated boxes.
class SphereVsBoxesChecker : public ob::StateValidityChecker {
 public:
  SphereVsBoxesChecker(const ob::SpaceInformationPtr& si,
                       std::vector<AxisAlignedBox> boxes, double radius)
      : ob::StateValidityChecker(si), m_boxes(std::move(boxes)), m_r(radius) {}

  bool isValid(const ob::State* state) const override {
    const auto* s = state->as<ob::SE3StateSpace::StateType>();
    const double x = s->getX(), y = s->getY(), z = s->getZ();
    for (const auto& o : m_boxes) {
      if (x + m_r >= o.xmin && x - m_r <= o.xmax &&
          y + m_r >= o.ymin && y - m_r <= o.ymax &&
          z + m_r >= o.zmin && z - m_r <= o.zmax)
        return false;
    }
    return true;
  }

 private:
  std::vector<AxisAlignedBox> m_boxes;
  double m_r;
};

open_spite::Transformation3d PoseFromState(const ob::State* state) {
  const auto* s = state->as<ob::SE3StateSpace::StateType>();
  const auto& q = s->rotation();
  const double x = q.x, y = q.y, z = q.z, w = q.w;
  open_spite::Transformation3d T;
  T.rotation = {{{1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)},
                 {2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)},
                 {2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)}}};
  T.translation = {s->getX(), s->getY(), s->getZ()};
  return T;
}

void AppendBoxCorners(open_spite::PointCloud3d& cloud,
                      const open_spite::Transformation3d& T) {
  for (int sx : {-1, 1})
    for (int sy : {-1, 1})
      for (int sz : {-1, 1}) {
        auto p = T.Apply({sx * kBoxHalf[0], sy * kBoxHalf[1], sz * kBoxHalf[2]});
        cloud.emplace_back(p[0], p[1], p[2]);
      }
}

std::vector<open_spite::BodyMotion> VertexMotion(const ob::State* state) {
  std::vector<open_spite::BodyMotion> motions(1);
  const auto T = PoseFromState(state);
  AppendBoxCorners(motions[0].cloud, T);
  motions[0].transforms.push_back(T);
  return motions;
}

std::vector<open_spite::BodyMotion> EdgeMotion(const ob::StateSpacePtr& space,
                                               const ob::State* s1,
                                               const ob::State* s2,
                                               int samples) {
  std::vector<open_spite::BodyMotion> motions(1);
  motions[0].cloud.reserve(8 * (samples + 1));
  ob::State* interp = space->allocState();
  for (int i = 0; i <= samples; ++i) {
    space->interpolate(s1, s2, double(i) / samples, interp);
    const auto T = PoseFromState(interp);
    AppendBoxCorners(motions[0].cloud, T);
    motions[0].transforms.push_back(T);
  }
  space->freeState(interp);
  return motions;
}

}  // namespace

int main(int argc, char** argv) {
  Options opt;
  if (!ParseArgs(argc, argv, opt)) {
    std::cerr << "usage: build_roadmap --out DIR [--grow SECONDS] "
                 "[--env-lo F] [--env-hi F] [--edge-samples N] "
                 "[--obstacles FILE]\n";
    return 1;
  }
  const auto obstacles = LoadObstacles(opt);

  // ---- Grow the PRM against static obstacles.
  auto space = std::make_shared<ob::SE3StateSpace>();
  ob::RealVectorBounds bounds(3);
  bounds.setLow(opt.envLo);
  bounds.setHigh(opt.envHi);
  space->setBounds(bounds);

  auto si = std::make_shared<ob::SpaceInformation>(space);
  si->setStateValidityChecker(std::make_shared<SphereVsBoxesChecker>(
      si, obstacles, kCircumscribedRadius));
  si->setStateValidityCheckingResolution(0.01);
  si->setup();

  // PRM requires a problem definition before it can be grown (its setup
  // is deferred otherwise and constructRoadmap crashes). The start/goal
  // pair only seeds growth; the whole roadmap is serialized regardless.
  auto pdef = std::make_shared<ob::ProblemDefinition>(si);
  ob::ScopedState<ob::SE3StateSpace> start(space), goal(space);
  const double margin = 0.05 * (opt.envHi - opt.envLo);
  start->setXYZ(opt.envLo + margin, opt.envLo + margin, opt.envLo + margin);
  start->rotation().setIdentity();
  goal->setXYZ(opt.envHi - margin, opt.envHi - margin, opt.envHi - margin);
  goal->rotation().setIdentity();
  pdef->setStartAndGoalStates(start, goal);

  og::PRM prm(si);
  prm.setProblemDefinition(pdef);
  prm.setup();
  std::cout << "growing PRM for " << opt.growSeconds << "s ...\n";
  prm.constructRoadmap(
      ob::timedPlannerTerminationCondition(opt.growSeconds));

  ob::PlannerData pd(si);
  prm.getPlannerData(pd);
  const unsigned n = pd.numVertices();
  std::cout << "roadmap: " << n << " vertices, " << pd.numEdges()
            << " edges\n";
  if (n == 0 || pd.numEdges() == 0) {
    std::cerr << "FATAL: empty roadmap; increase --grow\n";
    return 1;
  }

  // ---- Extract the graph and build the approximations.
  spite_d::RoadmapGraph graph;
  graph.vertices.resize(n);

  DynamicRoadmapTool drm;
  drm.SetUseUnderApprox(true);
  drm.SetBodySpheres(
      {{open_spite::SphereInBody{open_spite::Point3d(0, 0, 0), kInscribedRadius}}});

  for (unsigned v = 0; v < n; ++v) {
    const ob::State* state = pd.getVertex(v).getState();
    const auto* s = state->as<ob::SE3StateSpace::StateType>();
    graph.vertices[v].position = {s->getX(), s->getY(), s->getZ()};
    graph.vertices[v].quaternion = {s->rotation().x, s->rotation().y,
                                    s->rotation().z, s->rotation().w};
    auto motions = VertexMotion(state);
    drm.AddVertex(v, motions);
  }

  for (unsigned v = 0; v < n; ++v) {
    std::vector<unsigned> neighbors;
    pd.getEdges(v, neighbors);
    for (unsigned w : neighbors) {
      if (w <= v) continue;  // PlannerData lists both directions; keep one.
      const ob::State* sv = pd.getVertex(v).getState();
      const ob::State* sw = pd.getVertex(w).getState();
      graph.edges.push_back({v, w, space->distance(sv, sw)});
      auto motions = EdgeMotion(space, sv, sw, opt.edgeSamples);
      drm.AddEdge(v, w, motions);
    }
  }
  std::cout << "unique edges: " << graph.edges.size() << "\n";

  drm.Build({opt.envLo, opt.envLo, opt.envLo},
            {opt.envHi, opt.envHi, opt.envHi});

  // ---- Serialize.
  std::error_code ec;
  std::filesystem::create_directories(opt.outDir, ec);
  if (ec) {
    std::cerr << "FATAL: cannot create output dir " << opt.outDir << ": "
              << ec.message() << "\n";
    return 1;
  }
  const std::string graphPath = opt.outDir + "/roadmap_graph.txt";
  const std::string geomPath = opt.outDir + "/roadmap_geoms.txt";
  if (!spite_d::WriteRoadmapGraph(graph, graphPath)) {
    std::cerr << "FATAL: cannot write " << graphPath << "\n";
    return 1;
  }
  drm.WriteGeometries(geomPath);
  std::cout << "wrote " << graphPath << "\n";
  std::cout << "wrote " << geomPath << "\n";
  return 0;
}
