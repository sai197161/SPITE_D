// Offline roadmap builder for the UR5 manipulator.
//
// Joint-space (6-DOF) PRM with per-link swept-volume approximations:
// for every vertex/edge, forward kinematics places each link's
// collision box along the motion, producing one BodyMotion per link
// (the multi-body case of DynamicRoadmapTool). Output files match
// build_roadmap's, with the graph in v2 format (joint configurations;
// VertexState.position holds the end-effector point for visualization
// and nearest-vertex lookup).
//
// FK and link geometry follow open-spite's examples/ompl_manipulator
// (ur5_fk.hpp DH table; a LINK_R x LINK_R x LINK_LEN box per link with
// one inscribed sphere). Static obstacles shape the roadmap only;
// dynamic obstacles are handled online by the validity server.
//
// Usage:
//   build_roadmap_ur5 --out DIR [--grow SECONDS] [--edge-samples N]
//                     [--obstacles FILE]   # AABBs, one per line

#include "planner/roadmap_io.hpp"

#include "DynamicRoadmapTool.h"
#include "ur5_fk.hpp"

#include <ompl/base/PlannerData.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/geometric/planners/prm/PRM.h>

#include <Eigen/Dense>

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

constexpr int kJoints = ur5::kNumJoints;
constexpr int kLinks = ur5::kNumLinks;
constexpr double kLinkR = 0.05;    // link box half-extent in x,y
constexpr double kLinkLen = 0.16;  // link box full length along local z

// Workspace bounding box for the AABB trees: generous around UR5 reach.
constexpr double kWsLo = -1.2, kWsHi = 1.2;

struct AxisAlignedBox {
  double xmin, ymin, zmin, xmax, ymax, zmax;
};

struct Options {
  std::string outDir;
  std::string obstaclesFile;
  double growSeconds = 1.0;
  int edgeSamples = 8;
};

bool ParseArgs(int argc, char** argv, Options& opt) {
  for (int i = 1; i < argc; ++i) {
    if (!std::strcmp(argv[i], "--out") && ++i < argc) opt.outDir = argv[i];
    else if (!std::strcmp(argv[i], "--obstacles") && ++i < argc) opt.obstaclesFile = argv[i];
    else if (!std::strcmp(argv[i], "--grow") && ++i < argc) opt.growSeconds = std::atof(argv[i]);
    else if (!std::strcmp(argv[i], "--edge-samples") && ++i < argc) opt.edgeSamples = std::atoi(argv[i]);
    else { std::cerr << "unknown/incomplete arg: " << argv[i] << "\n"; return false; }
  }
  return !opt.outDir.empty();
}

std::vector<AxisAlignedBox> LoadObstacles(const Options& opt) {
  if (opt.obstaclesFile.empty()) {
    // Defaults from open-spite's manipulator example: a block along the
    // reach and a shelf-like box, both shaping the roadmap.
    return {{0.55, -0.15, 0.00, 0.75, 0.15, 0.30},
            {-0.15, 0.55, 0.30, 0.15, 0.85, 0.60}};
  }
  std::vector<AxisAlignedBox> boxes;
  std::ifstream in(opt.obstaclesFile);
  AxisAlignedBox b;
  while (in >> b.xmin >> b.ymin >> b.zmin >> b.xmax >> b.ymax >> b.zmax)
    boxes.push_back(b);
  return boxes;
}

open_spite::PointCloud3d LinkCloudLocal() {
  open_spite::PointCloud3d c;
  const double h = kLinkLen * 0.5;
  for (int sx : {-1, 1})
    for (int sy : {-1, 1})
      for (int sz : {-1, 1})
        c.emplace_back(sx * kLinkR, sy * kLinkR, sz * h);
  return c;
}

open_spite::Transformation3d AffineToXf(const Eigen::Affine3d& T) {
  open_spite::Transformation3d xf;
  const Eigen::Matrix3d R = T.linear();
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c) xf.rotation[r][c] = R(r, c);
  const Eigen::Vector3d t = T.translation();
  xf.translation = {t.x(), t.y(), t.z()};
  return xf;
}

void AppendTransformed(const open_spite::PointCloud3d& local,
                       const Eigen::Affine3d& T,
                       open_spite::PointCloud3d& out) {
  for (const auto& p : local) {
    const Eigen::Vector3d w = T * Eigen::Vector3d(p.x(), p.y(), p.z());
    out.emplace_back(w.x(), w.y(), w.z());
  }
}

std::vector<double> StateToConfig(const ob::State* state) {
  const auto* s = state->as<ob::RealVectorStateSpace::StateType>();
  return std::vector<double>(s->values, s->values + kJoints);
}

/// Per-link AABB-vs-static-obstacle check (as in the example): fast and
/// conservative enough to shape the roadmap.
class ArmValidityChecker : public ob::StateValidityChecker {
 public:
  ArmValidityChecker(const ob::SpaceInformationPtr& si,
                     std::vector<AxisAlignedBox> boxes)
      : ob::StateValidityChecker(si),
        m_local(LinkCloudLocal()),
        m_boxes(std::move(boxes)) {}

  bool isValid(const ob::State* state) const override {
    const auto frames = ur5::linkFrames(StateToConfig(state));
    for (const auto& T : frames) {
      double lo[3] = {1e30, 1e30, 1e30}, hi[3] = {-1e30, -1e30, -1e30};
      for (const auto& p : m_local) {
        const Eigen::Vector3d w = T * Eigen::Vector3d(p.x(), p.y(), p.z());
        const double v[3] = {w.x(), w.y(), w.z()};
        for (int k = 0; k < 3; ++k) {
          lo[k] = std::min(lo[k], v[k]);
          hi[k] = std::max(hi[k], v[k]);
        }
      }
      for (const auto& o : m_boxes) {
        if (hi[0] >= o.xmin && lo[0] <= o.xmax && hi[1] >= o.ymin &&
            lo[1] <= o.ymax && hi[2] >= o.zmin && lo[2] <= o.zmax)
          return false;
      }
    }
    return true;
  }

 private:
  open_spite::PointCloud3d m_local;
  std::vector<AxisAlignedBox> m_boxes;
};

std::vector<open_spite::BodyMotion> MotionsAlong(
    const std::vector<std::vector<double>>& configs) {
  const auto local = LinkCloudLocal();
  std::vector<open_spite::BodyMotion> motions(kLinks);
  for (const auto& q : configs) {
    const auto frames = ur5::linkFrames(q);
    for (int l = 0; l < kLinks; ++l) {
      AppendTransformed(local, frames[l], motions[l].cloud);
      motions[l].transforms.push_back(AffineToXf(frames[l]));
    }
  }
  return motions;
}

}  // namespace

int main(int argc, char** argv) {
  Options opt;
  if (!ParseArgs(argc, argv, opt)) {
    std::cerr << "usage: build_roadmap_ur5 --out DIR [--grow SECONDS] "
                 "[--edge-samples N] [--obstacles FILE]\n";
    return 1;
  }
  const auto obstacles = LoadObstacles(opt);

  // ---- Joint-space PRM.
  auto space = std::make_shared<ob::RealVectorStateSpace>(kJoints);
  ob::RealVectorBounds bounds(kJoints);
  const auto limits = ur5::jointLimits();
  for (int j = 0; j < kJoints; ++j) {
    bounds.setLow(j, limits[j].first);
    bounds.setHigh(j, limits[j].second);
  }
  space->setBounds(bounds);

  auto si = std::make_shared<ob::SpaceInformation>(space);
  si->setStateValidityChecker(
      std::make_shared<ArmValidityChecker>(si, obstacles));
  si->setStateValidityCheckingResolution(0.01);
  si->setup();

  auto pdef = std::make_shared<ob::ProblemDefinition>(si);
  ob::ScopedState<ob::RealVectorStateSpace> start(space), goal(space);
  for (int j = 0; j < kJoints; ++j) {
    start[j] = 0.0;
    goal[j] = (j == 0) ? M_PI / 2.0 : (j == 1 ? -M_PI / 4.0 : 0.0);
  }
  pdef->setStartAndGoalStates(start, goal);

  og::PRM prm(si);
  prm.setProblemDefinition(pdef);
  prm.setup();
  std::cout << "growing UR5 joint-space PRM for " << opt.growSeconds
            << "s ...\n";
  prm.constructRoadmap(ob::timedPlannerTerminationCondition(opt.growSeconds));

  ob::PlannerData pd(si);
  prm.getPlannerData(pd);
  const unsigned n = pd.numVertices();
  std::cout << "roadmap: " << n << " vertices, " << pd.numEdges()
            << " edges (both directions)\n";
  if (n == 0 || pd.numEdges() == 0) {
    std::cerr << "FATAL: empty roadmap; increase --grow\n";
    return 1;
  }

  // ---- Extract graph + build per-link approximations.
  spite_d::RoadmapGraph graph;
  graph.dof = kJoints;
  graph.vertices.resize(n);
  graph.configurations.resize(n);

  DynamicRoadmapTool drm;
  drm.SetUseUnderApprox(true);
  std::vector<std::vector<open_spite::SphereInBody>> spheres(kLinks);
  for (int l = 0; l < kLinks; ++l)
    spheres[l] = {open_spite::SphereInBody{open_spite::Point3d(0, 0, 0), kLinkR}};
  drm.SetBodySpheres(spheres);

  for (unsigned v = 0; v < n; ++v) {
    const auto q = StateToConfig(pd.getVertex(v).getState());
    graph.configurations[v] = q;
    const auto frames = ur5::linkFrames(q);
    const Eigen::Vector3d ee = frames.back().translation();
    graph.vertices[v].position = {ee.x(), ee.y(), ee.z()};

    auto motions = MotionsAlong({q});
    drm.AddVertex(v, motions);
  }

  for (unsigned v = 0; v < n; ++v) {
    std::vector<unsigned> neighbors;
    pd.getEdges(v, neighbors);
    for (unsigned w : neighbors) {
      if (w <= v) continue;  // Both directions listed; keep one.
      const auto qa = graph.configurations[v];
      const auto qb = graph.configurations[w];

      std::vector<std::vector<double>> configs;
      configs.reserve(opt.edgeSamples + 1);
      for (int s = 0; s <= opt.edgeSamples; ++s) {
        const double a = double(s) / opt.edgeSamples;
        std::vector<double> q(kJoints);
        for (int j = 0; j < kJoints; ++j)
          q[j] = (1.0 - a) * qa[j] + a * qb[j];
        configs.push_back(std::move(q));
      }

      graph.edges.push_back(
          {v, w,
           space->distance(pd.getVertex(v).getState(),
                           pd.getVertex(w).getState())});
      auto motions = MotionsAlong(configs);
      drm.AddEdge(v, w, motions);
    }
  }
  std::cout << "unique edges: " << graph.edges.size() << "\n";

  drm.Build({kWsLo, kWsLo, kWsLo}, {kWsHi, kWsHi, kWsHi});

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
