// Persistence round-trip:
//   1. RoadmapGraph write -> read -> identical.
//   2. DynamicRoadmapTool geometries write -> ReadGeometriesFromFile ->
//      a reloaded ValidityServer reproduces the same classification as
//      the in-memory one (same scenario as test_validity_server).

#include "planner/roadmap_io.hpp"
#include "spite/validity_server.hpp"
#include "trajectory/predictor.hpp"

#include "DynamicRoadmapTool.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <memory>

namespace {

using spite_d::Vec3;

constexpr double kRobotHalf = 0.25;

open_spite::BodyMotion MakeLineMotion(const Vec3& from, const Vec3& to,
                                      size_t steps) {
  open_spite::BodyMotion motion;
  for (size_t i = 0; i <= steps; ++i) {
    const double s = steps == 0 ? 0.0 : double(i) / double(steps);
    Vec3 center;
    for (int a = 0; a < 3; ++a) center[a] = from[a] + s * (to[a] - from[a]);
    open_spite::Transformation3d tf;
    tf.translation = {center[0], center[1], center[2]};
    motion.transforms.push_back(tf);
    for (int sx : {-1, 1})
      for (int sy : {-1, 1})
        for (int sz : {-1, 1})
          motion.cloud.emplace_back(center[0] + sx * kRobotHalf,
                                    center[1] + sy * kRobotHalf,
                                    center[2] + sz * kRobotHalf);
  }
  return motion;
}

void RunScenario(spite_d::ValidityServer& server) {
  using V = spite_d::ValidityServer::Validity;

  spite_d::TrackedObstacle track;
  track.id = 42;
  track.pose.translation = {1.0, -2.0, 0.0};
  track.velocity = {0.0, 1.0, 0.0};
  track.halfExtents = {0.3, 0.3, 0.3};

  spite_d::ConstantVelocityPredictor predictor;
  server.Update({predictor.Predict(track, 4.0, 0.25)});

  assert(server.GetEdgeValidity(0, 1) == V::INVALID);
  assert(server.GetEdgeValidity(2, 3) == V::VALID);
  server.Forget(42);
  assert(server.GetEdgeValidity(0, 1) == V::VALID);
}

}  // namespace

int main() {
  using namespace spite_d;

  const char* graphPath = "roundtrip_graph.txt";
  const char* geomPath = "roundtrip_geoms.txt";

  // ---- 1. Graph round-trip.
  RoadmapGraph graph;
  graph.vertices = {{{0, 0, 0}, {0, 0, 0, 1}},
                    {{2, 0, 0}, {0, 0, 0, 1}},
                    {{0, 5, 0}, {0, 0, 0, 1}},
                    {{2, 5, 0}, {0, 0, 0, 1}}};
  graph.edges = {{0, 1, 2.0}, {2, 3, 2.0}};
  assert(WriteRoadmapGraph(graph, graphPath));

  RoadmapGraph reread;
  assert(ReadRoadmapGraph(reread, graphPath));
  assert(reread.vertices.size() == 4 && reread.edges.size() == 2);
  assert(std::abs(reread.vertices[1].position[0] - 2.0) < 1e-15);
  assert(reread.edges[1].src == 2 && reread.edges[1].tgt == 3);

  // ---- 2. Geometry round-trip through a full validity scenario.
  const Vec3 v0{0, 0, 0}, v1{2, 0, 0}, v2{0, 5, 0}, v3{2, 5, 0};
  {
    auto drm = std::make_unique<DynamicRoadmapTool>();
    drm->SetUseUnderApprox(true);
    drm->SetBodySpheres(
        {{{DynamicRoadmapTool::Point3d(0, 0, 0), kRobotHalf}}});
    std::vector<open_spite::BodyMotion> m;
    m = {MakeLineMotion(v0, v0, 0)};
    drm->AddVertex(0, m);
    m = {MakeLineMotion(v1, v1, 0)};
    drm->AddVertex(1, m);
    m = {MakeLineMotion(v2, v2, 0)};
    drm->AddVertex(2, m);
    m = {MakeLineMotion(v3, v3, 0)};
    drm->AddVertex(3, m);
    m = {MakeLineMotion(v0, v1, 10)};
    drm->AddEdge(0, 1, m);
    m = {MakeLineMotion(v2, v3, 10)};
    drm->AddEdge(2, 3, m);
    drm->Build({-10, -10, -10}, {10, 10, 10});
    drm->WriteGeometries(geomPath);

    ValidityServer inMemory(std::move(drm), {});
    RunScenario(inMemory);
  }
  {
    auto drm = std::make_unique<DynamicRoadmapTool>();
    drm->SetUseUnderApprox(true);
    const bool ok = drm->ReadGeometriesFromFile(geomPath);
    assert(ok);

    ValidityServer reloaded(std::move(drm), {});
    RunScenario(reloaded);
  }

  std::remove(graphPath);
  std::remove(geomPath);
  return 0;
}
