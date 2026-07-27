// Integration test of the baseline pipeline core:
//   ConstantVelocityPredictor -> ValidityServer(DynamicRoadmapTool) -> Replanner
//
// Roadmap: a 0.25-halfwidth cube robot with two corridors,
//   edge (0,1): (0,0,0) -> (2,0,0)   -- crossed by the obstacle
//   edge (2,3): (0,5,0) -> (2,5,0)   -- far away, must stay valid
// Obstacle: 0.3-halfwidth cube at x=1 moving +y through the first corridor.

#include "spite_d/planner/replanner.hpp"
#include "spite_d/spite/validity_server.hpp"
#include "spite_d/trajectory/predictor.hpp"

#include "DynamicRoadmapTool.h"

#include <cassert>
#include <memory>

namespace {

using spite_d::Pose3;
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

}  // namespace

int main() {
  using namespace spite_d;

  auto drm = std::make_unique<DynamicRoadmapTool>();
  drm->SetUseUnderApprox(true);
  drm->SetBodySpheres({{{DynamicRoadmapTool::Point3d(0, 0, 0), kRobotHalf}}});

  const Vec3 v0{0, 0, 0}, v1{2, 0, 0}, v2{0, 5, 0}, v3{2, 5, 0};
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

  ValidityServer server(std::move(drm), {/*sigmaGain=*/0.0});

  // Obstacle crossing corridor (0,1) at x=1, moving in +y.
  TrackedObstacle track;
  track.id = 42;
  track.stamp = 0.0;
  track.pose.translation = {1.0, -2.0, 0.0};
  track.velocity = {0.0, 1.0, 0.0};
  track.halfExtents = {0.3, 0.3, 0.3};

  ConstantVelocityPredictor predictor;
  server.Update({predictor.Predict(track, /*horizon=*/4.0, /*dt=*/0.25)});

  using V = ValidityServer::Validity;
  // The crossed corridor: the obstacle's under-approx spheres pass through
  // the edge spline -> certified INVALID (Red).
  assert(server.GetEdgeValidity(0, 1) == V::INVALID);
  // The far corridor is untouched -> Green.
  assert(server.GetEdgeValidity(2, 3) == V::VALID);
  // Vertices sit outside the obstacle's swept volume.
  assert(server.GetVertexValidity(0) == V::VALID);
  assert(server.GetVertexValidity(1) == V::VALID);

  // Replanner sees the blocked edge through the validity predicate.
  Replanner planner(4, {{0, 1, 1.0}, {1, 3, 5.5}, {0, 2, 5.0}, {2, 3, 1.0}});
  const auto isValid = [&server](size_t a, size_t b) {
    // Edges absent from the roadmap geometry (connectors 1-3, 0-2 here)
    // default to VALID in the tool; only certified-INVALID edges block.
    return server.GetEdgeValidity(a, b) != V::INVALID;
  };
  auto path = planner.Plan(0, 3, isValid);
  assert((path == std::vector<Replanner::VID>{0, 2, 3}));

  // Track lost: its contributions are dropped and the corridor re-opens.
  server.Forget(42);
  assert(server.GetEdgeValidity(0, 1) == V::VALID);
  path = planner.Plan(0, 3, isValid);
  assert((path == std::vector<Replanner::VID>{0, 1, 3}));

  // ---- RetainOnly: obstacles that vanish from the snapshot must stop
  // blocking. This is what the ROS node relies on -- without it a
  // departed obstacle blocks its last-known edges for the process's
  // lifetime.
  track.id = 7;
  track.stamp = 0.0;
  track.pose.translation = {1.0, -2.0, 0.0};
  server.Update({predictor.Predict(track, 4.0, 0.25)});
  assert(server.GetEdgeValidity(0, 1) == V::INVALID);

  // A snapshot that still contains id 7 changes nothing.
  server.RetainOnly({7});
  assert(server.GetEdgeValidity(0, 1) == V::INVALID);

  // A snapshot without it releases the track and heals the corridor.
  server.RetainOnly({});
  assert(server.GetEdgeValidity(0, 1) == V::VALID);

  // Slots are recycled: many short-lived ids (the track-churn pattern
  // real perception produces) must not accumulate. Each new id reuses
  // the slot freed by the previous one, and the final state is clean.
  for (int32_t id = 100; id < 140; ++id) {
    track.id = id;
    server.Update({predictor.Predict(track, 4.0, 0.25)});
    assert(server.GetEdgeValidity(0, 1) == V::INVALID);
    server.RetainOnly({});
    assert(server.GetEdgeValidity(0, 1) == V::VALID);
  }

  return 0;
}
