// Integration test of the span contribution:
//
//   1. A conforming obstacle costs almost nothing: over 30 frames the
//      expensive predict+geometry+tree path runs only at spawn and at
//      the scheduled horizon refresh (vs. 30 times for the baseline).
//   2. Classifications stay correct between rebuilds (soundness).
//   3. A deviation (obstacle turns) is caught by conformance, forcing
//      a rebuild that both blocks the newly threatened corridor and
//      heals the abandoned one.
//
// Same two-corridor roadmap as test_validity_server.

#include "trajectory/span_pipeline.hpp"
#include "spite/validity_server.hpp"
#include "trajectory/predictor.hpp"

#include "DynamicRoadmapTool.h"

#include <cassert>
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

}  // namespace

int main() {
  using namespace spite_d;
  using V = ValidityServer::Validity;

  // Two-corridor roadmap: edge (0,1) along y=0, edge (2,3) along y=5.
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

  const double kSigmaGain = 1.0;
  ValidityServer server(std::move(drm), {kSigmaGain});

  ConstantVelocityPredictor predictor(/*stdGrowthRate=*/0.0);
  SpanPipeline::Params params;
  params.span.sigmaGain = kSigmaGain;  // Envelope == geometry inflation.
  params.span.slack = 0.05;
  params.span.refreshFraction = 0.7;
  params.horizon = 4.0;
  params.dt = 0.25;
  SpanPipeline pipeline(predictor, server, params);

  // Obstacle crossing corridor (0,1): from (1,-2,0), +y at 1.2 m/s.
  // Observations follow the constant-velocity ground truth exactly.
  TrackedObstacle track;
  track.id = 7;
  track.halfExtents = {0.3, 0.3, 0.3};
  track.velocity = {0.0, 1.2, 0.0};
  track.positionStd = {0.05, 0.05, 0.05};

  const double kDt = 0.1;
  for (int f = 0; f < 30; ++f) {
    track.stamp = f * kDt;
    track.pose.translation = {1.0, -2.0 + 1.2 * track.stamp, 0.0};
    pipeline.Update({track});
    // Soundness between rebuilds: the crossed corridor stays certified
    // INVALID while the span is alive; the far corridor stays VALID.
    assert(server.GetEdgeValidity(0, 1) == V::INVALID);
    assert(server.GetEdgeValidity(2, 3) == V::VALID);
  }

  // The amortization claim: 30 frames, but the expensive path ran only
  // at spawn (t=0) and at the scheduled refresh (0.7 * 4s = 2.8s).
  const auto& stats = pipeline.GetStats();
  assert(stats.frames == 30);
  assert(stats.rebuilds == 2);
  assert(stats.conforming == 28);

  // Deviation: the obstacle turns and heads for corridor (2,3).
  const size_t rebuildsBefore = pipeline.GetStats().rebuilds;
  track.stamp = 3.0;
  track.pose.translation = {1.0, 4.0, 0.0};  // Far off its prediction.
  track.velocity = {0.0, 1.2, 0.0};
  pipeline.Update({track});
  assert(pipeline.GetStats().rebuilds == rebuildsBefore + 1);
  // The rebuilt geometry threatens corridor 2 and abandons corridor 1.
  assert(server.GetEdgeValidity(2, 3) == V::INVALID);
  assert(server.GetEdgeValidity(0, 1) == V::VALID);

  // Track lost: everything heals.
  pipeline.Forget(7);
  assert(server.GetEdgeValidity(2, 3) == V::VALID);

  // ---- Time-sliced mode (k=4): the corridor heals BEHIND the obstacle
  // by slice expiry alone -- zero geometry rebuilds -- which the
  // union-over-horizon span above could never do (it stays INVALID for
  // the whole horizon).
  SpanPipeline::Params slicedParams = params;
  slicedParams.slices = 4;
  slicedParams.span.refreshFraction = 1.0;  // Isolate expiry from refresh.
  SpanPipeline sliced(predictor, server, slicedParams);

  TrackedObstacle track2 = track;
  track2.id = 8;
  for (int f = 0; f <= 31; ++f) {  // t = 0 .. 3.1
    track2.stamp = f * kDt;
    track2.pose.translation = {1.0, -2.0 + 1.2 * track2.stamp, 0.0};
    sliced.Update({track2});
    if (f <= 25)  // While a blocking slice is still active: certified red.
      assert(server.GetEdgeValidity(0, 1) == V::INVALID);
  }
  // t = 3.1: slices 0-2 have lapsed; only slice 3 (obstacle far past the
  // corridor) remains -> the corridor is open again...
  assert(server.GetEdgeValidity(0, 1) == V::VALID);
  // ...and it cost one span build total, the rest was bookkeeping.
  assert(sliced.GetStats().rebuilds == 1);
  assert(sliced.GetStats().expiries == 3);

  sliced.Forget(8);
  assert(server.GetEdgeValidity(0, 1) == V::VALID);

  return 0;
}
