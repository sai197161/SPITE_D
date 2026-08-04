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

/// Two-corridor roadmap: edge (0,1) along y=0, edge (2,3) along y=5.
/// Built twice so each mode gets an independent, identical world.
std::unique_ptr<DynamicRoadmapTool> MakeRoadmap() {
  using spite_d::Vec3;
  auto drm = std::make_unique<DynamicRoadmapTool>();
  drm->SetUseUnderApprox(true);
  drm->SetBodySpheres({{{DynamicRoadmapTool::Point3d(0, 0, 0), kRobotHalf}}});
  const Vec3 v0{0, 0, 0}, v1{2, 0, 0}, v2{0, 5, 0}, v3{2, 5, 0};
  std::vector<open_spite::BodyMotion> m;
  m = {MakeLineMotion(v0, v0, 0)}; drm->AddVertex(0, m);
  m = {MakeLineMotion(v1, v1, 0)}; drm->AddVertex(1, m);
  m = {MakeLineMotion(v2, v2, 0)}; drm->AddVertex(2, m);
  m = {MakeLineMotion(v3, v3, 0)}; drm->AddVertex(3, m);
  m = {MakeLineMotion(v0, v1, 10)}; drm->AddEdge(0, 1, m);
  m = {MakeLineMotion(v2, v3, 10)}; drm->AddEdge(2, 3, m);
  drm->Build({-10, -10, -10}, {10, 10, 10});
  return drm;
}

}  // namespace

int main() {
  using namespace spite_d;
  using V = ValidityServer::Validity;

  const double kSigmaGain = 1.0;
  ConstantVelocityPredictor predictor(/*stdGrowthRate=*/0.0);

  SpanPipeline::Params params;
  params.span.sigmaGain = kSigmaGain;  // Envelope == geometry inflation.
  params.span.slack = 0.05;
  params.span.refreshFraction = 0.7;
  params.horizon = 4.0;
  params.dt = 0.25;

  // Two identical worlds: one certified every frame (the baseline), one
  // driven through SpanPipeline. Same roadmap, same obstacle, same
  // predictor -- the only difference is when geometry is rebuilt.
  ValidityServer baseline(MakeRoadmap(), {kSigmaGain});
  ValidityServer immediate(MakeRoadmap(), {kSigmaGain});
  ValidityServer server(MakeRoadmap(), {kSigmaGain});
  SpanPipeline pipeline(predictor, server, params);

  // Obstacle crossing corridor (0,1): from (1,-2,0), +y at 1.2 m/s.
  // Observations follow the constant-velocity ground truth exactly.
  TrackedObstacle track;
  track.id = 7;
  track.halfExtents = {0.3, 0.3, 0.3};
  track.velocity = {0.0, 1.2, 0.0};
  track.positionStd = {0.05, 0.05, 0.05};

  // Soundness floor: whatever a span predicts, it must never call an edge
  // valid that the obstacle is blocking RIGHT NOW.
  //
  // Note the yardstick is the obstacle's current position, not the
  // frame-by-frame baseline. A span is frozen, so its remaining lookahead
  // shrinks as it ages -- 4s of foresight when built, 1.2s just before it
  // refreshes -- while the baseline always re-predicts a full horizon. The
  // baseline therefore blocks distant edges the span has not yet looked far
  // enough ahead to see. That is a difference in foresight, not in safety,
  // and refreshFraction is the knob that bounds how far foresight may erode.
  const auto sound = [&](size_t a, size_t b) {
    return !(server.GetEdgeValidity(a, b) == V::VALID &&
             immediate.GetEdgeValidity(a, b) == V::INVALID);
  };

  const double kDt = 0.1;
  for (int f = 0; f < 30; ++f) {
    track.stamp = f * kDt;
    track.pose.translation = {1.0, -2.0 + 1.2 * track.stamp, 0.0};

    baseline.Update({predictor.Predict(track, params.horizon, params.dt)});
    // Horizon 0 -> a single sample at the obstacle's current pose.
    immediate.Update({predictor.Predict(track, 0.0, params.dt)});
    pipeline.Update({track});

    assert(sound(0, 1));
    assert(sound(2, 3));
  }

  // Note the far corridor (y=5) is deliberately NOT asserted valid here.
  // The horizon is 4s and the obstacle moves 1.2 m/s, so the prediction
  // reaches 4.8m ahead: from about t=1.8s it extends past y=5 and both
  // modes correctly block that corridor too. Lookahead buys foresight at
  // the cost of blocking things the obstacle has not reached yet -- the
  // conservatism that time-slicing exists to reduce.

  // While the obstacle is still approaching or inside the corridor, both
  // modes agree it is blocked -- amortization does not delay detection.
  assert(server.GetEdgeValidity(0, 1) == baseline.GetEdgeValidity(0, 1));

  // The amortization claim: the baseline rebuilt geometry on all 30 frames;
  // the span rebuilt at spawn and once at the scheduled refresh (0.7 * 4s).
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
  // A fresh world: the scenarios above left their own tracks and slots in
  // `server`, and two pipelines sharing one server would fight over them.
  SpanPipeline::Params slicedParams = params;
  slicedParams.slices = 4;
  slicedParams.span.refreshFraction = 1.0;  // Isolate expiry from refresh.
  ValidityServer slicedServer(MakeRoadmap(), {kSigmaGain});
  SpanPipeline sliced(predictor, slicedServer, slicedParams);

  // With 4 slices over a 4s horizon each slice spans 1s, and the corridor
  // heals in stages as they lapse:
  //
  //   t < 2.0   slice 1 covers y in [-0.8, 0.4]; its deflated spheres reach
  //             the corridor spline           -> certified INVALID
  //   2.0-3.0   slice 1 has expired. Slice 2 (y in [0.4, 1.6]) still
  //             overlaps by its inflated box but no longer by its spheres
  //                                           -> honestly UNDECIDED
  //   t >= 3.0  slice 2 expires too           -> corridor VALID again
  //
  // A union-over-horizon span cannot produce this: it stays blocked for the
  // entire horizon until the whole span is rebuilt. The staging is what the
  // extra per-slice geometry buys.
  TrackedObstacle track2 = track;
  track2.id = 8;
  for (int f = 0; f <= 31; ++f) {  // t = 0 .. 3.1
    track2.stamp = f * kDt;
    track2.pose.translation = {1.0, -2.0 + 1.2 * track2.stamp, 0.0};
    sliced.Update({track2});

    const auto v = slicedServer.GetEdgeValidity(0, 1);
    if (track2.stamp < 2.0)
      assert(v == V::INVALID);          // obstacle is crossing
    else if (track2.stamp < 3.0)
      assert(v != V::VALID);            // undecided, still not passable
    else
      assert(v == V::VALID);            // all covering slices lapsed
  }
  assert(slicedServer.GetEdgeValidity(0, 1) == V::VALID);
  // ...and it cost one span build total, the rest was bookkeeping.
  assert(sliced.GetStats().rebuilds == 1);
  assert(sliced.GetStats().expiries == 3);

  sliced.Forget(8);
  assert(slicedServer.GetEdgeValidity(0, 1) == V::VALID);

  return 0;
}
