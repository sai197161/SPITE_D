// Pure Span semantics: interpolation, envelope, conformance, expiry.

#include "trajectory/span.hpp"

#include <cassert>
#include <cmath>

int main() {
  using namespace spite_d;

  // Constant-velocity trajectory: from (0,0,0) moving +x at 1 m/s,
  // horizon 2 s sampled at 0.5 s; sigma constant 0.1.
  PredictedTrajectory traj;
  traj.id = 9;
  traj.halfExtents = {0.3, 0.3, 0.3};
  for (int i = 0; i <= 4; ++i) {
    const double t = 10.0 + 0.5 * i;
    traj.stamps.push_back(t);
    Pose3 pose;
    pose.translation = {0.5 * i, 0.0, 0.0};
    traj.poses.push_back(pose);
    traj.positionStd.push_back({0.1, 0.1, 0.1});
  }

  SpanParams params;
  params.sigmaGain = 1.0;
  params.slack = 0.05;
  params.refreshFraction = 0.7;
  Span span(traj, params);

  assert(span.StartTime() == 10.0 && span.EndTime() == 12.0);

  // Interpolation halfway between samples: t=10.25 -> x=0.25.
  assert(std::abs((*span.PredictedPosition(10.25))[0] - 0.25) < 1e-12);
  // Clamped past the end.
  assert(std::abs((*span.PredictedPosition(99.0))[0] - 2.0) < 1e-12);

  // Envelope: 0.3 + 1.0*0.1 + 0.05 = 0.45 per axis.
  assert(std::abs(span.EnvelopeHalfWidth(11.0)[0] - 0.45) < 1e-12);

  TrackedObstacle obs;
  obs.id = 9;

  // On the predicted path -> conforms.
  obs.stamp = 11.0;
  obs.pose.translation = {1.0, 0.0, 0.0};
  assert(span.Conforms(obs));

  // Deviated but inside the envelope -> still conforms.
  obs.pose.translation = {1.0, 0.44, 0.0};
  assert(span.Conforms(obs));

  // Outside the envelope -> violation.
  obs.pose.translation = {1.0, 0.46, 0.0};
  assert(!span.Conforms(obs));

  // Past the horizon -> exhausted regardless of position.
  obs.stamp = 12.5;
  obs.pose.translation = {2.0, 0.0, 0.0};
  assert(!span.Conforms(obs));

  // Refresh: horizon 2 s, fraction 0.7 -> refresh from t = 11.4.
  assert(!span.NeedsRefresh(11.3));
  assert(span.NeedsRefresh(11.5));

  // ---- Slicing: 5 samples over [10,12], k=2 -> 3 samples each, the
  // middle sample (t=11) shared by both slices (no geometric gap).
  const auto slices = SliceTrajectory(traj, 2);
  assert(slices.size() == 2);
  assert(slices[0].stamps.size() == 3 && slices[1].stamps.size() == 3);
  assert(slices[0].stamps.back() == 11.0 && slices[1].stamps.front() == 11.0);
  assert(slices[0].poses.back().translation[0] ==
         slices[1].poses.front().translation[0]);

  // k=1 passthrough.
  assert(SliceTrajectory(traj, 1).size() == 1);

  // Slice indexing over [10,12] with k=4 (width 0.5).
  assert(SliceIndexAt(9.0, 10.0, 12.0, 4) == 0);   // clamped low
  assert(SliceIndexAt(10.2, 10.0, 12.0, 4) == 0);
  assert(SliceIndexAt(10.6, 10.0, 12.0, 4) == 1);
  assert(SliceIndexAt(11.9, 10.0, 12.0, 4) == 3);
  assert(SliceIndexAt(13.0, 10.0, 12.0, 4) == 3);  // clamped high

  return 0;
}
