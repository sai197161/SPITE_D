#pragma once

// Spans: temporal amortization of obstacle validity certification.
//
// A Span freezes one predicted trajectory (plus the confidence gain k
// it was inflated with) over its horizon. The claim that makes spans
// sound: every roadmap classification derived from a span's geometry
// remains trustworthy AS LONG AS the obstacle conforms to the span --
// i.e. each new observation stays within the inflated envelope the
// geometry was built from. Conformance checking is the cheap runtime
// test of that condition; construction of the actual SPITE geometry
// (OBB + spheres) stays in the spite module, built from the same
// trajectory via ValidityServer.
//
// This module is deliberately open-spite-free and ROS-free: pure math
// over common/types.hpp, unit-testable anywhere.

#include "common/types.hpp"

#include <cstddef>
#include <optional>

namespace spite_d {

struct SpanParams {
  /// Confidence gain: the envelope half-width at time t is
  /// halfExtents + k * sigma(t) per axis (matches the geometry
  /// inflation in ValidityServer). Deviation beyond it = violation.
  double sigmaGain{1.0};
  /// Extra conformance slack (meters) so borderline sensor noise does
  /// not thrash rebuilds when sigmaGain is small or zero.
  double slack{0.05};
  /// Rebuild when this fraction of the horizon has elapsed, even if
  /// still conforming, so the span never runs dry of lookahead.
  double refreshFraction{0.7};
};

/// Split a sampled trajectory into k contiguous sub-trajectories of
/// equal time extent. Boundary samples are shared by both neighboring
/// slices so their geometries leave no gap. k=1 returns {trajectory}.
std::vector<PredictedTrajectory> SliceTrajectory(
    const PredictedTrajectory& trajectory, size_t k);

/// Index of the slice containing time t for a k-sliced horizon
/// [start, end] (clamped to [0, k-1]).
size_t SliceIndexAt(double t, double start, double end, size_t k);

/// One obstacle's frozen prediction over [StartTime, EndTime].
class Span {
 public:
  Span(PredictedTrajectory trajectory, SpanParams params);

  int32_t Id() const { return m_traj.id; }
  double StartTime() const { return m_traj.stamps.front(); }
  double EndTime() const { return m_traj.stamps.back(); }

  /// The trajectory this span's geometry was built from.
  const PredictedTrajectory& Trajectory() const { return m_traj; }

  /// Predicted center position at time t (linear interpolation;
  /// clamped to the span's ends). nullopt if the span is empty.
  std::optional<Vec3> PredictedPosition(double t) const;

  /// Per-axis envelope half-width at time t: halfExtents + k*sigma + slack.
  Vec3 EnvelopeHalfWidth(double t) const;

  /// Does an observation conform to this span?
  ///   - false if the observation is past EndTime (span exhausted);
  ///   - false if any axis deviation from the predicted position
  ///     exceeds the envelope half-width at that time.
  bool Conforms(const TrackedObstacle& observation) const;

  /// True once the observation time passes refreshFraction of the
  /// horizon -- callers should rebuild to restore lookahead even while
  /// the obstacle still conforms.
  bool NeedsRefresh(double t) const;

 private:
  PredictedTrajectory m_traj;
  SpanParams m_params;
};

}  // namespace spite_d
