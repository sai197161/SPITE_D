#pragma once

// Trajectory prediction interface.
//
// This is the seam where prediction strategies plug in: the basic
// constant-velocity extrapolation, TBD/TBQ variants, or learned
// predictors. Downstream consumers (spite validity updates, and later
// span construction in dynamic_map) depend only on this interface.

#include "common/types.hpp"

namespace spite_d {

class Predictor {
 public:
  virtual ~Predictor() = default;

  /// Predict the obstacle's trajectory over [track.stamp, track.stamp + horizon]
  /// sampled every dt seconds (the last sample lands on or before the
  /// horizon end).
  virtual PredictedTrajectory Predict(const TrackedObstacle& track,
                                      double horizon, double dt) const = 0;
};

/// Constant-velocity extrapolation with linearly growing uncertainty.
/// Mirrors the prediction implied by map_manager's Kalman tracker
/// (planar velocity, orientation held fixed).
class ConstantVelocityPredictor : public Predictor {
 public:
  /// @param stdGrowthRate Added position std (m/s of lookahead) on top
  ///        of the track's current positionStd. Zero recovers the pure
  ///        fixed-horizon scheme.
  explicit ConstantVelocityPredictor(double stdGrowthRate = 0.0)
      : m_stdGrowthRate(stdGrowthRate) {}

  PredictedTrajectory Predict(const TrackedObstacle& track, double horizon,
                              double dt) const override;

 private:
  double m_stdGrowthRate;
};

}  // namespace spite_d
