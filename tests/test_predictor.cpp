#include "trajectory/predictor.hpp"

#include <cassert>
#include <cmath>

int main() {
  using namespace spite_d;

  TrackedObstacle track;
  track.id = 7;
  track.stamp = 100.0;
  track.pose.translation = {1.0, 2.0, 0.5};
  track.velocity = {0.5, -1.0, 0.0};
  track.halfExtents = {0.3, 0.3, 0.9};
  track.positionStd = {0.05, 0.05, 0.05};

  ConstantVelocityPredictor predictor(/*stdGrowthRate=*/0.1);
  const PredictedTrajectory traj = predictor.Predict(track, /*horizon=*/2.0,
                                                     /*dt=*/0.5);

  assert(traj.id == 7);
  assert(traj.stamps.size() == 5);  // t = 0, 0.5, 1.0, 1.5, 2.0
  assert(traj.poses.size() == 5);
  assert(traj.positionStd.size() == 5);

  // First sample is the current state.
  assert(std::abs(traj.stamps.front() - 100.0) < 1e-12);
  assert(std::abs(traj.poses.front().translation[0] - 1.0) < 1e-12);

  // Last sample: p + v * 2.
  assert(std::abs(traj.poses.back().translation[0] - 2.0) < 1e-12);
  assert(std::abs(traj.poses.back().translation[1] - 0.0) < 1e-12);
  assert(std::abs(traj.poses.back().translation[2] - 0.5) < 1e-12);

  // Uncertainty grows linearly: 0.05 + 0.1 * 2.
  assert(std::abs(traj.positionStd.back()[0] - 0.25) < 1e-12);

  // Zero growth rate recovers the fixed-horizon scheme.
  ConstantVelocityPredictor fixed(0.0);
  const PredictedTrajectory fixedTraj = fixed.Predict(track, 2.0, 0.5);
  assert(std::abs(fixedTraj.positionStd.back()[0] - 0.05) < 1e-12);

  return 0;
}
