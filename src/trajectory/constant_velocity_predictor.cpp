#include "trajectory/predictor.hpp"

namespace spite_d {

PredictedTrajectory ConstantVelocityPredictor::Predict(
    const TrackedObstacle& track, double horizon, double dt) const {
  PredictedTrajectory out;
  out.id = track.id;
  out.halfExtents = track.halfExtents;

  const size_t steps = dt > 0.0 ? static_cast<size_t>(horizon / dt) : 0;
  out.stamps.reserve(steps + 1);
  out.poses.reserve(steps + 1);
  out.positionStd.reserve(steps + 1);

  for (size_t i = 0; i <= steps; ++i) {
    const double dtI = i * dt;
    Pose3 pose = track.pose;
    for (int a = 0; a < 3; ++a)
      pose.translation[a] += track.velocity[a] * dtI;

    Vec3 std = track.positionStd;
    for (int a = 0; a < 3; ++a)
      std[a] += m_stdGrowthRate * dtI;

    out.stamps.push_back(track.stamp + dtI);
    out.poses.push_back(pose);
    out.positionStd.push_back(std);
  }
  return out;
}

}  // namespace spite_d
