#include "trajectory/span.hpp"

#include <algorithm>
#include <cmath>

namespace spite_d {

std::vector<PredictedTrajectory> SliceTrajectory(
    const PredictedTrajectory& trajectory, size_t k) {
  if (k <= 1 || trajectory.stamps.size() < 2) return {trajectory};

  const double start = trajectory.stamps.front();
  const double end = trajectory.stamps.back();
  const double width = (end - start) / double(k);

  std::vector<PredictedTrajectory> slices(k);
  for (size_t s = 0; s < k; ++s) {
    slices[s].id = trajectory.id;
    slices[s].halfExtents = trajectory.halfExtents;
  }
  for (size_t i = 0; i < trajectory.stamps.size(); ++i) {
    const double t = trajectory.stamps[i];
    // A sample belongs to every slice whose closed interval contains it
    // (boundary samples land in two slices; no geometric gaps).
    const double rel = (t - start) / width;
    const size_t lo =
        std::min(size_t(std::floor(rel - 1e-9 < 0 ? 0 : rel - 1e-9)), k - 1);
    const size_t hi = std::min(size_t(std::floor(rel + 1e-9)), k - 1);
    for (size_t s = lo; s <= hi; ++s) {
      slices[s].stamps.push_back(t);
      slices[s].poses.push_back(trajectory.poses[i]);
      if (i < trajectory.positionStd.size())
        slices[s].positionStd.push_back(trajectory.positionStd[i]);
    }
  }
  return slices;
}

size_t SliceIndexAt(double t, double start, double end, size_t k) {
  if (k <= 1 || end <= start) return 0;
  const double rel = (t - start) / (end - start) * double(k);
  if (rel <= 0.0) return 0;
  const size_t idx = size_t(rel);
  return std::min(idx, k - 1);
}

Span::Span(PredictedTrajectory trajectory, SpanParams params)
    : m_traj(std::move(trajectory)), m_params(params) {}

std::optional<Vec3> Span::PredictedPosition(double t) const {
  const auto& stamps = m_traj.stamps;
  if (stamps.empty()) return std::nullopt;

  if (t <= stamps.front()) return m_traj.poses.front().translation;
  if (t >= stamps.back()) return m_traj.poses.back().translation;

  const auto upper = std::upper_bound(stamps.begin(), stamps.end(), t);
  const size_t hi = size_t(upper - stamps.begin());
  const size_t lo = hi - 1;
  const double span = stamps[hi] - stamps[lo];
  const double alpha = span > 0.0 ? (t - stamps[lo]) / span : 0.0;

  Vec3 p;
  for (int a = 0; a < 3; ++a)
    p[a] = (1.0 - alpha) * m_traj.poses[lo].translation[a] +
           alpha * m_traj.poses[hi].translation[a];
  return p;
}

Vec3 Span::EnvelopeHalfWidth(double t) const {
  // Sigma at the nearest sample at-or-after t (conservative: sigma is
  // non-decreasing in lookahead for any sane predictor).
  Vec3 sigma{0, 0, 0};
  if (!m_traj.positionStd.empty()) {
    const auto upper =
        std::lower_bound(m_traj.stamps.begin(), m_traj.stamps.end(), t);
    size_t idx = size_t(upper - m_traj.stamps.begin());
    idx = std::min(idx, m_traj.positionStd.size() - 1);
    sigma = m_traj.positionStd[idx];
  }
  Vec3 half;
  for (int a = 0; a < 3; ++a)
    half[a] = m_traj.halfExtents[a] + m_params.sigmaGain * sigma[a] +
              m_params.slack;
  return half;
}

bool Span::Conforms(const TrackedObstacle& observation) const {
  if (m_traj.stamps.empty()) return false;
  if (observation.stamp > EndTime()) return false;  // Exhausted.

  const auto predicted = PredictedPosition(observation.stamp);
  if (!predicted) return false;

  const Vec3 half = EnvelopeHalfWidth(observation.stamp);
  for (int a = 0; a < 3; ++a) {
    if (std::abs(observation.pose.translation[a] - (*predicted)[a]) > half[a])
      return false;
  }
  return true;
}

bool Span::NeedsRefresh(double t) const {
  if (m_traj.stamps.empty()) return true;
  const double horizon = EndTime() - StartTime();
  if (horizon <= 0.0) return true;
  return (t - StartTime()) >= m_params.refreshFraction * horizon;
}

}  // namespace spite_d
