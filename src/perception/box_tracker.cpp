#include "perception/box_tracker.hpp"

#include <algorithm>
#include <cmath>

namespace spite_d {

namespace {

/// Camera-frame point (+x right, +y down, +z forward) to world.
Vec3 CameraToWorld(const Pose3& cameraPose, float x, float y, float z) {
  const Vec3 local{x, y, z};
  Vec3 world;
  for (int r = 0; r < 3; ++r)
    world[r] = cameraPose.rotation[r][0] * local[0] +
               cameraPose.rotation[r][1] * local[1] +
               cameraPose.rotation[r][2] * local[2] +
               cameraPose.translation[r];
  return world;
}

/// map_manager's association test: bird-view overlap ratio above the
/// threshold, or center distance below half the summed diagonal.
bool Associated(const cv::Rect& a, const cv::Rect& b, float overlapThreshold) {
  const cv::Rect overlap = a & b;
  const float ratio =
      std::max(overlap.area() / float(a.area()), overlap.area() / float(b.area()));
  if (ratio >= overlapThreshold) return true;

  const float dx = (a.x + 0.5f * a.width) - (b.x + 0.5f * b.width);
  const float dy = (a.y + 0.5f * a.height) - (b.y + 0.5f * b.height);
  const float dist = std::sqrt(dx * dx + dy * dy);
  const float metric = std::sqrt(std::pow(a.width + b.width, 2.0f) +
                                 std::pow(a.height + b.height, 2.0f)) /
                       2.0f;
  return dist <= metric;
}

} 

BoxTracker::BoxTracker(const Params& params)
    : m_params(params), m_detector(params.detector) {}

void BoxTracker::PredictAndCorrect(Track& track,
                                   const std::array<double, 2>& zPos,
                                   const std::array<double, 2>& zVel,
                                   double dt) {
  // Constant-velocity predict: x += v*dt, P += Q (diagonal Q).
  track.kf[0] += track.kf[2] * dt;
  track.kf[1] += track.kf[3] * dt;
  // P = A P A^T + Q for A = [[I, dt*I],[0, I]].
  auto& P = track.kfP;
  for (int i = 0; i < 2; ++i) {
    P[i][i] += dt * (P[i + 2][i] + P[i][i + 2]) + dt * dt * P[i + 2][i + 2];
    P[i][i + 2] += dt * P[i + 2][i + 2];
    P[i + 2][i] += dt * P[i + 2][i + 2];
  }
  for (int i = 0; i < 4; ++i) P[i][i] += m_params.q;

  // Correct with the full-state measurement z = (cx, cy, vx, vy),
  // diagonal R: per-component scalar Kalman gain (H = I).
  const std::array<double, 4> z{zPos[0], zPos[1], zVel[0], zVel[1]};
  for (int i = 0; i < 4; ++i) {
    const double k = P[i][i] / (P[i][i] + m_params.r);
    track.kf[i] += k * (z[i] - track.kf[i]);
    P[i][i] *= (1.0 - k);
  }
}

std::vector<TrackedObstacle> BoxTracker::Update(const DepthFrame& frame) {
  // Wrap the borrowed depth buffer without copying.
  const cv::Mat depth(frame.height, frame.width, CV_16UC1,
                      const_cast<uint16_t*>(frame.depth));

  // The detector uses the frame's intrinsics.
  m_params.detector.fx = frame.fx;
  m_params.detector.fy = frame.fy;
  m_params.detector.px = frame.cx;
  m_params.detector.py = frame.cy;
  m_detector = UvDetector(m_params.detector);
  const auto& detections = m_detector.Detect(depth);

  const double dt =
      m_lastStamp < 0.0 ? 0.0 : std::max(1e-6, frame.stamp - m_lastStamp);
  m_lastStamp = frame.stamp;

  // ---- Greedy association of detections to existing tracks.
  std::vector<bool> trackMatched(m_tracks.size(), false);
  std::vector<int> detectionTrack(detections.size(), -1);

  for (size_t d = 0; d < detections.size(); ++d) {
    for (size_t t = 0; t < m_tracks.size(); ++t) {
      if (trackMatched[t]) continue;
      if (Associated(detections[d].birdRect, m_tracks[t].birdRect,
                     m_params.overlapThreshold)) {
        trackMatched[t] = true;
        detectionTrack[d] = int(t);
        break;
      }
    }
  }

  // ---- Update matched tracks / spawn new ones.
  std::vector<Track> nextTracks;
  nextTracks.reserve(m_tracks.size() + detections.size());

  for (size_t d = 0; d < detections.size(); ++d) {
    const CameraFrameBox& det = detections[d];
    const Vec3 center =
        CameraToWorld(frame.cameraPose, det.x, det.y, det.z);
    // Axis-aligned world extents: camera x -> world via rotation; with
    // the identity-yaw convention of the mentor pipeline we keep the
    // camera-frame extents mapped through absolute rotation columns.
    Vec3 extents{0, 0, 0};
    {
      const Vec3 camExt{det.xWidth, det.yWidth, det.zWidth};
      for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
          extents[r] += std::abs(frame.cameraPose.rotation[r][c]) * camExt[c];
    }

    Track* track = nullptr;
    if (detectionTrack[d] >= 0) {
      nextTracks.push_back(m_tracks[detectionTrack[d]]);
      track = &nextTracks.back();
      track->missed = 0;
      ++track->age;

      const std::array<double, 2> zPos{center[0], center[1]};
      const std::array<double, 2> zVel{
          dt > 0.0 ? (center[0] - track->kf[0]) / dt : 0.0,
          dt > 0.0 ? (center[1] - track->kf[1]) / dt : 0.0};
      PredictAndCorrect(*track, zPos, zVel, dt);
    } else {
      nextTracks.push_back({});
      track = &nextTracks.back();
      track->id = m_nextId++;
      track->age = 1;
      track->kf = {center[0], center[1], 0.0, 0.0};
      for (auto& row : track->kfP) row.fill(0.0);
      for (int i = 0; i < 4; ++i) track->kfP[i][i] = 1.0;
    }
    track->birdRect = det.birdRect;

    // Size fixing: freeze extents once the track has been seen long
    // enough (map_manager fixes after its history fills).
    if (!track->sizeFixed && track->age >= m_params.sizeFixFrames) {
      track->fixedExtents = extents;
      track->sizeFixed = true;
    }
    const Vec3 outExtents = track->sizeFixed ? track->fixedExtents : extents;

    // Velocity: running average of recent Kalman velocities.
    track->velocityHistory.push_back({track->kf[2], track->kf[3]});
    while (int(track->velocityHistory.size()) > m_params.velocityAvgWindow)
      track->velocityHistory.pop_front();
    double vx = 0.0, vy = 0.0;
    for (const auto& v : track->velocityHistory) {
      vx += v[0];
      vy += v[1];
    }
    vx /= track->velocityHistory.size();
    vy /= track->velocityHistory.size();

    TrackedObstacle& out = track->latest;
    out.id = track->id;
    out.stamp = frame.stamp;
    out.pose.translation = {track->kf[0], track->kf[1], center[2]};
    out.halfExtents = {outExtents[0] / 2, outExtents[1] / 2, outExtents[2] / 2};
    out.velocity = {vx, vy, 0.0};
    out.positionStd = {std::sqrt(track->kfP[0][0]), std::sqrt(track->kfP[1][1]),
                       0.0};
  }

  // ---- Keep unmatched tracks alive briefly (coast), then drop.
  for (size_t t = 0; t < m_tracks.size(); ++t) {
    if (trackMatched[t]) continue;
    Track track = m_tracks[t];
    if (++track.missed > m_params.maxMissedFrames) continue;
    // Coast on constant velocity.
    track.kf[0] += track.kf[2] * dt;
    track.kf[1] += track.kf[3] * dt;
    track.latest.pose.translation[0] = track.kf[0];
    track.latest.pose.translation[1] = track.kf[1];
    track.latest.stamp = frame.stamp;
    nextTracks.push_back(std::move(track));
  }

  m_tracks = std::move(nextTracks);

  std::vector<TrackedObstacle> out;
  out.reserve(m_tracks.size());
  for (const Track& t : m_tracks) out.push_back(t.latest);
  return out;
}

}  // namespace spite_d
