#pragma once

// Multi-object tracker over UvDetector output.
//
// Port of map_manager's association + velocity estimation, restructured
// around persistent tracks:
//
//   - Association: greedy frame-to-frame matching on bird's-view
//     rectangles by overlap ratio OR center distance (same test as
//     UVtracker::check_status).
//   - State: per-track 4-state Kalman filter (cx, cy, vx, vy) in the
//     WORLD frame with a constant-velocity model, matching
//     dynamicMap::updateTrackedStates. Velocity output is the running
//     average of the last `velocityAvgWindow` Kalman velocities.
//   - IDs: unlike map_manager (frame-index based), tracks carry stable
//     integer ids, required by TrackedObstacle and downstream span
//     bookkeeping. Tracks unmatched for `maxMissedFrames` are dropped.
//
// The world frame: detections are deprojected in the camera frame
// (+x right, +y down, +z forward) and transformed by the frame's
// cameraPose. Boxes are axis-aligned in the world frame (identity
// rotation), matching the mentor pipeline's box3D convention; oriented
// boxes from track history are a later refinement.

#include "common/types.hpp"
#include "perception/tracker.hpp"
#include "perception/uv_detector.hpp"

#include <cstdint>
#include <deque>
#include <vector>

namespace spite_d {

class BoxTracker : public Tracker {
 public:
  struct Params {
    UvDetector::Params detector;
    float overlapThreshold{0.4f};  ///< bird-view overlap ratio to match.
    int velocityAvgWindow{5};      ///< frames averaged for output velocity.
    int maxMissedFrames{3};        ///< drop a track after this many misses.
    int sizeFixFrames{10};         ///< freeze box size after this many frames.
    /// Kalman noise (world-frame meters): process / observation.
    double q{0.05}, r{0.1};
  };

  explicit BoxTracker(const Params& params);

  std::vector<TrackedObstacle> Update(const DepthFrame& frame) override;

 private:
  struct Track {
    int32_t id;
    cv::Rect birdRect;
    int missed{0};
    int age{0};
    // Kalman state (cx, cy, vx, vy) and covariance, world frame.
    std::array<double, 4> kf{};
    std::array<std::array<double, 4>, 4> kfP{};
    std::deque<std::array<double, 2>> velocityHistory;
    // Fixed size once the track matures (map_manager's "fix" behavior).
    Vec3 fixedExtents{0, 0, 0};
    bool sizeFixed{false};
    // Latest world-frame measurement-derived box.
    TrackedObstacle latest;
  };

  void PredictAndCorrect(Track& track, const std::array<double, 2>& measuredPos,
                         const std::array<double, 2>& measuredVel, double dt);

  Params m_params;
  UvDetector m_detector;
  std::vector<Track> m_tracks;
  int32_t m_nextId{0};
  double m_lastStamp{-1.0};
};

}  // namespace spite_d
