#pragma once

// Multi-object tracking interface for the perception module.

#include "common/types.hpp"

#include <vector>

namespace spite_d {

/// Minimal depth-frame handoff: row-major uint16 depth in millimeters
/// plus the intrinsics needed to deproject. Matches what a
/// sensor_msgs/Image (16UC1) + CameraInfo pair carries.
struct DepthFrame {
  int width{0};
  int height{0};
  double fx{0}, fy{0}, cx{0}, cy{0};
  double stamp{0.0};
  Pose3 cameraPose;  ///< Camera-to-world transform
  const uint16_t* depth{nullptr}; 
};

class Tracker {
 public:
  virtual ~Tracker() = default;

  /// input one frame and output the current set of tracked obstacles
  /// (world frame, with velocities and covariances).
  virtual std::vector<TrackedObstacle> Update(const DepthFrame& frame) = 0;
};

}  // namespace spite_d
