// Synthetic-depth test of the perception core (UvDetector + BoxTracker).
//
// Renders a box obstacle analytically into 16UC1 depth frames (front
// face projected through the pinhole model, background empty), then
// checks that the tracker recovers position, velocity, and a stable
// track id against the known ground truth. Camera pose is identity, so
// the world frame equals the camera frame (+x right, +y down, +z fwd).

#include "perception/box_tracker.hpp"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <vector>

namespace {

constexpr int kWidth = 640;
constexpr int kHeight = 480;
constexpr double kFx = 608.0, kFy = 608.0, kCx = 320.0, kCy = 240.0;

struct GroundTruthBox {
  double x, y, z;        // center, meters
  double hx, hy, hz;     // half extents
};

/// Paint the box's front face into the depth buffer (millimeters).
void Render(std::vector<uint16_t>& depth, const GroundTruthBox& box) {
  std::fill(depth.begin(), depth.end(), 0);
  const double zFront = box.z - box.hz;
  const int u0 = int(kFx * (box.x - box.hx) / zFront + kCx);
  const int u1 = int(kFx * (box.x + box.hx) / zFront + kCx);
  const int v0 = int(kFy * (box.y - box.hy) / zFront + kCy);
  const int v1 = int(kFy * (box.y + box.hy) / zFront + kCy);
  const uint16_t mm = uint16_t(zFront * 1000.0);
  for (int v = std::max(0, v0); v <= std::min(kHeight - 1, v1); ++v)
    for (int u = std::max(0, u0); u <= std::min(kWidth - 1, u1); ++u)
      depth[v * kWidth + u] = mm;
}

spite_d::DepthFrame MakeFrame(const std::vector<uint16_t>& depth,
                              double stamp) {
  spite_d::DepthFrame frame;
  frame.width = kWidth;
  frame.height = kHeight;
  frame.fx = kFx;
  frame.fy = kFy;
  frame.cx = kCx;
  frame.cy = kCy;
  frame.stamp = stamp;
  frame.depth = depth.data();
  return frame;
}

}  // namespace

int main() {
  using namespace spite_d;

  BoxTracker::Params params;
  params.detector.depthScale = 1000.0;
  BoxTracker tracker(params);

  // A 1.2m-tall box at z=2.5m moving +x at 0.5 m/s, 10 Hz for 20 frames.
  const double kVx = 0.5, kDt = 0.1;
  GroundTruthBox gt{-0.4, 0.0, 2.5, 0.3, 0.6, 0.2};

  std::vector<uint16_t> depth(kWidth * kHeight);
  std::vector<TrackedObstacle> tracks;
  int32_t firstId = -1;

  for (int f = 0; f < 20; ++f) {
    const double t = f * kDt;
    GroundTruthBox at = gt;
    at.x += kVx * t;
    Render(depth, at);
    tracks = tracker.Update(MakeFrame(depth, t));

    assert(tracks.size() == 1);  // Exactly one obstacle, every frame.
    if (f == 0) firstId = tracks[0].id;
    assert(tracks[0].id == firstId);  // Stable id throughout.
  }

  // Final ground truth: x = -0.4 + 0.5*1.9 = 0.55.
  const auto& obstacle = tracks[0];
  assert(std::abs(obstacle.pose.translation[0] - 0.55) < 0.15);
  assert(std::abs(obstacle.pose.translation[1] - 0.0) < 0.20);
  assert(std::abs(obstacle.pose.translation[2] - 2.5) < 0.40);

  // Velocity converged to ~0.5 m/s along +x, ~0 along y.
  assert(std::abs(obstacle.velocity[0] - kVx) < 0.15);
  assert(std::abs(obstacle.velocity[1]) < 0.15);

  // Extents in the right ballpark (U-map quantization is coarse).
  assert(std::abs(obstacle.halfExtents[0] - 0.3) < 0.15);
  assert(std::abs(obstacle.halfExtents[1] - 0.6) < 0.25);

  // ---- Two obstacles get distinct stable ids.
  BoxTracker tracker2(params);
  GroundTruthBox a{-0.8, 0.0, 2.0, 0.25, 0.5, 0.2};
  GroundTruthBox b{0.9, 0.0, 3.5, 0.3, 0.6, 0.2};
  std::vector<uint16_t> depth2(kWidth * kHeight);
  for (int f = 0; f < 5; ++f) {
    std::fill(depth2.begin(), depth2.end(), 0);
    // Render both (later render wins where they overlap; they don't).
    std::vector<uint16_t> tmp(kWidth * kHeight);
    Render(depth2, a);
    Render(tmp, b);
    for (size_t i = 0; i < depth2.size(); ++i)
      if (tmp[i]) depth2[i] = tmp[i];
    tracks = tracker2.Update(MakeFrame(depth2, f * kDt));
    assert(tracks.size() == 2);
  }
  assert(tracks[0].id != tracks[1].id);

  return 0;
}
