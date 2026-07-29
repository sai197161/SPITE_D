#include "spite/validity_server.hpp"

#include "DynamicRoadmapTool.h"
#include "Geometry/SimpleGeometries.h"
#include "OverApprox.h"

#include <algorithm>
#include <map>
#include <set>
#include <unordered_map>
#include <vector>

namespace spite_d {

namespace {

using Point3d = DynamicRoadmapTool::Point3d;

/// World-frame corners of an OBB given center pose and (possibly
/// inflated) half extents.
void AppendCorners(const Pose3& pose, const Vec3& half,
                   open_spite::PointCloud3d& out) {
  for (int sx : {-1, 1})
    for (int sy : {-1, 1})
      for (int sz : {-1, 1}) {
        const Vec3 local{sx * half[0], sy * half[1], sz * half[2]};
        Vec3 world;
        for (int r = 0; r < 3; ++r)
          world[r] = pose.rotation[r][0] * local[0] +
                     pose.rotation[r][1] * local[1] +
                     pose.rotation[r][2] * local[2] + pose.translation[r];
        out.emplace_back(world[0], world[1], world[2]);
      }
}

ValidityServer::Validity FromTool(DynamicRoadmapTool::Validity v) {
  switch (v) {
    case DynamicRoadmapTool::Validity::VALID:
      return ValidityServer::Validity::VALID;
    case DynamicRoadmapTool::Validity::INVALID:
      return ValidityServer::Validity::INVALID;
    default:
      return ValidityServer::Validity::UNKNOWN;
  }
}

}  // namespace

struct ValidityServer::Impl {
  std::unique_ptr<DynamicRoadmapTool> drm;
  Params params;
  std::unordered_map<int32_t, size_t> trackToSlot;
  /// Slice-mode slots, keyed by (track id, slice index).
  std::map<std::pair<int32_t, size_t>, size_t> sliceSlots;
  size_t nextSlot{0};
  /// Slots released by Forget/RetainOnly, available for reuse. Without
  /// this, track-id churn allocates a fresh slot per id forever and the
  /// reconciliation pass grows without bound.
  std::vector<size_t> freeSlots;

  size_t AcquireSlot() {
    if (freeSlots.empty()) return nextSlot++;
    const size_t slot = freeSlots.back();
    freeSlots.pop_back();
    return slot;
  }

  void ReleaseSlot(size_t slot) {
    ParkSlot(slot);
    freeSlots.push_back(slot);
  }

  /// Build one slot's SPITE geometry from one trajectory (whole-horizon
  /// or a slice -- same +/-k*sigma construction either way).
  void BuildSlotGeometry(size_t slot, const PredictedTrajectory& traj) {
    // Over-approximation: OBB of the predicted OBB corners, +k*sigma.
    open_spite::PointCloud3d cloud;
    cloud.reserve(traj.poses.size() * 8);
    for (size_t i = 0; i < traj.poses.size(); ++i) {
      Vec3 half = traj.halfExtents;
      if (i < traj.positionStd.size())
        for (int a = 0; a < 3; ++a)
          half[a] += params.sigmaGain * traj.positionStd[i][a];
      AppendCorners(traj.poses[i], half, cloud);
    }
    OrientedBoundingBox obb;
    open_spite::GetBoundingBox(cloud, obb);
    drm->UpdateObstacleOBB(slot, obb);

    // Under-approximation: inscribed spheres along the path, -k*sigma;
    // vanished radii contribute nothing (no false REDs).
    const double baseRadius = *std::min_element(traj.halfExtents.begin(),
                                                traj.halfExtents.end());
    std::vector<DynamicRoadmapTool::Sphere> spheres;
    spheres.reserve(traj.poses.size());
    for (size_t i = 0; i < traj.poses.size(); ++i) {
      double sigma = 0.0;
      if (i < traj.positionStd.size())
        sigma = *std::max_element(traj.positionStd[i].begin(),
                                  traj.positionStd[i].end());
      const double radius = baseRadius - params.sigmaGain * sigma;
      if (radius <= 0.0) continue;
      const auto& t = traj.poses[i].translation;
      spheres.push_back({Point3d(t[0], t[1], t[2]), radius});
    }
    drm->UpdateObstacleSpheres(slot, spheres);
  }

  /// Empty a slot so reconciliation drops its contributions. (A far
  /// AABB, not a default OBB: cornerless OBBs crash open-spite queries.)
  void ParkSlot(size_t slot) {
    constexpr double kFar = 1e9;
    drm->UpdateObstacleAABB(
        slot, DynamicRoadmapTool::Box3d(Point3d(kFar, kFar, kFar),
                                        Point3d(kFar + 1, kFar + 1, kFar + 1)));
    drm->UpdateObstacleSpheres(slot, {});
  }
};

ValidityServer::ValidityServer(std::unique_ptr<DynamicRoadmapTool> drm,
                               Params params)
    : m_impl(new Impl{std::move(drm), params, {}, {}, 0}) {}

ValidityServer::~ValidityServer() = default;

void ValidityServer::Update(
    const std::vector<PredictedTrajectory>& predictions) {
  auto& impl = *m_impl;

  for (const PredictedTrajectory& traj : predictions) {
    if (traj.poses.empty()) continue;
    auto it = impl.trackToSlot.find(traj.id);
    if (it == impl.trackToSlot.end())
      it = impl.trackToSlot.emplace(traj.id, impl.AcquireSlot()).first;
    impl.BuildSlotGeometry(it->second, traj);
  }

  impl.drm->ShallowUpdate();
}

void ValidityServer::UpdateSlices(
    int32_t obstacleId, const std::vector<PredictedTrajectory>& slices) {
  auto& impl = *m_impl;
  for (size_t s = 0; s < slices.size(); ++s) {
    if (slices[s].poses.empty()) continue;
    auto it = impl.sliceSlots.find({obstacleId, s});
    if (it == impl.sliceSlots.end())
      it = impl.sliceSlots.emplace(std::make_pair(obstacleId, s),
                                   impl.AcquireSlot()).first;
    impl.BuildSlotGeometry(it->second, slices[s]);
  }
  impl.drm->ShallowUpdate();
}

void ValidityServer::ExpireSlices(int32_t obstacleId,
                                  size_t firstActiveSlice) {
  auto& impl = *m_impl;
  bool any = false;
  for (auto& [key, slot] : impl.sliceSlots) {
    if (key.first != obstacleId || key.second >= firstActiveSlice) continue;
    impl.ParkSlot(slot);
    any = true;
  }
  // Bookkeeping only: parked slots are re-reconciled, no tree queries
  // against real geometry. This is the cheap-expiry win of slicing.
  if (any) impl.drm->ShallowUpdate();
}

void ValidityServer::Forget(int32_t obstacleId) {
  auto& impl = *m_impl;
  bool any = false;

  auto it = impl.trackToSlot.find(obstacleId);
  if (it != impl.trackToSlot.end()) {
    impl.ReleaseSlot(it->second);
    impl.trackToSlot.erase(it);
    any = true;
  }
  for (auto sit = impl.sliceSlots.begin(); sit != impl.sliceSlots.end();) {
    if (sit->first.first == obstacleId) {
      impl.ReleaseSlot(sit->second);
      sit = impl.sliceSlots.erase(sit);
      any = true;
    } else {
      ++sit;
    }
  }
  if (any) impl.drm->ShallowUpdate();
}

void ValidityServer::RetainOnly(const std::vector<int32_t>& activeIds) {
  auto& impl = *m_impl;
  const std::set<int32_t> active(activeIds.begin(), activeIds.end());
  bool any = false;

  for (auto it = impl.trackToSlot.begin(); it != impl.trackToSlot.end();) {
    if (active.count(it->first)) {
      ++it;
      continue;
    }
    impl.ReleaseSlot(it->second);
    it = impl.trackToSlot.erase(it);
    any = true;
  }
  for (auto it = impl.sliceSlots.begin(); it != impl.sliceSlots.end();) {
    if (active.count(it->first.first)) {
      ++it;
      continue;
    }
    impl.ReleaseSlot(it->second);
    it = impl.sliceSlots.erase(it);
    any = true;
  }
  // One reconciliation pass no matter how many tracks were dropped.
  if (any) impl.drm->ShallowUpdate();
}

ValidityServer::Validity ValidityServer::GetVertexValidity(size_t vid) const {
  return FromTool(m_impl->drm->GetVertexValidity(vid));
}

ValidityServer::Validity ValidityServer::GetEdgeValidity(size_t src,
                                                         size_t tgt) const {
  return FromTool(m_impl->drm->GetEdgeValidity(src, tgt));
}

std::vector<std::pair<size_t, size_t>> ValidityServer::BlockedEdges() const {
  std::vector<std::pair<size_t, size_t>> blocked;
  for (const auto& [element, geoms] : m_impl->drm->GetElementMap()) {
    if (element.size() != 2) continue;
    if (FromTool(m_impl->drm->GetEdgeValidity(element[0], element[1])) !=
        Validity::VALID)
      blocked.emplace_back(element[0], element[1]);
  }
  return blocked;
}

}  // namespace spite_d
