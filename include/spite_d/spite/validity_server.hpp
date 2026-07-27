#pragma once

// ValidityServer: the bridge between perception-driven obstacle
// predictions and open-spite's DynamicRoadmapTool.
//
// Baseline (no spans): every perception update, each obstacle's
// predicted trajectory over the horizon is collapsed into SPITE
// obstacle geometry --
//   over-approximation:  OBB of the predicted OBB corners across all
//                        prediction samples, inflated by +k*sigma;
//   under-approximation: spheres inscribed in the obstacle placed along
//                        the predicted center path, deflated by -k*sigma
// -- then fed to DynamicRoadmapTool::UpdateObstacleOBB/Spheres +
// MarkDirty + ShallowUpdate. This recomputes obstacle geometry every
// frame; the span representation in dynamic_map will later amortize
// exactly this step, so keep this path intact as the ablation baseline.
//
// Gray elements can optionally be resolved with a fine collision check
// (fcl) or left lazy for the replanner to resolve on demand.

#include "spite_d/common/types.hpp"

#include <map>
#include <memory>
#include <utility>
#include <vector>

class DynamicRoadmapTool;  // open-spite

namespace spite_d {

class ValidityServer {
 public:
  enum class Validity { VALID, UNKNOWN, INVALID };

  struct Params {
    double sigmaGain{0.0};   ///< k in the +/- k*sigma inflation/deflation.
    bool resolveGray{true};  ///< Fine-check gray elements immediately vs lazily.
  };

  /// Takes ownership of a DynamicRoadmapTool that has already been
  /// built (Build() or ReadGeometriesFromFile()).
  ValidityServer(std::unique_ptr<DynamicRoadmapTool> drm, Params params);
  ~ValidityServer();

  /// Ingest this frame's predictions and update roadmap validity.
  /// Tracks appearing for the first time are registered; missing tracks
  /// are treated as unchanged (obstacle removal handled via Forget()).
  void Update(const std::vector<PredictedTrajectory>& predictions);

  /// Time-sliced mode: register per-slice geometries for one obstacle
  /// (slice index = position in the vector). Each slice occupies its
  /// own slot, so classifications carry implicit intervals.
  void UpdateSlices(int32_t obstacleId,
                    const std::vector<PredictedTrajectory>& slices);

  /// Drop the contributions of all slices before firstActiveSlice --
  /// pure bookkeeping (no geometry queries), so edges blocked only by
  /// the obstacle's past heal for free as time advances.
  void ExpireSlices(int32_t obstacleId, size_t firstActiveSlice);

  /// Drop a track (target lost / left the workspace) and re-validate
  /// the roadmap elements it was invalidating (whole-horizon and
  /// slice slots alike). The freed slot is reused by the next new track.
  void Forget(int32_t obstacleId);

  /// Keep only these obstacle ids; drop every other tracked obstacle in
  /// a single reconciliation pass. Callers that receive a full snapshot
  /// of currently-visible obstacles each frame should use this: without
  /// it, an obstacle that leaves the scene keeps blocking the edges it
  /// last intersected for the lifetime of the process, and its slot is
  /// never released.
  void RetainOnly(const std::vector<int32_t>& activeIds);

  Validity GetVertexValidity(size_t vid) const;
  Validity GetEdgeValidity(size_t src, size_t tgt) const;

  /// Edges currently classified INVALID or UNKNOWN, for the replanner.
  std::vector<std::pair<size_t, size_t>> BlockedEdges() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> m_impl;
};

}  // namespace spite_d
