#pragma once

// Roadmap graph persistence.
//
// The offline build_roadmap tool writes two files:
//   - the graph (this module): vertex states + edges with costs, which the
//     replanner needs and open-spite's WriteGeometries does not store;
//   - the geometries (open-spite's format): OBB/spline approximations,
//     reloaded via DynamicRoadmapTool::ReadGeometriesFromFile.
// Loading the graph requires neither OMPL nor open-spite.

#include "common/types.hpp"
#include "planner/replanner.hpp"

#include <string>
#include <vector>

namespace spite_d {

struct RoadmapGraph {
  /// Vertex states indexed by VID. Rotation is a unit quaternion
  /// (x, y, z, w) to match the SE3 state it was sampled from. For
  /// articulated robots, position holds the end-effector workspace
  /// point (for visualization / nearest-vertex lookup) and the real
  /// state lives in configurations.
  struct VertexState {
    Vec3 position{0, 0, 0};
    std::array<double, 4> quaternion{0, 0, 0, 1};
  };

  std::vector<VertexState> vertices;
  std::vector<Replanner::Edge> edges;

  /// C-space dimension for articulated robots; 0 = rigid-body roadmap
  /// (v1 files). When nonzero, configurations[v] has dof joint values.
  size_t dof{0};
  std::vector<std::vector<double>> configurations;
};

/// Text format "spite_d roadmap v1" (rigid body) or v2 (adds dof +
/// per-vertex configurations). Writes v2 iff graph.dof > 0.
bool WriteRoadmapGraph(const RoadmapGraph& graph, const std::string& path);

/// Reads either version. Returns false if missing or malformed.
bool ReadRoadmapGraph(RoadmapGraph& graph, const std::string& path);

}
