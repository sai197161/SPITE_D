#pragma once

// Replanner: shortest-path search over the currently-valid subgraph of
// the roadmap, with replanning triggered when the validity server
// invalidates an edge on the active path.
//
// Deliberately simple (plain A* / Dijkstra over the whole roadmap) --
// replanning quality is not the contribution of this package, so keep
// this a stable baseline. Incremental repair (D*-Lite) can be swapped
// in later behind the same interface if it ever becomes the bottleneck.

#include <cstddef>
#include <functional>
#include <vector>

namespace spite_d {

class Replanner {
 public:
  using VID = size_t;

  struct Edge {
    VID src;
    VID tgt;
    double cost;
  };

  /// Predicate answering "is this edge currently traversable". Wired to
  /// ValidityServer::GetEdgeValidity; UNKNOWN edges are treated as
  /// traversable-but-must-fine-check (lazy) or blocked, per policy.
  using EdgeValidFn = std::function<bool(VID, VID)>;

  Replanner(size_t numVertices, const std::vector<Edge>& edges);

  /// Shortest path from start to goal using only edges the predicate
  /// admits. Empty result = no path in the current valid subgraph.
  std::vector<VID> Plan(VID start, VID goal, const EdgeValidFn& isValid) const;

  /// True if any edge of the given path is rejected by the predicate --
  /// the trigger for replanning.
  bool PathBlocked(const std::vector<VID>& path, const EdgeValidFn& isValid) const;

 private:
  size_t m_numVertices;
  std::vector<std::vector<Edge>> m_adjacency;
};

}  // namespace spite_d
