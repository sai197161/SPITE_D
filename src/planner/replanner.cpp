#include "planner/replanner.hpp"

#include <limits>
#include <queue>

namespace spite_d {

Replanner::Replanner(size_t numVertices, const std::vector<Edge>& edges)
    : m_numVertices(numVertices), m_adjacency(numVertices) {
  for (const Edge& e : edges) {
    m_adjacency[e.src].push_back(e);
    m_adjacency[e.tgt].push_back({e.tgt, e.src, e.cost});
  }
}

std::vector<Replanner::VID> Replanner::Plan(VID start, VID goal,
                                            const EdgeValidFn& isValid) const {
  constexpr double kInf = std::numeric_limits<double>::infinity();
  std::vector<double> dist(m_numVertices, kInf);
  std::vector<VID> parent(m_numVertices, m_numVertices);

  using QEntry = std::pair<double, VID>;
  std::priority_queue<QEntry, std::vector<QEntry>, std::greater<>> queue;

  dist[start] = 0.0;
  queue.push({0.0, start});

  while (!queue.empty()) {
    auto [d, v] = queue.top();
    queue.pop();
    if (d > dist[v]) continue;
    if (v == goal) break;

    for (const Edge& e : m_adjacency[v]) {
      if (!isValid(e.src, e.tgt)) continue;
      const double nd = d + e.cost;
      if (nd < dist[e.tgt]) {
        dist[e.tgt] = nd;
        parent[e.tgt] = v;
        queue.push({nd, e.tgt});
      }
    }
  }

  if (dist[goal] == kInf) return {};

  std::vector<VID> path;
  for (VID v = goal; v != start; v = parent[v]) path.push_back(v);
  path.push_back(start);
  return {path.rbegin(), path.rend()};
}

bool Replanner::PathBlocked(const std::vector<VID>& path,
                            const EdgeValidFn& isValid) const {
  for (size_t i = 0; i + 1 < path.size(); ++i)
    if (!isValid(path[i], path[i + 1])) return true;
  return false;
}

}  // namespace spite_d
