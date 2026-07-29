#include "planner/roadmap_io.hpp"

#include <fstream>

namespace spite_d {

namespace {
constexpr const char* kHeaderV1 = "spite_d roadmap v1";
constexpr const char* kHeaderV2 = "spite_d roadmap v2";
}

bool WriteRoadmapGraph(const RoadmapGraph& graph, const std::string& path) {
  std::ofstream out(path);
  if (!out) return false;

  const bool v2 = graph.dof > 0;
  if (v2 && graph.configurations.size() != graph.vertices.size()) return false;

  out.precision(17);
  out << (v2 ? kHeaderV2 : kHeaderV1) << "\n";
  if (v2) out << "dof " << graph.dof << "\n";
  out << "verts " << graph.vertices.size() << "\n";
  for (size_t i = 0; i < graph.vertices.size(); ++i) {
    const auto& v = graph.vertices[i];
    out << v.position[0] << " " << v.position[1] << " " << v.position[2];
    for (double q : v.quaternion) out << " " << q;
    if (v2)
      for (double j : graph.configurations[i]) out << " " << j;
    out << "\n";
  }
  out << "edges " << graph.edges.size() << "\n";
  for (const auto& e : graph.edges)
    out << e.src << " " << e.tgt << " " << e.cost << "\n";
  return static_cast<bool>(out);
}

bool ReadRoadmapGraph(RoadmapGraph& graph, const std::string& path) {
  std::ifstream in(path);
  if (!in) return false;

  std::string header;
  std::getline(in, header);
  const bool v2 = header == kHeaderV2;
  if (!v2 && header != kHeaderV1) return false;

  std::string token;
  size_t count = 0;

  graph.dof = 0;
  graph.configurations.clear();
  if (v2) {
    if (!(in >> token >> graph.dof) || token != "dof" || graph.dof == 0)
      return false;
  }

  if (!(in >> token >> count) || token != "verts") return false;
  graph.vertices.assign(count, {});
  if (v2) graph.configurations.assign(count, std::vector<double>(graph.dof));
  for (size_t i = 0; i < count; ++i) {
    auto& v = graph.vertices[i];
    if (!(in >> v.position[0] >> v.position[1] >> v.position[2] >>
          v.quaternion[0] >> v.quaternion[1] >> v.quaternion[2] >>
          v.quaternion[3]))
      return false;
    if (v2)
      for (double& j : graph.configurations[i])
        if (!(in >> j)) return false;
  }

  if (!(in >> token >> count) || token != "edges") return false;
  graph.edges.assign(count, {});
  for (auto& e : graph.edges) {
    if (!(in >> e.src >> e.tgt >> e.cost)) return false;
    if (e.src >= graph.vertices.size() || e.tgt >= graph.vertices.size())
      return false;
  }
  return true;
}

}
