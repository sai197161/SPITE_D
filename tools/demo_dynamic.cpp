// Terminal demo + ablation of the dynamic-validity pipeline (no ROS,
// no simulator). Loads a roadmap produced by build_roadmap and drives
// a simulated obstacle across the initial path in one or both modes:
//
//   baseline -- every frame: predict + rebuild obstacle geometry +
//               re-query the roadmap trees (frame-by-frame SPITE).
//   spans    -- SpanPipeline: geometry frozen per horizon; each frame
//               costs a conformance check, with rebuilds only at spawn,
//               scheduled refresh, or prediction violation.
//
// Both modes see the identical obstacle motion, so the printed summary
// is the paper's ablation in miniature: same classifications and
// replans, orders fewer expensive updates.
//
// Usage: demo_dynamic --roadmap DIR [--frames N] [--hz F]
//                     [--mode baseline|spans|both] [--trace]

#include "spite_d/dynamic_map/span_pipeline.hpp"
#include "spite_d/planner/replanner.hpp"
#include "spite_d/planner/roadmap_io.hpp"
#include "spite_d/spite/validity_server.hpp"
#include "spite_d/trajectory/predictor.hpp"

#include "DynamicRoadmapTool.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace {

using namespace spite_d;

constexpr double kSigmaGain = 1.0;

struct Options {
  std::string dir;
  int frames = 40;
  double hz = 10.0;
  std::string mode = "both";
  size_t slices = 1;
  bool trace = false;
  std::string dumpJson;   // path for the viewer's data file
  size_t renderCap = 4000; // max edges written to JSON
};

/// The viewer cannot draw 60k edges smoothly, so pick a subset once and
/// share it across modes: every edge near the obstacle's swept path (the
/// ones that actually change label) plus a spatial sample for context.
struct RenderSet {
  std::vector<size_t> edgeIds;             // indices into graph.edges
  std::unordered_map<size_t, size_t> slot; // edge index -> position above

  static RenderSet Build(const RoadmapGraph& graph, const Vec3& aim,
                         double reach, size_t cap) {
    RenderSet rs;
    std::vector<size_t> near, far;
    for (size_t e = 0; e < graph.edges.size(); ++e) {
      const auto& a = graph.vertices[graph.edges[e].src].position;
      const auto& b = graph.vertices[graph.edges[e].tgt].position;
      const double da = std::sqrt(std::pow(a[0] - aim[0], 2) +
                                  std::pow(a[1] - aim[1], 2) +
                                  std::pow(a[2] - aim[2], 2));
      const double db = std::sqrt(std::pow(b[0] - aim[0], 2) +
                                  std::pow(b[1] - aim[1], 2) +
                                  std::pow(b[2] - aim[2], 2));
      (std::min(da, db) < reach ? near : far).push_back(e);
    }
    rs.edgeIds = near;
    // Deterministic stride sample of the rest, up to the cap.
    if (rs.edgeIds.size() < cap && !far.empty()) {
      const size_t want = cap - rs.edgeIds.size();
      const size_t stride = std::max<size_t>(1, far.size() / std::max<size_t>(1, want));
      for (size_t i = 0; i < far.size() && rs.edgeIds.size() < cap; i += stride)
        rs.edgeIds.push_back(far[i]);
    }
    if (rs.edgeIds.size() > cap) rs.edgeIds.resize(cap);
    for (size_t i = 0; i < rs.edgeIds.size(); ++i) rs.slot[rs.edgeIds[i]] = i;
    return rs;
  }
};

/// One frame of viewer state.
struct FrameDump {
  double t = 0;
  Vec3 obstacle{0, 0, 0};
  std::vector<size_t> red, gray;   // positions within RenderSet
  std::vector<size_t> path;
  bool replanned = false;
  bool rebuilt = false;
  double ms = 0;
  // Span envelope: frozen predicted centers + per-sample half widths.
  std::vector<Vec3> envCenters, envHalf;
};

struct ModeResult {
  std::string name;
  size_t geometryRebuilds = 0;
  size_t expiries = 0;
  int replans = 0;
  std::vector<double> updateMs;
  std::vector<size_t> pathSizes;
  std::vector<FrameDump> frames;

  double Mean() const {
    double s = 0;
    for (double v : updateMs) s += v;
    return updateMs.empty() ? 0 : s / updateMs.size();
  }
  double Median() const {
    if (updateMs.empty()) return 0;
    std::vector<double> v = updateMs;
    std::sort(v.begin(), v.end());
    return v[v.size() / 2];
  }
  double Max() const {
    return updateMs.empty() ? 0
                            : *std::max_element(updateMs.begin(), updateMs.end());
  }
};

std::unique_ptr<DynamicRoadmapTool> LoadDrm(const std::string& dir) {
  auto drm = std::make_unique<DynamicRoadmapTool>();
  drm->SetUseUnderApprox(true);
  if (!drm->ReadGeometriesFromFile(dir + "/roadmap_geoms.txt")) return nullptr;
  return drm;
}

/// Obstacle parameters scaled to the roadmap's workspace so the same
/// demo works for a 10m box world and a ~1.7m arm reach alike.
struct Scenario {
  Vec3 aimPoint;
  double obstacleHalf;
  double speed;

  static Scenario From(const RoadmapGraph& graph, const Vec3& aim) {
    Vec3 lo{1e30, 1e30, 1e30}, hi{-1e30, -1e30, -1e30};
    for (const auto& v : graph.vertices)
      for (int a = 0; a < 3; ++a) {
        lo[a] = std::min(lo[a], v.position[a]);
        hi[a] = std::max(hi[a], v.position[a]);
      }
    const double diag = std::sqrt(std::pow(hi[0] - lo[0], 2) +
                                  std::pow(hi[1] - lo[1], 2) +
                                  std::pow(hi[2] - lo[2], 2));
    // Ratios chosen to reproduce the original box-world scenario
    // (diag ~17 -> half 0.4, speed 1.2).
    return {aim, 0.024 * diag, 0.07 * diag};
  }
};

/// Runs one mode over the shared scenario; the obstacle is aimed at the
/// midpoint of the initial path and follows constant velocity exactly.
ModeResult RunMode(const std::string& name, const Options& opt,
                   const RoadmapGraph& graph, size_t start, size_t goal,
                   const Scenario& scenario, const RenderSet* render) {
  ModeResult result;
  result.name = name;

  auto drm = LoadDrm(opt.dir);
  ValidityServer server(std::move(drm), {kSigmaGain});
  Replanner planner(graph.vertices.size(), graph.edges);
  ConstantVelocityPredictor predictor(/*stdGrowthRate=*/0.1);

  SpanPipeline::Params spanParams;
  spanParams.span.sigmaGain = kSigmaGain;
  spanParams.horizon = 2.0;
  spanParams.dt = 0.2;
  spanParams.slices = opt.slices;
  SpanPipeline pipeline(predictor, server, spanParams);
  const bool useSpans = name.rfind("spans", 0) == 0;

  const auto isValid = [&server](size_t a, size_t b) {
    return server.GetEdgeValidity(a, b) != ValidityServer::Validity::INVALID;
  };

  TrackedObstacle track;
  track.id = 1;
  track.velocity = {0.0, scenario.speed, 0.0};
  track.halfExtents = {scenario.obstacleHalf, scenario.obstacleHalf,
                       scenario.obstacleHalf};
  const double std0 = 0.125 * scenario.obstacleHalf;
  track.positionStd = {std0, std0, std0};

  std::vector<size_t> path = planner.Plan(start, goal, isValid);
  size_t lastRebuilds = 0;

  if (opt.trace)
    std::printf("[%s]\n%5s %12s %7s %7s %9s %10s\n", name.c_str(), "t[s]",
                "obstacle y", "gray", "red", "replanned", "update[ms]");

  for (int f = 0; f < opt.frames; ++f) {
    const double t = f / opt.hz;
    track.stamp = t;
    // Start 1.25 s of travel before the aim point, whatever the scale.
    track.pose.translation = {scenario.aimPoint[0],
                              (scenario.aimPoint[1] - 1.25 * scenario.speed) +
                                  scenario.speed * t,
                              scenario.aimPoint[2]};

    const auto t0 = std::chrono::steady_clock::now();
    if (useSpans) {
      pipeline.Update({track});
    } else {
      server.Update({predictor.Predict(track, spanParams.horizon,
                                       spanParams.dt)});
    }
    result.updateMs.push_back(std::chrono::duration<double, std::milli>(
                                  std::chrono::steady_clock::now() - t0)
                                  .count());

    bool replanned = false;
    if (path.empty() || planner.PathBlocked(path, isValid)) {
      path = planner.Plan(start, goal, isValid);
      replanned = true;
      ++result.replans;
    }
    result.pathSizes.push_back(path.size());

    if (render) {
      FrameDump fd;
      fd.t = t;
      fd.obstacle = track.pose.translation;
      fd.replanned = replanned;
      fd.ms = result.updateMs.back();
      fd.path = path;
      const size_t rebuildsNow =
          useSpans ? pipeline.GetStats().rebuilds : size_t(f + 1);
      fd.rebuilt = rebuildsNow > lastRebuilds;
      lastRebuilds = rebuildsNow;

      for (size_t slot = 0; slot < render->edgeIds.size(); ++slot) {
        const auto& e = graph.edges[render->edgeIds[slot]];
        const auto v = server.GetEdgeValidity(e.src, e.tgt);
        if (v == ValidityServer::Validity::INVALID) fd.red.push_back(slot);
        else if (v == ValidityServer::Validity::UNKNOWN) fd.gray.push_back(slot);
      }

      if (useSpans) {
        if (const Span* span = pipeline.GetSpan(track.id)) {
          const auto& traj = span->Trajectory();
          for (size_t i = 0; i < traj.poses.size(); ++i) {
            fd.envCenters.push_back(traj.poses[i].translation);
            fd.envHalf.push_back(span->EnvelopeHalfWidth(traj.stamps[i]));
          }
        }
      }
      result.frames.push_back(std::move(fd));
    }

    if (opt.trace) {
      size_t gray = 0, red = 0;
      for (const auto& e : graph.edges) {
        const auto v = server.GetEdgeValidity(e.src, e.tgt);
        if (v == ValidityServer::Validity::INVALID) ++red;
        else if (v == ValidityServer::Validity::UNKNOWN) ++gray;
      }
      std::printf("%5.1f %12.2f %7zu %7zu %9s %10.2f\n", t,
                  track.pose.translation[1], gray, red,
                  replanned ? "YES" : "-", result.updateMs.back());
    }
  }

  // In sliced mode each span rebuild constructs one geometry per slice.
  result.geometryRebuilds =
      useSpans ? pipeline.GetStats().rebuilds * std::max<size_t>(1, opt.slices)
               : size_t(opt.frames);
  result.expiries = useSpans ? pipeline.GetStats().expiries : 0;
  return result;
}


/// Minimal JSON writer for the HTML viewer (no dependency needed).
void WriteViewerJson(const std::string& path, const std::string& roadmapName,
                     const RoadmapGraph& graph, const RenderSet& render,
                     const Scenario& scenario,
                     const std::vector<ModeResult>& results) {
  std::ofstream o(path);
  if (!o) {
    std::fprintf(stderr, "cannot write %s\n", path.c_str());
    return;
  }
  o.precision(4);
  o << std::fixed;

  const auto vec3 = [&o](const Vec3& v) {
    o << "[" << v[0] << "," << v[1] << "," << v[2] << "]";
  };
  const auto idxList = [&o](const std::vector<size_t>& v) {
    o << "[";
    for (size_t i = 0; i < v.size(); ++i) o << (i ? "," : "") << v[i];
    o << "]";
  };

  o << "{\n\"roadmap\":{\"name\":\"" << roadmapName << "\",\"totalEdges\":"
    << graph.edges.size() << ",\"totalVertices\":" << graph.vertices.size()
    << ",\"dof\":" << graph.dof << ",\n\"vertices\":[";
  for (size_t v = 0; v < graph.vertices.size(); ++v) {
    if (v) o << ",";
    vec3(graph.vertices[v].position);
  }
  o << "],\n\"edges\":[";
  for (size_t i = 0; i < render.edgeIds.size(); ++i) {
    const auto& e = graph.edges[render.edgeIds[i]];
    if (i) o << ",";
    o << "[" << e.src << "," << e.tgt << "]";
  }
  o << "]},\n\"obstacleHalf\":" << scenario.obstacleHalf << ",\n\"modes\":[";

  for (size_t m = 0; m < results.size(); ++m) {
    const auto& r = results[m];
    if (m) o << ",";
    o << "\n{\"name\":\"" << r.name << "\",\"geomPasses\":"
      << r.geometryRebuilds << ",\"expiries\":" << r.expiries
      << ",\"replans\":" << r.replans << ",\"meanMs\":" << r.Mean()
      << ",\"medianMs\":" << r.Median() << ",\"maxMs\":" << r.Max()
      << ",\n\"frames\":[";
    for (size_t f = 0; f < r.frames.size(); ++f) {
      const auto& fd = r.frames[f];
      if (f) o << ",";
      o << "\n{\"t\":" << fd.t << ",\"obstacle\":";
      vec3(fd.obstacle);
      o << ",\"ms\":" << fd.ms << ",\"replanned\":"
        << (fd.replanned ? "true" : "false") << ",\"rebuilt\":"
        << (fd.rebuilt ? "true" : "false") << ",\"red\":";
      idxList(fd.red);
      o << ",\"gray\":";
      idxList(fd.gray);
      o << ",\"path\":";
      idxList(fd.path);
      if (!fd.envCenters.empty()) {
        o << ",\"env\":[";
        for (size_t i = 0; i < fd.envCenters.size(); ++i) {
          if (i) o << ",";
          o << "{\"c\":";
          vec3(fd.envCenters[i]);
          o << ",\"h\":";
          vec3(fd.envHalf[i]);
          o << "}";
        }
        o << "]";
      }
      o << "}";
    }
    o << "]}";
  }
  o << "\n]}\n";
  std::printf("wrote viewer data: %s\n", path.c_str());
}

}  // namespace

int main(int argc, char** argv) {
  Options opt;
  for (int i = 1; i < argc; ++i) {
    if (!std::strcmp(argv[i], "--roadmap") && ++i < argc) opt.dir = argv[i];
    else if (!std::strcmp(argv[i], "--frames") && ++i < argc) opt.frames = std::atoi(argv[i]);
    else if (!std::strcmp(argv[i], "--hz") && ++i < argc) opt.hz = std::atof(argv[i]);
    else if (!std::strcmp(argv[i], "--mode") && ++i < argc) opt.mode = argv[i];
    else if (!std::strcmp(argv[i], "--slices") && ++i < argc) opt.slices = std::atoi(argv[i]);
    else if (!std::strcmp(argv[i], "--dump-json") && ++i < argc) opt.dumpJson = argv[i];
    else if (!std::strcmp(argv[i], "--render-cap") && ++i < argc) opt.renderCap = std::atoi(argv[i]);
    else if (!std::strcmp(argv[i], "--trace")) opt.trace = true;
  }
  if (opt.dir.empty()) {
    std::fprintf(stderr,
                 "usage: demo_dynamic --roadmap DIR [--frames N] [--hz F] "
                 "[--mode baseline|spans|both] [--trace]\n");
    return 1;
  }

  RoadmapGraph graph;
  if (!ReadRoadmapGraph(graph, opt.dir + "/roadmap_graph.txt")) {
    std::fprintf(stderr, "cannot read %s/roadmap_graph.txt\n", opt.dir.c_str());
    return 1;
  }
  std::printf("roadmap: %zu vertices, %zu edges\n", graph.vertices.size(),
              graph.edges.size());

  // Start/goal near opposite workspace corners; aim point = midpoint of
  // the obstacle-free initial path (computed once so both modes share it).
  const auto nearest = [&](double x, double y, double z) {
    size_t best = 0;
    double bestD = 1e30;
    for (size_t v = 0; v < graph.vertices.size(); ++v) {
      const auto& p = graph.vertices[v].position;
      const double d = std::pow(p[0] - x, 2) + std::pow(p[1] - y, 2) +
                       std::pow(p[2] - z, 2);
      if (d < bestD) { bestD = d; best = v; }
    }
    return best;
  };
  const size_t start = nearest(0.5, 0.5, 0.5);
  const size_t goal = nearest(9.5, 9.5, 9.5);

  Vec3 aimPoint;
  {
    Replanner planner(graph.vertices.size(), graph.edges);
    const auto path =
        planner.Plan(start, goal, [](size_t, size_t) { return true; });
    if (path.empty()) {
      std::fprintf(stderr, "no initial path -- roadmap too sparse?\n");
      return 1;
    }
    aimPoint = graph.vertices[path[path.size() / 2]].position;
  }
  const Scenario scenario = Scenario::From(graph, aimPoint);
  std::printf("query: %zu -> %zu, obstacle half=%.2f speed=%.2f aimed at "
              "(%.2f, %.2f, %.2f)\n\n",
              start, goal, scenario.obstacleHalf, scenario.speed, aimPoint[0],
              aimPoint[1], aimPoint[2]);

  std::unique_ptr<RenderSet> render;
  if (!opt.dumpJson.empty()) {
    // Reach: obstacle travel over the run, plus a margin, so every edge
    // it can plausibly touch is drawn.
    const double reach = 2.0 * scenario.speed * (opt.frames / opt.hz) +
                         6.0 * scenario.obstacleHalf;
    render = std::make_unique<RenderSet>(
        RenderSet::Build(graph, aimPoint, reach, opt.renderCap));
    std::printf("viewer: rendering %zu of %zu edges\n\n",
                render->edgeIds.size(), graph.edges.size());
  }

  std::vector<ModeResult> results;
  if (opt.mode == "baseline" || opt.mode == "both")
    results.push_back(RunMode("baseline", opt, graph, start, goal, scenario,
                              render.get()));
  if (opt.mode == "spans" || opt.mode == "both") {
    std::string name = "spans";
    if (opt.slices > 1) name += "(k=" + std::to_string(opt.slices) + ")";
    results.push_back(RunMode(name, opt, graph, start, goal, scenario,
                              render.get()));
  }

  std::printf("%-11s %7s %11s %9s %8s %26s\n", "mode", "frames", "geom-passes",
              "expiries", "replans", "update ms mean/med/max");
  for (const auto& r : results)
    std::printf("%-11s %7d %11zu %9zu %8d %12.3f /%7.3f /%7.3f\n",
                r.name.c_str(), opt.frames, r.geometryRebuilds, r.expiries,
                r.replans, r.Mean(), r.Median(), r.Max());

  if (results.size() == 2) {
    const auto& base = results[0];
    const auto& spans = results[1];
    if (spans.Mean() > 0)
      std::printf("\nspan speedup: %.1fx mean update (conforming frames are "
                  "~free: median %.4f ms), %zu -> %zu expensive geometry "
                  "passes\n",
                  base.Mean() / spans.Mean(), spans.Median(),
                  base.geometryRebuilds, spans.geometryRebuilds);
    if (base.pathSizes == spans.pathSizes && base.replans == spans.replans)
      std::printf("identical replan behavior in both modes "
                  "(same path sizes every frame)\n");
    else
      std::printf("NOTE: modes diverged in replan behavior "
                  "(%d vs %d replans) -- expected only from span "
                  "conservatism at rebuild boundaries\n",
                  base.replans, spans.replans);
  }
  if (render && !opt.dumpJson.empty()) {
    // Roadmap name from the directory's last component.
    std::string name = opt.dir;
    const size_t slash = name.find_last_of('/');
    if (slash != std::string::npos) name = name.substr(slash + 1);
    WriteViewerJson(opt.dumpJson, name, graph, *render, scenario, results);
  }
  return 0;
}
