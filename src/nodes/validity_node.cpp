// Validity node: tracked obstacles -> roadmap validity + path.
//
// Loads the offline-built roadmap (graph + geometries), updates edge
// validity from incoming obstacles, and republishes the current
// shortest valid path between the configured start/goal vertices,
// replanning whenever the active path becomes blocked.
//
// Subscribes:  /spite_d_perception/obstacles (ObstacleArray)
//
// Takes tracked obstacles rather than ready-made predictions because
// SpanPipeline has to own prediction: a span is a frozen prediction, and
// rebuilding one means predicting again from the current observation.
// Both modes therefore predict here, so `use_spans` changes only when
// certification geometry is rebuilt -- an A/B on identical input.
// Publishes:   ~/path    (nav_msgs/Path)              -- vertex positions
//              ~/markers (visualization_msgs/MarkerArray) -- for RViz:
//                roadmap edges coloured by validity, plus the active path
//
// Parameters:  roadmap_graph, roadmap_geoms (file paths, required)
//              start_vid, goal_vid, sigma_gain
//              use_spans      true: amortise certification over spans;
//                false: recertify every obstacle every frame (baseline).
//                The A/B for the contribution -- same input either way.
//              horizon, dt, slices, conformance_slack, refresh_fraction,
//              std_growth_rate  prediction + span parameters
//              report_period_s  seconds between summary lines
//              marker_frame   frame the markers are published in
//              max_drawn_edges  RViz cannot chew 100k edges; edges beyond
//                this cap are skipped, blocked ones kept first
// Compiled only under ament (Linux).

#include "planner/replanner.hpp"
#include "planner/roadmap_io.hpp"
#include "spite/validity_server.hpp"
#include "trajectory/predictor.hpp"
#include "trajectory/span_pipeline.hpp"

#include "DynamicRoadmapTool.h"

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <spite_d/msg/obstacle_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <string>
#include <vector>

class ValidityNode : public rclcpp::Node {
 public:
  ValidityNode() : Node("spite_d_validity") {
    const auto graphPath = declare_parameter<std::string>("roadmap_graph");
    const auto geomPath = declare_parameter<std::string>("roadmap_geoms");
    m_startVid = size_t(declare_parameter<int>("start_vid", 0));
    m_goalVid = size_t(declare_parameter<int>("goal_vid", 1));

    if (!spite_d::ReadRoadmapGraph(m_graph, graphPath))
      throw std::runtime_error("cannot read roadmap graph: " + graphPath);

    auto drm = std::make_unique<DynamicRoadmapTool>();
    drm->SetUseUnderApprox(true);
    if (!drm->ReadGeometriesFromFile(geomPath))
      throw std::runtime_error("cannot read roadmap geoms: " + geomPath);

    spite_d::ValidityServer::Params params;
    params.sigmaGain = declare_parameter("sigma_gain", 1.0);
    m_server = std::make_unique<spite_d::ValidityServer>(std::move(drm),
                                                         params);
    m_replanner = std::make_unique<spite_d::Replanner>(
        m_graph.vertices.size(), m_graph.edges);

    spite_d::SpanPipeline::Params spanParams;
    spanParams.horizon = declare_parameter("horizon", 3.0);
    spanParams.dt = declare_parameter("dt", 0.25);
    spanParams.slices = size_t(declare_parameter<int>("slices", 1));
    spanParams.span.sigmaGain = params.sigmaGain;
    spanParams.span.slack = declare_parameter("conformance_slack", 0.05);
    spanParams.span.refreshFraction = declare_parameter("refresh_fraction", 0.7);
    m_useSpans = declare_parameter("use_spans", true);

    m_predictor = std::make_unique<spite_d::ConstantVelocityPredictor>(
        declare_parameter("std_growth_rate", 0.1));
    m_pipeline = std::make_unique<spite_d::SpanPipeline>(
        *m_predictor, *m_server, spanParams);
    m_horizon = spanParams.horizon;
    m_dt = spanParams.dt;

    RCLCPP_INFO(get_logger(), "certification mode: %s",
                m_useSpans ? "SPANS (amortised)" : "BASELINE (every frame)");

    m_sub = create_subscription<spite_d::msg::ObstacleArray>(
        "/spite_d_perception/obstacles", 10,
        [this](spite_d::msg::ObstacleArray::ConstSharedPtr msg) {
          OnObstacles(*msg);
        });
    m_pathPub = create_publisher<nav_msgs::msg::Path>("~/path", 10);

    m_markerFrame = declare_parameter<std::string>("marker_frame", "map");
    m_maxDrawnEdges =
        size_t(declare_parameter<int>("max_drawn_edges", 4000));
    // Drawing costs a validity lookup per edge, so it is throttled well
    // below the prediction rate -- the display only has to be readable.
    m_markerPeriod = declare_parameter("marker_period_s", 0.2);
    m_reportPeriod = declare_parameter("report_period_s", 5.0);
    m_markerPub = create_publisher<visualization_msgs::msg::MarkerArray>(
        "~/markers", 1);

    RCLCPP_INFO(get_logger(),
                "roadmap loaded: %zu vertices, %zu edges; planning %zu -> %zu",
                m_graph.vertices.size(), m_graph.edges.size(), m_startVid,
                m_goalVid);
  }

 private:
  void OnObstacles(const spite_d::msg::ObstacleArray& msg) {
    std::vector<spite_d::TrackedObstacle> tracks;
    tracks.reserve(msg.obstacles.size());
    for (const auto& o : msg.obstacles) {
      spite_d::TrackedObstacle track;
      track.id = o.id;
      track.stamp = rclcpp::Time(o.header.stamp).seconds();
      track.pose.translation = {o.pose.position.x, o.pose.position.y,
                                o.pose.position.z};
      track.halfExtents = {o.half_extents.x, o.half_extents.y,
                           o.half_extents.z};
      track.velocity = {o.velocity.x, o.velocity.y, o.velocity.z};
      track.positionStd = {o.position_std.x, o.position_std.y,
                           o.position_std.z};
      tracks.push_back(track);
    }

    // Obstacles absent from this snapshot have left the scene: release
    // their slots and re-validate what they were blocking. Without this
    // a departed obstacle blocks its last-known edges forever.
    std::vector<int32_t> activeIds;
    activeIds.reserve(tracks.size());
    for (const auto& t : tracks) activeIds.push_back(t.id);

    if (m_useSpans) {
      // Spans decide internally whether each track needs a rebuild; a
      // conforming track never reaches the server, so the tree query is
      // skipped entirely.
      m_pipeline->RetainOnly(activeIds);
      m_pipeline->Update(tracks);
    } else {
      // Baseline: predict and recertify every track, every frame.
      std::vector<spite_d::PredictedTrajectory> predictions;
      predictions.reserve(tracks.size());
      for (const auto& t : tracks)
        predictions.push_back(m_predictor->Predict(t, m_horizon, m_dt));
      m_server->RetainOnly(activeIds);
      m_server->Update(predictions);
    }

    // Gray (UNKNOWN) edges are conservatively treated as blocked until
    // a fine-check hook lands.
    const auto isValid = [this](size_t a, size_t b) {
      return m_server->GetEdgeValidity(a, b) ==
             spite_d::ValidityServer::Validity::VALID;
    };

    if (m_currentPath.empty() ||
        m_replanner->PathBlocked(m_currentPath, isValid)) {
      m_currentPath = m_replanner->Plan(m_startVid, m_goalVid, isValid);
      ++m_replans;
      if (m_currentPath.empty())
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "no valid path from %zu to %zu", m_startVid,
                             m_goalVid);
    }

    nav_msgs::msg::Path path;
    path.header = msg.header;
    path.header.frame_id = "world";
    for (const auto vid : m_currentPath) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = m_graph.vertices[vid].position[0];
      pose.pose.position.y = m_graph.vertices[vid].position[1];
      pose.pose.position.z = m_graph.vertices[vid].position[2];
      pose.pose.orientation.x = m_graph.vertices[vid].quaternion[0];
      pose.pose.orientation.y = m_graph.vertices[vid].quaternion[1];
      pose.pose.orientation.z = m_graph.vertices[vid].quaternion[2];
      pose.pose.orientation.w = m_graph.vertices[vid].quaternion[3];
      path.poses.push_back(pose);
    }
    m_pathPub->publish(path);

    const auto now = this->now();
    if ((now - m_lastMarkers).seconds() >= m_markerPeriod) {
      m_lastMarkers = now;
      PublishMarkers(path.header.stamp);
    }
    ++m_updates;
    ReportIfDue();
  }

  /// Periodic one-line summary -- the preliminary numbers for this half
  /// of the pipeline. The update rate is the one to compare against the
  /// camera rate: certification is the expensive stage, and if it runs
  /// well below the sensor it is the bottleneck. The obb/tree split says
  /// which part of certification the time is going to.
  ///
  /// In span mode the timings cover only the frames that actually
  /// rebuilt, so "certify ms" is per rebuild rather than per frame, and
  /// the skip percentage is what the amortisation bought.
  void ReportIfDue() {
    const auto now = this->now();
    if (m_lastReport.nanoseconds() == 0) { m_lastReport = now; return; }
    const double elapsed = (now - m_lastReport).seconds();
    if (elapsed < m_reportPeriod) return;

    using V = spite_d::ValidityServer::Validity;
    size_t red = 0, gray = 0;
    for (const auto& e : m_graph.edges) {
      const auto v = m_server->GetEdgeValidity(e.src, e.tgt);
      if (v == V::INVALID) ++red;
      else if (v == V::UNKNOWN) ++gray;
    }

    const auto& t = m_server->GetTimings();
    const double n = t.passes ? double(t.passes) : 1.0;

    std::string mode = "baseline";
    if (m_useSpans) {
      // Stats are cumulative; report the window.
      const auto& s = m_pipeline->GetStats();
      const size_t obs = s.observations - m_lastStats.observations;
      const size_t rebuilt = s.rebuilds - m_lastStats.rebuilds;
      const size_t expired = s.expiries - m_lastStats.expiries;
      const double skipped =
          obs ? 100.0 * double(obs - rebuilt) / double(obs) : 0.0;
      mode = "spans: " + std::to_string(rebuilt) + " rebuilds / " +
             std::to_string(obs) + " obs (" +
             std::to_string(int(skipped + 0.5)) + "% skipped), " +
             std::to_string(expired) + " expiries";
      m_lastStats = s;
    }

    RCLCPP_INFO(get_logger(),
                "validity | %.1f Hz | %s | certify %.1f ms (obb %.2f / "
                "tree %.1f) | edges red %zu gray %zu of %zu | replans %zu",
                m_updates / elapsed, mode.c_str(), t.TotalMs() / n,
                t.obbFitMs / n, t.treeMs / n, red, gray,
                m_graph.edges.size(), m_replans);

    m_lastReport = now;
    m_updates = 0;
    m_server->ResetTimings();
  }

  /// Roadmap edges coloured by certification, plus the active path.
  /// Green = certified safe, grey = undecided, red = certified blocked.
  void PublishMarkers(const builtin_interfaces::msg::Time& stamp) {
    using V = spite_d::ValidityServer::Validity;
    visualization_msgs::msg::MarkerArray arr;

    auto base = [&](const std::string& ns, int32_t id, double width) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = m_markerFrame;
      m.header.stamp = stamp;
      m.ns = ns;
      m.id = id;
      m.type = visualization_msgs::msg::Marker::LINE_LIST;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.scale.x = width;
      m.pose.orientation.w = 1.0;
      return m;
    };
    auto vertexPoint = [&](size_t vid) {
      geometry_msgs::msg::Point p;
      p.x = m_graph.vertices[vid].position[0];
      p.y = m_graph.vertices[vid].position[1];
      p.z = m_graph.vertices[vid].position[2];
      return p;
    };

    auto green = base("roadmap_valid", 0, 0.004);
    green.color.g = 0.8f; green.color.a = 0.25f;
    auto grey = base("roadmap_unknown", 1, 0.008);
    grey.color.r = grey.color.g = grey.color.b = 0.6f; grey.color.a = 0.6f;
    auto red = base("roadmap_blocked", 2, 0.016);
    red.color.r = 0.9f; red.color.a = 1.0f;

    // Blocked and undecided edges are the interesting ones, so they are
    // never the edges dropped by the cap.
    size_t drawn = 0;
    std::vector<size_t> deferred;
    for (size_t e = 0; e < m_graph.edges.size(); ++e) {
      const auto& edge = m_graph.edges[e];
      const auto v = m_server->GetEdgeValidity(edge.src, edge.tgt);
      if (v == V::VALID) { deferred.push_back(e); continue; }
      auto& target = (v == V::INVALID) ? red : grey;
      target.points.push_back(vertexPoint(edge.src));
      target.points.push_back(vertexPoint(edge.tgt));
      ++drawn;
    }
    for (size_t e : deferred) {
      if (drawn >= m_maxDrawnEdges) break;
      const auto& edge = m_graph.edges[e];
      green.points.push_back(vertexPoint(edge.src));
      green.points.push_back(vertexPoint(edge.tgt));
      ++drawn;
    }
    arr.markers.push_back(green);
    arr.markers.push_back(grey);
    arr.markers.push_back(red);

    auto pathMarker = base("path", 3, 0.03);
    pathMarker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    pathMarker.color.r = 1.0f; pathMarker.color.g = 0.8f;
    pathMarker.color.a = 1.0f;
    for (size_t vid : m_currentPath) pathMarker.points.push_back(vertexPoint(vid));
    arr.markers.push_back(pathMarker);

    m_markerPub->publish(arr);
  }

  spite_d::RoadmapGraph m_graph;
  std::unique_ptr<spite_d::ValidityServer> m_server;
  std::unique_ptr<spite_d::Replanner> m_replanner;
  std::vector<size_t> m_currentPath;
  size_t m_startVid{0}, m_goalVid{1};

  std::unique_ptr<spite_d::Predictor> m_predictor;
  std::unique_ptr<spite_d::SpanPipeline> m_pipeline;
  spite_d::SpanPipeline::Stats m_lastStats;
  bool m_useSpans{true};
  double m_horizon{3.0}, m_dt{0.25};

  std::string m_markerFrame;
  size_t m_maxDrawnEdges{4000};
  double m_markerPeriod{0.2};
  rclcpp::Time m_lastMarkers{0, 0, RCL_ROS_TIME};

  double m_reportPeriod{5.0};
  rclcpp::Time m_lastReport{0, 0, RCL_ROS_TIME};
  size_t m_updates{0}, m_replans{0};

  rclcpp::Subscription<spite_d::msg::ObstacleArray>::SharedPtr m_sub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_pathPub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_markerPub;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ValidityNode>());
  rclcpp::shutdown();
  return 0;
}
