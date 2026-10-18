// Validity node: predicted trajectories -> roadmap validity + path.
//
// Loads the offline-built roadmap (graph + geometries), updates edge
// validity from incoming predictions, and republishes the current
// shortest valid path between the configured start/goal vertices,
// replanning whenever the active path becomes blocked.
//
// Subscribes:  /spite_d_prediction/predictions (PredictedTrajectoryArray)
// Publishes:   ~/path    (nav_msgs/Path)              -- vertex positions
//              ~/markers (visualization_msgs/MarkerArray) -- for RViz:
//                roadmap edges coloured by validity, plus the active path
//
// Parameters:  roadmap_graph, roadmap_geoms (file paths, required)
//              start_vid, goal_vid, sigma_gain
//              marker_frame   frame the markers are published in
//              max_drawn_edges  RViz cannot chew 100k edges; edges beyond
//                this cap are skipped, blocked ones kept first
// Compiled only under ament (Linux).

#include "planner/replanner.hpp"
#include "planner/roadmap_io.hpp"
#include "spite/validity_server.hpp"

#include "DynamicRoadmapTool.h"

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <spite_d/msg/predicted_trajectory_array.hpp>
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

    m_sub = create_subscription<spite_d::msg::PredictedTrajectoryArray>(
        "/spite_d_prediction/predictions", 10,
        [this](spite_d::msg::PredictedTrajectoryArray::ConstSharedPtr msg) {
          OnPredictions(*msg);
        });
    m_pathPub = create_publisher<nav_msgs::msg::Path>("~/path", 10);

    m_markerFrame = declare_parameter<std::string>("marker_frame", "map");
    m_maxDrawnEdges =
        size_t(declare_parameter<int>("max_drawn_edges", 4000));
    // Drawing costs a validity lookup per edge, so it is throttled well
    // below the prediction rate -- the display only has to be readable.
    m_markerPeriod = declare_parameter("marker_period_s", 0.2);
    m_markerPub = create_publisher<visualization_msgs::msg::MarkerArray>(
        "~/markers", 1);

    RCLCPP_INFO(get_logger(),
                "roadmap loaded: %zu vertices, %zu edges; planning %zu -> %zu",
                m_graph.vertices.size(), m_graph.edges.size(), m_startVid,
                m_goalVid);
  }

 private:
  void OnPredictions(const spite_d::msg::PredictedTrajectoryArray& msg) {
    std::vector<spite_d::PredictedTrajectory> predictions;
    for (const auto& t : msg.trajectories) {
      spite_d::PredictedTrajectory traj;
      traj.id = t.id;
      traj.halfExtents = {t.half_extents.x, t.half_extents.y,
                          t.half_extents.z};
      for (size_t i = 0; i < t.poses.size(); ++i) {
        traj.stamps.push_back(rclcpp::Time(t.stamps[i]).seconds());
        spite_d::Pose3 pose;
        pose.translation = {t.poses[i].position.x, t.poses[i].position.y,
                            t.poses[i].position.z};
        traj.poses.push_back(pose);
        if (i < t.position_std.size())
          traj.positionStd.push_back({t.position_std[i].x,
                                      t.position_std[i].y,
                                      t.position_std[i].z});
      }
      predictions.push_back(std::move(traj));
    }
    // Obstacles absent from this snapshot have left the scene: release
    // their slots and re-validate what they were blocking. Without this
    // a departed obstacle blocks its last-known edges forever.
    std::vector<int32_t> activeIds;
    activeIds.reserve(predictions.size());
    for (const auto& t : predictions) activeIds.push_back(t.id);
    m_server->RetainOnly(activeIds);

    m_server->Update(predictions);

    // Gray (UNKNOWN) edges are conservatively treated as blocked until
    // a fine-check hook lands.
    const auto isValid = [this](size_t a, size_t b) {
      return m_server->GetEdgeValidity(a, b) ==
             spite_d::ValidityServer::Validity::VALID;
    };

    if (m_currentPath.empty() ||
        m_replanner->PathBlocked(m_currentPath, isValid)) {
      m_currentPath = m_replanner->Plan(m_startVid, m_goalVid, isValid);
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

  std::string m_markerFrame;
  size_t m_maxDrawnEdges{4000};
  double m_markerPeriod{0.2};
  rclcpp::Time m_lastMarkers{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<spite_d::msg::PredictedTrajectoryArray>::SharedPtr m_sub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_pathPub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_markerPub;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ValidityNode>());
  rclcpp::shutdown();
  return 0;
}
