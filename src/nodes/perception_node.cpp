// Perception node: depth images -> tracked obstacles.
//
// Subscribes:  /camera/depth_image (sensor_msgs/Image, 32FC1 meters from
//              gz or 16UC1 millimeters from a RealSense)
//              /camera/camera_info (sensor_msgs/CameraInfo)
// Publishes:   ~/obstacles (spite_d/ObstacleArray)
//
// The camera is assumed fixed; its pose comes from parameters
// (camera.{x,y,z,yaw}), interpreted as the body frame (x forward,
// z up) of the SDF model. The optical-frame rotation handed to the
// tracker is derived from it. Compiled only under ament (Linux).

#include "perception/box_tracker.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <spite_d/msg/obstacle_array.hpp>
#include <spite_d/msg/obstacle_state.hpp>

#include <chrono>
#include <cmath>
#include <set>
#include <string>
#include <vector>

namespace {

/// World-from-optical rotation for a body-frame yaw (x-forward body,
/// z-up world; optical frame is +x right, +y down, +z forward).
spite_d::Mat3 OpticalRotationFromYaw(double yaw) {
  const double c = std::cos(yaw), s = std::sin(yaw);
  // Columns: x_opt(right) = (s,-c,0), y_opt(down) = (0,0,-1),
  //          z_opt(forward) = (c,s,0).
  return {{{s, 0, c}, {-c, 0, s}, {0, -1, 0}}};
}

}  // namespace

class PerceptionNode : public rclcpp::Node {
 public:
  PerceptionNode() : Node("spite_d_perception") {
    spite_d::BoxTracker::Params params;
    params.detector.depthScale =
        declare_parameter("depth_scale", 1000.0);
    m_tracker = std::make_unique<spite_d::BoxTracker>(params);

    m_reportPeriod = declare_parameter("report_period_s", 5.0);
    m_cameraPose.translation = {declare_parameter("camera.x", 0.2),
                                declare_parameter("camera.y", 5.0),
                                declare_parameter("camera.z", 1.0)};
    m_cameraPose.rotation =
        OpticalRotationFromYaw(declare_parameter("camera.yaw", 0.0));

    m_infoSub = create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera/camera_info", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
          m_fx = msg->k[0];
          m_fy = msg->k[4];
          m_cx = msg->k[2];
          m_cy = msg->k[5];
          m_haveInfo = true;
        });

    m_depthSub = create_subscription<sensor_msgs::msg::Image>(
        "/camera/depth_image", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
          OnDepth(*msg);
        });

    m_pub = create_publisher<spite_d::msg::ObstacleArray>("~/obstacles", 10);
    m_markerPub = create_publisher<visualization_msgs::msg::MarkerArray>(
        "~/markers", 1);
  }

 private:
  void OnDepth(const sensor_msgs::msg::Image& msg) {
    if (!m_haveInfo) return;

    // Normalize to 16UC1 millimeters.
    const size_t n = size_t(msg.width) * msg.height;
    m_depthMm.resize(n);
    if (msg.encoding == "32FC1") {
      const float* d = reinterpret_cast<const float*>(msg.data.data());
      for (size_t i = 0; i < n; ++i)
        m_depthMm[i] =
            std::isfinite(d[i]) ? uint16_t(d[i] * 1000.0f) : uint16_t(0);
    } else if (msg.encoding == "16UC1") {
      const uint16_t* d = reinterpret_cast<const uint16_t*>(msg.data.data());
      m_depthMm.assign(d, d + n);
    } else {
      RCLCPP_WARN_ONCE(get_logger(), "unsupported depth encoding: %s",
                       msg.encoding.c_str());
      return;
    }

    spite_d::DepthFrame frame;
    frame.width = int(msg.width);
    frame.height = int(msg.height);
    frame.fx = m_fx;
    frame.fy = m_fy;
    frame.cx = m_cx;
    frame.cy = m_cy;
    frame.stamp = rclcpp::Time(msg.header.stamp).seconds();
    frame.cameraPose = m_cameraPose;
    frame.depth = m_depthMm.data();

    const auto t0 = std::chrono::steady_clock::now();
    const auto tracks = m_tracker->Update(frame);
    m_detectMsSum += std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t0).count();
    ++m_frames;
    m_trackCount = tracks.size();
    for (const auto& t : tracks) m_idsSeen.insert(t.id);
    ReportIfDue();

    spite_d::msg::ObstacleArray out;
    out.header = msg.header;
    out.header.frame_id = "world";
    for (const auto& t : tracks) {
      spite_d::msg::ObstacleState o;
      o.header = out.header;
      o.id = t.id;
      o.pose.position.x = t.pose.translation[0];
      o.pose.position.y = t.pose.translation[1];
      o.pose.position.z = t.pose.translation[2];
      o.pose.orientation.w = 1.0;
      o.half_extents.x = t.halfExtents[0];
      o.half_extents.y = t.halfExtents[1];
      o.half_extents.z = t.halfExtents[2];
      o.velocity.x = t.velocity[0];
      o.velocity.y = t.velocity[1];
      o.velocity.z = t.velocity[2];
      o.position_std.x = t.positionStd[0];
      o.position_std.y = t.positionStd[1];
      o.position_std.z = t.positionStd[2];
      out.obstacles.push_back(o);
    }
    m_pub->publish(out);
    PublishMarkers(out);
  }

  /// Periodic one-line summary. These are the preliminary numbers worth
  /// copying out of the terminal: whether perception keeps up with the
  /// camera, how long detection takes, and how badly track ids churn --
  /// distinct ids far above the number of people present means the
  /// tracker is losing and re-acquiring them.
  void ReportIfDue() {
    const auto now = this->now();
    if (m_lastReport.nanoseconds() == 0) { m_lastReport = now; return; }
    const double elapsed = (now - m_lastReport).seconds();
    if (elapsed < m_reportPeriod) return;

    RCLCPP_INFO(get_logger(),
                "perception | %.1f Hz | detect %.1f ms | tracks now %zu | "
                "distinct ids in %.0fs: %zu",
                m_frames / elapsed, m_frames ? m_detectMsSum / m_frames : 0.0,
                m_trackCount, elapsed, m_idsSeen.size());

    m_lastReport = now;
    m_frames = 0;
    m_detectMsSum = 0.0;
    m_idsSeen.clear();
  }

  /// One translucent box plus an id label per tracked obstacle, so the
  /// first question on real hardware -- "does the detector see anything,
  /// and is it in the right place?" -- can be answered by looking.
  void PublishMarkers(const spite_d::msg::ObstacleArray& obstacles) {
    visualization_msgs::msg::MarkerArray arr;

    visualization_msgs::msg::Marker clear;
    clear.header = obstacles.header;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(clear);

    int32_t id = 0;
    for (const auto& o : obstacles.obstacles) {
      visualization_msgs::msg::Marker box;
      box.header = obstacles.header;
      box.ns = "obstacles";
      box.id = id++;
      box.type = visualization_msgs::msg::Marker::CUBE;
      box.action = visualization_msgs::msg::Marker::ADD;
      box.pose = o.pose;
      box.scale.x = 2.0 * o.half_extents.x;
      box.scale.y = 2.0 * o.half_extents.y;
      box.scale.z = 2.0 * o.half_extents.z;
      box.color.r = 0.9f;
      box.color.b = 0.2f;
      box.color.a = 0.5f;
      arr.markers.push_back(box);

      visualization_msgs::msg::Marker label;
      label.header = obstacles.header;
      label.ns = "obstacle_ids";
      label.id = id;
      label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      label.action = visualization_msgs::msg::Marker::ADD;
      label.pose = o.pose;
      label.pose.position.z += o.half_extents.z + 0.15;
      label.scale.z = 0.18;
      label.color.r = label.color.g = label.color.b = 1.0f;
      label.color.a = 0.9f;
      // Watching the id change tells you the tracker is dropping and
      // re-acquiring the same person, which is worth seeing directly.
      label.text = "id " + std::to_string(o.id);
      arr.markers.push_back(label);
    }
    m_markerPub->publish(arr);
  }

  std::unique_ptr<spite_d::BoxTracker> m_tracker;
  spite_d::Pose3 m_cameraPose;
  std::vector<uint16_t> m_depthMm;
  double m_fx{0}, m_fy{0}, m_cx{0}, m_cy{0};
  bool m_haveInfo{false};

  double m_reportPeriod{5.0};
  rclcpp::Time m_lastReport{0, 0, RCL_ROS_TIME};
  size_t m_frames{0}, m_trackCount{0};
  double m_detectMsSum{0.0};
  std::set<int32_t> m_idsSeen;

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr m_infoSub;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_depthSub;
  rclcpp::Publisher<spite_d::msg::ObstacleArray>::SharedPtr m_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_markerPub;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PerceptionNode>());
  rclcpp::shutdown();
  return 0;
}
