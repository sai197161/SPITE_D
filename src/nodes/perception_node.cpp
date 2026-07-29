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
#include <spite_d/msg/obstacle_array.hpp>
#include <spite_d/msg/obstacle_state.hpp>

#include <cmath>
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

    const auto tracks = m_tracker->Update(frame);

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
  }

  std::unique_ptr<spite_d::BoxTracker> m_tracker;
  spite_d::Pose3 m_cameraPose;
  std::vector<uint16_t> m_depthMm;
  double m_fx{0}, m_fy{0}, m_cx{0}, m_cy{0};
  bool m_haveInfo{false};

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr m_infoSub;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_depthSub;
  rclcpp::Publisher<spite_d::msg::ObstacleArray>::SharedPtr m_pub;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PerceptionNode>());
  rclcpp::shutdown();
  return 0;
}
