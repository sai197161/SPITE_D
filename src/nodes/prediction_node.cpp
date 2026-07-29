// Prediction node: tracked obstacles -> predicted trajectories.
//
// Subscribes:  /spite_d_perception/obstacles (spite_d/ObstacleArray)
// Publishes:   ~/predictions (spite_d/PredictedTrajectoryArray)
//
// Wraps the Predictor interface; constant-velocity for now. The
// horizon/dt/std growth parameters mirror ConstantVelocityPredictor.
// Compiled only under ament (Linux).

#include "trajectory/predictor.hpp"

#include <rclcpp/rclcpp.hpp>
#include <spite_d/msg/obstacle_array.hpp>
#include <spite_d/msg/predicted_trajectory_array.hpp>

class PredictionNode : public rclcpp::Node {
 public:
  PredictionNode() : Node("spite_d_prediction") {
    m_horizon = declare_parameter("horizon", 3.0);
    m_dt = declare_parameter("dt", 0.25);
    m_predictor = std::make_unique<spite_d::ConstantVelocityPredictor>(
        declare_parameter("std_growth_rate", 0.1));

    m_sub = create_subscription<spite_d::msg::ObstacleArray>(
        "/spite_d_perception/obstacles", 10,
        [this](spite_d::msg::ObstacleArray::ConstSharedPtr msg) {
          OnObstacles(*msg);
        });
    m_pub = create_publisher<spite_d::msg::PredictedTrajectoryArray>(
        "~/predictions", 10);
  }

 private:
  void OnObstacles(const spite_d::msg::ObstacleArray& msg) {
    spite_d::msg::PredictedTrajectoryArray out;
    out.header = msg.header;

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

      const auto traj = m_predictor->Predict(track, m_horizon, m_dt);

      spite_d::msg::PredictedTrajectory t;
      t.header = msg.header;
      t.id = traj.id;
      t.half_extents.x = traj.halfExtents[0];
      t.half_extents.y = traj.halfExtents[1];
      t.half_extents.z = traj.halfExtents[2];
      for (size_t i = 0; i < traj.poses.size(); ++i) {
        t.stamps.push_back(rclcpp::Time(int64_t(traj.stamps[i] * 1e9)));
        geometry_msgs::msg::Pose pose;
        pose.position.x = traj.poses[i].translation[0];
        pose.position.y = traj.poses[i].translation[1];
        pose.position.z = traj.poses[i].translation[2];
        pose.orientation.w = 1.0;
        t.poses.push_back(pose);
        geometry_msgs::msg::Vector3 std;
        std.x = traj.positionStd[i][0];
        std.y = traj.positionStd[i][1];
        std.z = traj.positionStd[i][2];
        t.position_std.push_back(std);
      }
      out.trajectories.push_back(t);
    }
    m_pub->publish(out);
  }

  double m_horizon{3.0}, m_dt{0.25};
  std::unique_ptr<spite_d::Predictor> m_predictor;
  rclcpp::Subscription<spite_d::msg::ObstacleArray>::SharedPtr m_sub;
  rclcpp::Publisher<spite_d::msg::PredictedTrajectoryArray>::SharedPtr m_pub;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PredictionNode>());
  rclcpp::shutdown();
  return 0;
}
