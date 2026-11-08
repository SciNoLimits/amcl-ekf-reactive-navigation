#include <gz/transport/Node.hh>
#include <gz/msgs/pose_v.pb.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

class GroundTruthPublisher final : public rclcpp::Node
{
public:
  GroundTruthPublisher() : Node("gazebo_ground_truth_publisher")
  {
    topic_ = declare_parameter<std::string>(
      "gazebo_topic", "/world/default/dynamic_pose/info");
    entity_ = declare_parameter<std::string>("entity_name", "robot");
    frame_ = declare_parameter<std::string>("frame_id", "world");

    publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/ground_truth/pose", rclcpp::QoS(50));

    if (!gz_node_.Subscribe(
        topic_, &GroundTruthPublisher::on_pose, this)) {
      throw std::runtime_error("Failed to subscribe to " + topic_);
    }

    RCLCPP_INFO(
      get_logger(), "Publishing Gazebo entity '%s' from '%s' to /ground_truth/pose",
      entity_.c_str(), topic_.c_str());
  }

private:
  void on_pose(const gz::msgs::Pose_V & msg)
  {
    if (!msg.has_header()) {
      return;
    }

    for (int i = 0; i < msg.pose_size(); ++i) {
      const auto & pose = msg.pose(i);
      if (pose.name() != entity_) {
        continue;
      }

      const auto & stamp = msg.header().stamp();
      geometry_msgs::msg::PoseStamped out;
      out.header.stamp.sec = static_cast<int32_t>(stamp.sec());
      out.header.stamp.nanosec = static_cast<uint32_t>(stamp.nsec());
      out.header.frame_id = frame_;

      out.pose.position.x = pose.position().x();
      out.pose.position.y = pose.position().y();
      out.pose.position.z = pose.position().z();
      out.pose.orientation.x = pose.orientation().x();
      out.pose.orientation.y = pose.orientation().y();
      out.pose.orientation.z = pose.orientation().z();
      out.pose.orientation.w = pose.orientation().w();

      publisher_->publish(out);
      return;
    }
  }

  std::string topic_;
  std::string entity_;
  std::string frame_;
  gz::transport::Node gz_node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<GroundTruthPublisher>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("ekf_ground_truth"), "%s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
