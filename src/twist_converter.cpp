// twist_converter.cpp
//
// This node subscribes to normalized Twist messages (values in [0, 1]) 
// and to a topic that publishes the current end‐effector pose (PoseStamped).
// It uses the most recent pose as the base, applies scaled twist increments,
// and publishes an updated PoseStamped message.
//
// References:
//   - Craig, J. J. (2005). Introduction to Robotics: Mechanics and Control.
//   - Siciliano, B., & Khatib, O. (Eds.). (2016). Springer Handbook of Robotics.

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <Eigen/Geometry>
#include <memory>
#include <mutex>

class TwistConverter : public rclcpp::Node
{
public:
  TwistConverter() : Node("twist_converter"), has_latest_pose_(false)
  {
    // Declare and retrieve parameters.
    this->declare_parameter<double>("linear_scaling", 0.1);
    this->declare_parameter<double>("angular_scaling", 0.1);
    this->declare_parameter<std::string>("base_frame", "base_link");
    this->declare_parameter<std::string>("ee_pose_topic", "/optimo/ee_pose");

    this->get_parameter("linear_scaling", linear_scaling_);
    this->get_parameter("angular_scaling", angular_scaling_);
    this->get_parameter("base_frame", base_frame_);
    this->get_parameter("ee_pose_topic", ee_pose_topic_);

    // Publisher: publish the updated PoseStamped message.
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/optimo/ee_pose_desired", 10);

    // Subscriber: listen to normalized Twist messages.
    twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/optimo/servo_node/delta_twist_cmds", 10,
      std::bind(&TwistConverter::twist_callback, this, std::placeholders::_1));

    // Subscriber: listen to the current end-effector PoseStamped.
    ee_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      ee_pose_topic_, 10,
      std::bind(&TwistConverter::ee_pose_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "TwistConverter node initialized (ee_pose_topic: %s)", ee_pose_topic_.c_str());
  }

private:
  // Callback to update the latest end-effector pose.
  void ee_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg)
  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    latest_pose_ = *pose_msg;
    has_latest_pose_ = true;
    RCLCPP_DEBUG(this->get_logger(), "Received new end-effector pose.");
  }

  // Callback to process the incoming twist command.
  void twist_callback(const geometry_msgs::msg::Twist::SharedPtr twist_msg)
  {
    if (!has_latest_pose_) {
      RCLCPP_WARN(this->get_logger(), "No latest pose available yet; cannot update pose.");
      return;
    }

    geometry_msgs::msg::PoseStamped base_pose;
    {
      std::lock_guard<std::mutex> lock(pose_mutex_);
      base_pose = latest_pose_;
    }

    geometry_msgs::msg::PoseStamped new_pose;
    new_pose.header.stamp = this->now();
    new_pose.header.frame_id = base_pose.header.frame_id;

    // Update position using scaled twist increments.
    new_pose.pose.position.x = base_pose.pose.position.x + twist_msg->linear.x * linear_scaling_;
    new_pose.pose.position.y = base_pose.pose.position.y + twist_msg->linear.y * linear_scaling_;
    new_pose.pose.position.z = base_pose.pose.position.z + twist_msg->linear.z * linear_scaling_;

    // Update orientation.
    // Convert the base orientation to an Eigen quaternion.
    Eigen::Quaterniond base_q(
      base_pose.pose.orientation.w,
      base_pose.pose.orientation.x,
      base_pose.pose.orientation.y,
      base_pose.pose.orientation.z);

    // Create an incremental rotation from the twist's angular components.
    Eigen::AngleAxisd rot_x(twist_msg->angular.x * angular_scaling_, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd rot_y(twist_msg->angular.y * angular_scaling_, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rot_z(twist_msg->angular.z * angular_scaling_, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond delta_q = rot_z * rot_y * rot_x;

    // Compute the new orientation.
    Eigen::Quaterniond new_q = delta_q * base_q;
    new_q.normalize();
    new_pose.pose.orientation.x = new_q.x();
    new_pose.pose.orientation.y = new_q.y();
    new_pose.pose.orientation.z = new_q.z();
    new_pose.pose.orientation.w = new_q.w();

    // Publish the updated pose.
    pose_pub_->publish(new_pose);
    RCLCPP_INFO(this->get_logger(), "Published updated PoseStamped based on twist command.");
  }

  // Parameters.
  double linear_scaling_;
  double angular_scaling_;
  std::string base_frame_;
  std::string ee_pose_topic_;

  // ROS publisher and subscribers.
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ee_pose_sub_;

  // Latest end-effector pose.
  geometry_msgs::msg::PoseStamped latest_pose_;
  bool has_latest_pose_;
  std::mutex pose_mutex_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TwistConverter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
