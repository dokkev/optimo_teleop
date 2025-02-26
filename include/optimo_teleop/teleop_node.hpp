#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "optimo_msgs/msg/pose_elbow.hpp"
#include <memory>
#include <mutex>

class OptimoTeleop : public rclcpp::Node
{
public:
  explicit OptimoTeleop(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~OptimoTeleop();

protected:
  void current_pose_callback(const optimo_msgs::msg::PoseElbow::SharedPtr msg);
  void twist_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

private:
  rclcpp::Subscription<optimo_msgs::msg::PoseElbow>::SharedPtr current_pose_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_subscriber_;
  rclcpp::Publisher<optimo_msgs::msg::PoseElbow>::SharedPtr pose_pub_;
  std::mutex pose_mutex_;
  optimo_msgs::msg::PoseElbow current_pose_;
  optimo_msgs::msg::PoseElbow desired_pose_;
  const double position_scale_ = 0.0001;
  const double orientation_scale_ = 0.0000001;

  bool initialized_ = false;
};
