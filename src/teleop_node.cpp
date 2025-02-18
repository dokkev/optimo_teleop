
#include "optimo_teleop/teleop_node.hpp"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "optimo_msgs/msg/pose_elbow.hpp"
#include "optimo_msgs/srv/servo_cb.hpp"

#include <chrono>
#include <functional>
#include <vector>

using namespace std::chrono_literals;

OptimoTeleop::OptimoTeleop(const rclcpp::NodeOptions & options)
: Node("optimo_teleop", options)
{
  // Subscribe to a PoseStamped topic.
  pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/optimo/ee_pose_desired", 10,
    std::bind(&OptimoTeleop::pose_callback, this, std::placeholders::_1));

  // Publisher to send the converted PoseElbow messages.
  pose_pub_ = this->create_publisher<optimo_msgs::msg::PoseElbow>("/optimo/servo/ee_pose_desired", 10);

  // Create the service client for the servo_cb service.
  servo_client_ = this->create_client<optimo_msgs::srv::ServoCb>("/optimo/optimo_effort_controller/servo_cb");

  // Set up a one-shot timer to call the servo_cb service after 2 seconds.
  service_timer_ = this->create_wall_timer(
    2s, std::bind(&OptimoTeleop::call_servo_service, this));

  RCLCPP_INFO(this->get_logger(), "OptimoTeleop node initialized");
}

OptimoTeleop::~OptimoTeleop()
{
}

void OptimoTeleop::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  // Store the received pose.
  last_pose_ = *msg;

  // Convert PoseStamped to PoseElbow.
  optimo_msgs::msg::PoseElbow target;
  target.pose = msg->pose;
  // Set a default elbow angle (could be modified as needed).
  target.elbow_angle = 0.0;

  // Publish the target PoseElbow.
  pose_pub_->publish(target);

  RCLCPP_INFO(this->get_logger(), "Published PoseElbow target from PoseStamped");
}

void OptimoTeleop::call_servo_service()
{
  // Cancel the timer so that we only call the service once.
  service_timer_->cancel();

  // Wait for the servo_cb service to become available.
  while (!servo_client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for servo_cb service");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Waiting for servo_cb service to become available...");
  }

  // Create and populate the service request.
  auto request = std::make_shared<optimo_msgs::srv::ServoCb::Request>();
  request->duration = 10;  // For example, 10 seconds.
  // Specify the topic name that carries the PoseElbow messages.
  request->topic_name = "servo_target_pose";
  request->cartesian = true;  // Activate Cartesian servo mode.

  RCLCPP_INFO(this->get_logger(), "Calling servo_cb with topic '%s'", request->topic_name.c_str());

  // Call the service and wait for the result.
  auto future_result = servo_client_->async_send_request(request);
  try {
    auto response = future_result.get();
    if (response->success) {
      RCLCPP_INFO(this->get_logger(), "servo_cb service call succeeded, servo mode activated");
    } else {
      RCLCPP_ERROR(this->get_logger(), "servo_cb service call failed");
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Service call to servo_cb failed: %s", e.what());
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OptimoTeleop>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
