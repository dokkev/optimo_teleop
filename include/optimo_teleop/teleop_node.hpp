// File: include/optimo_teleop/teleop_node.hpp
#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "optimo_msgs/msg/pose_elbow.hpp"
#include "optimo_msgs/srv/servo_cb.hpp"
#include <memory>

/**
 * @brief The OptimoTeleop node subscribes to PoseStamped messages,
 * converts them to PoseElbow messages, publishes them on a topic that the servo
 * callback listens to, and then calls the servo_cb service to activate servo mode.
 */
class OptimoTeleop : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for OptimoTeleop.
   * @param options Optional node options.
   */
  explicit OptimoTeleop(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destructor for OptimoTeleop.
   */
  virtual ~OptimoTeleop();

protected:
  /**
   * @brief Callback function for processing PoseStamped messages.
   *
   * This function converts the received PoseStamped message to a PoseElbow message.
   *
   * @param msg Shared pointer to the received PoseStamped message.
   */
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  /**
   * @brief Calls the servo_cb service to activate servo mode.
   *
   * The service request passes the topic name "servo_target_pose" (which carries the PoseElbow
   * messages), a duration, and a flag indicating Cartesian mode.
   */
  void call_servo_service();

private:
  // Subscriber for incoming PoseStamped messages.
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;

  // Publisher for sending converted PoseElbow messages.
  rclcpp::Publisher<optimo_msgs::msg::PoseElbow>::SharedPtr pose_pub_;

  // Service client to call the servo_cb service.
  rclcpp::Client<optimo_msgs::srv::ServoCb>::SharedPtr servo_client_;

  // One-shot timer to call the service after a short delay.
  rclcpp::TimerBase::SharedPtr service_timer_;

  // Store the last received PoseStamped.
  geometry_msgs::msg::PoseStamped last_pose_;
};
