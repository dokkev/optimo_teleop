#include "optimo_teleop/teleop_node.hpp"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "optimo_msgs/msg/pose_elbow.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <chrono>
#include <functional>
#include <mutex>

using namespace std::chrono_literals;

OptimoTeleop::OptimoTeleop(const rclcpp::NodeOptions & options)
: Node("optimo_teleop", options)
{
    // Subscribe to current end-effector pose
    current_pose_subscriber_ = this->create_subscription<optimo_msgs::msg::PoseElbow>(
        "/optimo/ee_pose_current", 10,
        std::bind(&OptimoTeleop::current_pose_callback, this, std::placeholders::_1));

    // Subscribe to twist-based input for controlling the end-effector
    twist_subscriber_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "/optimo/servo/twist_cmd", 10,
        std::bind(&OptimoTeleop::twist_callback, this, std::placeholders::_1));

    // Publisher to send adjusted PoseElbow messages
    pose_pub_ = this->create_publisher<optimo_msgs::msg::PoseElbow>("/optimo/servo/ee_pose_desired", 10);

    RCLCPP_INFO(this->get_logger(), "OptimoTeleop node initialized.");
}

OptimoTeleop::~OptimoTeleop()
{
}

void OptimoTeleop::current_pose_callback(const optimo_msgs::msg::PoseElbow::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(pose_mutex_);
    current_pose_ = *msg;

    if (!initialized_)
    {
        desired_pose_ = current_pose_;
        initialized_ = true;
    }
}

void OptimoTeleop::twist_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(pose_mutex_);

    if (!initialized_)
    {
        RCLCPP_WARN(this->get_logger(), "Desired pose not initialized yet.");
        return;
    }

    // Update position using linear twist components.
    desired_pose_.pose.position.x += position_scale_ * msg->twist.linear.x;
    desired_pose_.pose.position.y += position_scale_ * msg->twist.linear.y;
    desired_pose_.pose.position.z += position_scale_ * msg->twist.linear.z;

    // Update orientation using angular twist components.
    // Convert current orientation to tf2 quaternion.
    // tf2::Quaternion q;
    // tf2::fromMsg(desired_pose_.pose.orientation, q);

    // Compute incremental rotation from twist angular values.
    // The orientation_scale_ factor determines the sensitivity of the angular update.
    // double delta_roll  = orientation_scale_ * msg->twist.angular.x;
    // double delta_pitch = orientation_scale_ * msg->twist.angular.y;
    // double delta_yaw   = orientation_scale_ * msg->twist.angular.z;

    // // Create a quaternion representing the incremental rotation.
    // tf2::Quaternion dq;
    // dq.setRPY(delta_roll, delta_pitch, delta_yaw);

    // // Update the current orientation by applying the incremental rotation.
    // q = dq * q;
    // q.normalize();  // Ensure the quaternion remains normalized.

    // // Convert back to a geometry_msgs quaternion.
    // desired_pose_.pose.orientation = tf2::toMsg(q);

    // set desired orientation to current orientation
    desired_pose_.pose.orientation = current_pose_.pose.orientation;


    desired_pose_.elbow_angle = 0.0;
    
    // Publish the updated desired PoseElbow.
    pose_pub_->publish(desired_pose_);

    RCLCPP_INFO(this->get_logger(), "Published adjusted PoseElbow: [%.2f, %.2f, %.2f] (timestamp: %d.%d)",
                desired_pose_.pose.position.x, desired_pose_.pose.position.y, desired_pose_.pose.position.z,
                msg->header.stamp.sec, msg->header.stamp.nanosec);
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OptimoTeleop>();

    // Process subscriber callbacks
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
