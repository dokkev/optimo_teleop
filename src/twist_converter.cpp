.

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <Eigen/Geometry>
#include <memory>
#include <vector>
#include <string>
#include <mutex>


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::shutdown();
  return 0;
}
