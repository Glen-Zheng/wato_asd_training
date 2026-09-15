#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <optional>

namespace robot
{

class ControlCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit ControlCore(const rclcpp::Logger& logger);

    std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint(
      const nav_msgs::msg::Path &path, const geometry_msgs::msg::Pose &robot_pose);

    geometry_msgs::msg::Twist computeVelocity(
      const geometry_msgs::msg::PoseStamped &target, const geometry_msgs::msg::Pose &robot_pose);

    double computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b);
    double extractYaw(const geometry_msgs::msg::Quaternion &quat);

    double lookahead_distance = 1.0;
    double linear_speed = 0.5;    

  
  private:
    rclcpp::Logger logger_;
};

} 

#endif 
