#include "control_core.hpp"
#include <cmath>

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger) 
  : logger_(logger) {}
double ControlCore::computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) {
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

double ControlCore::extractYaw(const geometry_msgs::msg::Quaternion &quat) {
  // Standard quaternion -> yaw (Z-axis rotation) conversion
  double siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y);
  double cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

std::optional<geometry_msgs::msg::PoseStamped> ControlCore::findLookaheadPoint(
    const nav_msgs::msg::Path &path, const geometry_msgs::msg::Pose &robot_pose) {
  if (path.poses.empty()) {
    return std::nullopt;
  }

  // Find the point on the path closest to the robot, so we never
  // snap back to an earlier point we've already passed.
  size_t closest_idx = 0;
  double closest_dist = computeDistance(robot_pose.position, path.poses[0].pose.position);
  for (size_t i = 1; i < path.poses.size(); ++i) {
    double d = computeDistance(robot_pose.position, path.poses[i].pose.position);
    if (d < closest_dist) {
      closest_dist = d;
      closest_idx = i;
    }
  }

  // Walk forward from the closest point looking for the lookahead point.
  for (size_t i = closest_idx; i < path.poses.size(); ++i) {
    double dist = computeDistance(robot_pose.position, path.poses[i].pose.position);
    if (dist >= lookahead_distance) {
      return path.poses[i];
    }
  }

  return path.poses.back();
}

geometry_msgs::msg::Twist ControlCore::computeVelocity(
    const geometry_msgs::msg::PoseStamped &target, const geometry_msgs::msg::Pose &robot_pose) {
  geometry_msgs::msg::Twist cmd_vel;

  double robot_yaw = extractYaw(robot_pose.orientation);

  // Transform the lookahead point into the robot's local frame
  double dx = target.pose.position.x - robot_pose.position.x;
  double dy = target.pose.position.y - robot_pose.position.y;

  double local_x = dx * std::cos(-robot_yaw) - dy * std::sin(-robot_yaw);
  double local_y = dx * std::sin(-robot_yaw) + dy * std::cos(-robot_yaw);

  double distance_sq = local_x * local_x + local_y * local_y;
  if (distance_sq < 1e-6) {
    // Target essentially on top of the robot; nothing meaningful to steer toward
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    return cmd_vel;
  }

  // Pure pursuit curvature: kappa = 2 * y / L^2, where L is the actual distance
  // to the target (not the nominal lookahead_distance) and y is the lateral offset.
  double curvature = (2.0 * local_y) / distance_sq;

  cmd_vel.linear.x = linear_speed;
  cmd_vel.angular.z = curvature * linear_speed;

  return cmd_vel;
}
}  
