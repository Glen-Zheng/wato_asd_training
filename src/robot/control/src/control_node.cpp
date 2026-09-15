#include "control_node.hpp"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore(this->get_logger())) {
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path", 10, [this](const nav_msgs::msg::Path::SharedPtr msg) { current_path_ = msg; });

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) { robot_odom_ = msg; });

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  control_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&ControlNode::controlLoop, this));  
}
void ControlNode::controlLoop() {
  if (!current_path_ || !robot_odom_ || current_path_->poses.empty()) {
    return;
  }

  const auto &robot_pose = robot_odom_->pose.pose;

  // Stop if we're within tolerance of the final goal
  double dist_to_goal = control_.computeDistance(
    robot_pose.position, current_path_->poses.back().pose.position);
  if (dist_to_goal < goal_tolerance_) {
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist());  // zero velocity
    return;
  }

  auto lookahead_point = control_.findLookaheadPoint(*current_path_, robot_pose);
  if (!lookahead_point) {
    return;
  }

  auto cmd_vel = control_.computeVelocity(*lookahead_point, robot_pose);
  cmd_vel_pub_->publish(cmd_vel);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
