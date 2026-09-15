#include <chrono>
#include <memory>
 
#include "costmap_node.hpp"
 
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Initialize the constructs and their parameters
  string_pub_ = this->create_publisher<std_msgs::msg::String>("/test_topic", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CostmapNode::publishMessage, this));


  //lidar
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10, std::bind(&CostmapNode::laserScanCallback, this, std::placeholders::_1));

  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);

}
 
// Define the timer to publish a message every 500ms
void CostmapNode::publishMessage() {
  auto message = std_msgs::msg::String();
  message.data = "Hello, ROS 2!";
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  string_pub_->publish(message);
}

//lidar cb function, on receive
void CostmapNode::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  // process the scan here

    // Step 1: Initialize costmap
  costmap_.initializeCostmap();

  // Step 2: Convert LaserScan to grid and mark obstacles
  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    double angle = scan->angle_min + i * scan->angle_increment;
    double range = scan->ranges[i];

    if (range > scan->range_min && range < scan->range_max) {
      int x_grid, y_grid;
      costmap_.convertToGrid(range, angle, x_grid, y_grid);
      costmap_.markObstacle(x_grid, y_grid);
    }
  }

  // Step 3: Inflate obstacles
  costmap_.inflateObstacles();

  // Step 4: Publish costmap
  auto grid_msg = costmap_.getOccupancyGrid(scan->header.frame_id, scan->header.stamp);
  costmap_pub_->publish(grid_msg);
}
 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}