#include "map_memory_core.hpp"
#include <cmath>

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger) 
  : logger_(logger) {  initializeGlobalMap();
}

void MapMemoryCore::initializeGlobalMap() {
  global_grid_.assign(height_, std::vector<int8_t>(width_, -1));  // -1 = unknown
}

void MapMemoryCore::integrateCostmap(const nav_msgs::msg::OccupancyGrid &local_costmap,
                                     double robot_x, double robot_y, double robot_yaw) {
  double local_res = local_costmap.info.resolution;
  double local_origin_x = local_costmap.info.origin.position.x;
  double local_origin_y = local_costmap.info.origin.position.y;
  int local_width = local_costmap.info.width;
  int local_height = local_costmap.info.height;

  for (int ly = 0; ly < local_height; ++ly) {
    for (int lx = 0; lx < local_width; ++lx) {
      int8_t value = local_costmap.data[ly * local_width + lx];
      if (value < 0) continue;  // unknown in the local costmap: don't touch the global cell

      // Local cell -> local (x, y) in the robot/lidar frame
      double local_x = local_origin_x + (lx + 0.5) * local_res;
      double local_y = local_origin_y + (ly + 0.5) * local_res;

      // Rotate by robot yaw, then translate by robot position -> global (x, y)
      double global_x = robot_x + local_x * std::cos(robot_yaw) - local_y * std::sin(robot_yaw);
      double global_y = robot_y + local_x * std::sin(robot_yaw) + local_y * std::cos(robot_yaw);

      // Global (x, y) -> global grid indices
      int gx = static_cast<int>((global_x - origin_x_) / resolution_);
      int gy = static_cast<int>((global_y - origin_y_) / resolution_);

      if (gx >= 0 && gx < width_ && gy >= 0 && gy < height_) {
        global_grid_[gy][gx] = value;  // new data overwrites old (known values only)
      }
    }
  }
}

nav_msgs::msg::OccupancyGrid MapMemoryCore::getGlobalMap(const std::string &frame_id, const rclcpp::Time &stamp) {
  nav_msgs::msg::OccupancyGrid msg;

  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;

  msg.info.resolution = resolution_;
  msg.info.width = width_;
  msg.info.height = height_;
  msg.info.origin.position.x = origin_x_;
  msg.info.origin.position.y = origin_y_;

  msg.data.resize(width_ * height_);
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      msg.data[y * width_ + x] = global_grid_[y][x];
    }
  }

  return msg;
}

} 
