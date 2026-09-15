#include "costmap_core.hpp"
#include <cmath>


namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger) {
      initializeCostmap();
}

void CostmapCore::initializeCostmap() {
  grid_.assign(height_, std::vector<int8_t>(width_, -1));  // -1 = unknown
}

void CostmapCore::convertToGrid(double range, double angle, int &x_grid, int &y_grid) {
  double x = range * std::cos(angle);
  double y = range * std::sin(angle);

  x_grid = static_cast<int>(std::floor((x - origin_x_) / resolution_));
  y_grid = static_cast<int>(std::floor((y - origin_y_) / resolution_));
  // these x,y  are relative to the pose
}

void CostmapCore::markFreeRay(int x0, int y0, int x1, int y1) {
  int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
  int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
  int err = dx + dy;

  while (x0 != x1 || y0 != y1) {
    if (x0 >= 0 && x0 < width_ && y0 >= 0 && y0 < height_) {
      if (grid_[y0][x0] < 0) grid_[y0][x0] = 0;  // only overwrite unknown, not obstacles
    }
    int e2 = 2 * err;
    if (e2 >= dy) { err += dy; x0 += sx; }
    if (e2 <= dx) { err += dx; y0 += sy; }
  }
}

void CostmapCore::markObstacle(int x_grid, int y_grid) {
  if (x_grid >= 0 && x_grid < width_ && y_grid >= 0 && y_grid < height_) {
    grid_[y_grid][x_grid] = max_cost_;
  }
}

void CostmapCore::inflateObstacles() {
  int cell_radius = static_cast<int>(inflation_radius_ / resolution_);
  auto original = grid_;  // read from a snapshot so inflation doesn't cascade

  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      if (original[y][x] != max_cost_) continue;

      for (int dy = -cell_radius; dy <= cell_radius; ++dy) {
        for (int dx = -cell_radius; dx <= cell_radius; ++dx) {
          int nx = x + dx;
          int ny = y + dy;
          if (nx < 0 || nx >= width_ || ny < 0 || ny >= height_) continue;

          double distance = std::sqrt(dx * dx + dy * dy) * resolution_;
          if (distance > inflation_radius_) continue;

          int8_t cost = static_cast<int8_t>(max_cost_ * (1.0 - distance / inflation_radius_));
          if (cost > grid_[ny][nx]) {
            grid_[ny][nx] = cost;
          }
        }
      }
    }
  }
}

nav_msgs::msg::OccupancyGrid CostmapCore::getOccupancyGrid(const std::string &frame_id, const rclcpp::Time &stamp) {
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
      msg.data[y * width_ + x] = grid_[y][x];
    }
  }

  return msg;
}

}