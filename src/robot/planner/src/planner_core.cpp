#include "planner_core.hpp"
#include <cmath>
#include <algorithm>

namespace robot
{

PlannerCore::PlannerCore(const rclcpp::Logger& logger) 
: logger_(logger) {}
CellIndex PlannerCore::worldToGrid(const nav_msgs::msg::OccupancyGrid &map, double x, double y) const {
int gx = static_cast<int>(std::floor((x - map.info.origin.position.x) / map.info.resolution));
int gy = static_cast<int>(std::floor((y - map.info.origin.position.y) / map.info.resolution));
  return CellIndex(gx, gy);
}

geometry_msgs::msg::Point PlannerCore::gridToWorld(const nav_msgs::msg::OccupancyGrid &map, const CellIndex &idx) const {
  geometry_msgs::msg::Point p;
  p.x = map.info.origin.position.x + (idx.x + 0.5) * map.info.resolution;
  p.y = map.info.origin.position.y + (idx.y + 0.5) * map.info.resolution;
  p.z = 0.0;
  return p;
}

bool PlannerCore::isValid(const nav_msgs::msg::OccupancyGrid &map, const CellIndex &idx) const {
  
  if (idx.x < 0 || idx.x >= static_cast<int>(map.info.width) ||
      idx.y < 0 || idx.y >= static_cast<int>(map.info.height)) {
    return false;
  }
  int8_t value = map.data[idx.y * map.info.width + idx.x];
  // Treat unknown (-1) as free/traversable; treat high cost as blocked.
  return value < 50;
}

double PlannerCore::heuristic(const CellIndex &a, const CellIndex &b) const {
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);  // Euclidean
}

std::vector<CellIndex> PlannerCore::getNeighbors(const CellIndex &idx) const {
  return {
    CellIndex(idx.x + 1, idx.y),     CellIndex(idx.x - 1, idx.y),
    CellIndex(idx.x, idx.y + 1),     CellIndex(idx.x, idx.y - 1),
    CellIndex(idx.x + 1, idx.y + 1), CellIndex(idx.x - 1, idx.y - 1),
    CellIndex(idx.x + 1, idx.y - 1), CellIndex(idx.x - 1, idx.y + 1)
  };
}

bool PlannerCore::planPath(const nav_msgs::msg::OccupancyGrid &map,
                           double start_x, double start_y,
                           double goal_x, double goal_y,
                           std::vector<geometry_msgs::msg::Point> &waypoints) {
  waypoints.clear();

  CellIndex start = worldToGrid(map, start_x, start_y);
  CellIndex goal = worldToGrid(map, goal_x, goal_y);

  if (!isValid(map, start) || !isValid(map, goal)) {
    RCLCPP_WARN(logger_, "Start or goal cell is invalid/occupied");
    return false;
  }

  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;
  std::unordered_map<CellIndex, double, CellIndexHash> g_score;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
  std::unordered_map<CellIndex, bool, CellIndexHash> closed;

  g_score[start] = 0.0;
  open_set.emplace(start, heuristic(start, goal));

  while (!open_set.empty()) {
    CellIndex current = open_set.top().index;
    open_set.pop();

    if (closed[current]) continue;
    closed[current] = true;

    if (current == goal) {
      // Reconstruct path
      std::vector<CellIndex> path_cells;
      CellIndex c = current;
      while (!(c == start)) {
        path_cells.push_back(c);
        c = came_from[c];
      }
      path_cells.push_back(start);
      std::reverse(path_cells.begin(), path_cells.end());

      for (const auto &cell : path_cells) {
        waypoints.push_back(gridToWorld(map, cell));
      }
      return true;
    }

    for (const auto &neighbor : getNeighbors(current)) {
      if (!isValid(map, neighbor) || closed[neighbor]) continue;

      double move_cost = (neighbor.x != current.x && neighbor.y != current.y) ? std::sqrt(2.0) : 1.0;
      double tentative_g = g_score[current] + move_cost;

      if (!g_score.count(neighbor) || tentative_g < g_score[neighbor]) {
        g_score[neighbor] = tentative_g;
        came_from[neighbor] = current;
        double f = tentative_g + heuristic(neighbor, goal);
        open_set.emplace(neighbor, f);
      }
    }
  }

  RCLCPP_WARN(logger_, "No path found to goal");
  return false;
}

} 
