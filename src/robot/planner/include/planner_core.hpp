#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point.hpp"

#include <unordered_map>
#include <queue>
#include <vector>

struct CellIndex {
  int x;
  int y;

  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}

  bool operator==(const CellIndex &other) const {
    return (x == other.x && y == other.y);
  }
  bool operator!=(const CellIndex &other) const {
    return (x != other.x || y != other.y);
  }
};

struct CellIndexHash {
  std::size_t operator()(const CellIndex &idx) const {
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};

struct AStarNode {
  CellIndex index;
  double f_score;

  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

struct CompareF {
  bool operator()(const AStarNode &a, const AStarNode &b) {
    return a.f_score > b.f_score;
  }
};

namespace robot
{

class PlannerCore {
  public:
    explicit PlannerCore(const rclcpp::Logger& logger);
    
    bool planPath(const nav_msgs::msg::OccupancyGrid &map,
                  double start_x, double start_y,
                  double goal_x, double goal_y,
                  std::vector<geometry_msgs::msg::Point> &waypoints);

    bool isValid(const nav_msgs::msg::OccupancyGrid &map, const CellIndex &idx) const;
    CellIndex worldToGrid(const nav_msgs::msg::OccupancyGrid &map, double x, double y) const;

  private:
    rclcpp::Logger logger_;

    geometry_msgs::msg::Point gridToWorld(const nav_msgs::msg::OccupancyGrid &map, const CellIndex &idx) const;
    double heuristic(const CellIndex &a, const CellIndex &b) const;
    std::vector<CellIndex> getNeighbors(const CellIndex &idx) const;    
};

}  

#endif  
