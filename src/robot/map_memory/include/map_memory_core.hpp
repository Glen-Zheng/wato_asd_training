#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{

class MapMemoryCore {
  public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);

    void initializeGlobalMap();
    void integrateCostmap(const nav_msgs::msg::OccupancyGrid &local_costmap,
                          double robot_x, double robot_y, double robot_yaw);

    nav_msgs::msg::OccupancyGrid getGlobalMap(const std::string &frame_id, const rclcpp::Time &stamp);

  private:
    rclcpp::Logger logger_;

    double resolution_ = 0.1;
    int width_ = 300;
    int height_ = 300;
    double origin_x_ = -15.0;
    double origin_y_ = -15.0;

    std::vector<std::vector<int8_t>> global_grid_;
};

}  

#endif  
