#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
namespace robot
{

class CostmapCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);


    //costmap logic
    void initializeCostmap();
    void convertToGrid(double range, double angle, int &x_grid, int &y_grid);
    void markObstacle(int x_grid, int y_grid);
    void inflateObstacles();

    nav_msgs::msg::OccupancyGrid getOccupancyGrid(const std::string &frame_id, const rclcpp::Time &stamp);

  private:
    rclcpp::Logger logger_;


    double resolution_ = 0.1;
    int width_ = 100;
    int height_ = 100;
    double origin_x_ = -5.0;
    double origin_y_ = -5.0;
    double inflation_radius_ = 1.0;
    int8_t max_cost_ = 100;

    std::vector<std::vector<int8_t>> grid_;

};

}  

#endif  