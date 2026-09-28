#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <vector>

namespace robot
{

class CostmapCore {
  public:
    explicit CostmapCore(const rclcpp::Logger& logger);

    void initializeCostmap();
    void resetCostmap();
    void convertToGrid(double range, double angle, int &x_grid, int &y_grid);
    void markObstacle(int x_grid, int y_grid);
    void inflateObstacles();

    const std::vector<std::vector<int>>& getGrid() const { return costmap_; }
    double getResolution() const { return resolution_; }
    int getWidth() const { return width_; }
    int getHeight() const { return height_; }
    double getOriginX() const { return -(width_ * resolution_) / 2.0; }
    double getOriginY() const { return -(height_ * resolution_) / 2.0; }

  private:
    rclcpp::Logger logger_;

    std::vector<std::vector<int>> costmap_;
    double resolution_;
    int width_;
    int height_;

    std::vector<std::pair<int, int>> obstacle_cells_;
    double inflation_radius_ = 3;// meters
    int max_cost_ = 100;
};

}

#endif