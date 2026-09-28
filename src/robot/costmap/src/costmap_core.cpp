#include "costmap_core.hpp"
#include <cmath>

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger) {}

void CostmapCore::initializeCostmap() {
  resolution_ = 0.1;
  width_ = 100;
  height_ = 100;

  resetCostmap();

  RCLCPP_INFO(logger_, "Costmap initialized: %d x %d cells at %.2f m/cell", width_, height_, resolution_);
}

void CostmapCore::resetCostmap() {
  costmap_ = std::vector<std::vector<int>>(height_, std::vector<int>(width_, 0));
  obstacle_cells_.clear();
}

void CostmapCore::convertToGrid(double range, double angle, int &x_grid, int &y_grid) {
  double x = range * std::cos(angle);
  double y = range * std::sin(angle);

  double origin_x = -(width_ * resolution_) / 2.0;
  double origin_y = -(height_ * resolution_) / 2.0;

  x_grid = static_cast<int>((x - origin_x) / resolution_);
  y_grid = static_cast<int>((y - origin_y) / resolution_);
}

void CostmapCore::markObstacle(int x_grid, int y_grid) {
  if (x_grid < 0 || x_grid >= width_ || y_grid < 0 || y_grid >= height_) {
    return;
  }

  costmap_[y_grid][x_grid] = max_cost_;
  obstacle_cells_.push_back({x_grid, y_grid});
}

void CostmapCore::inflateObstacles() {
  int radius_cells = static_cast<int>(inflation_radius_ / resolution_);

  for (const auto &obstacle : obstacle_cells_) {
    int ox = obstacle.first;
    int oy = obstacle.second;

    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
      for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
        int nx = ox + dx;
        int ny = oy + dy;

        if (nx < 0 || nx >= width_ || ny < 0 || ny >= height_) {
          continue;
        }

        double distance = std::sqrt(dx * dx + dy * dy) * resolution_;

        if (distance > inflation_radius_) {
          continue;
        }

        int cost = static_cast<int>(max_cost_ * (1.0 - (distance / inflation_radius_)));

        if (cost > costmap_[ny][nx]) {
          costmap_[ny][nx] = cost;
        }
      }
    }
  }
}

}