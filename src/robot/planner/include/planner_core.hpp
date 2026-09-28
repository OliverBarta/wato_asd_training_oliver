#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

namespace robot
{

struct CellIndex {
  int x;
  int y;

  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}

  bool operator==(const CellIndex& other) const { return x == other.x && y == other.y; }
  bool operator!=(const CellIndex& other) const { return !(*this == other); }
};

struct CellIndexHash {
  std::size_t operator()(const CellIndex& idx) const {
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};

// a coordinate with a f_score
struct AStarNode {
  CellIndex index;
  double f_score;// g score + h score

  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

struct CompareF {
  bool operator()(const AStarNode& a, const AStarNode& b) const {
    return a.f_score > b.f_score;
  }
};

class PlannerCore {
  public:
    explicit PlannerCore(const rclcpp::Logger& logger);

    nav_msgs::msg::Path planPath(const nav_msgs::msg::OccupancyGrid& map, double start_x, double start_y, double goal_x, double goal_y, const std::string& frame_id);

  private:
    rclcpp::Logger logger_;

    static constexpr int8_t OBSTACLE_THRESHOLD = 30;

    bool worldToGrid(const nav_msgs::msg::OccupancyGrid& map, double wx, double wy, CellIndex& cell) const;

    void gridToWorld(const nav_msgs::msg::OccupancyGrid& map, const CellIndex& cell, double& wx, double& wy) const;

    bool inBounds(const nav_msgs::msg::OccupancyGrid& map, const CellIndex& cell) const;

    bool isFree(const nav_msgs::msg::OccupancyGrid& map, const CellIndex& cell) const;

    int8_t getCost(const nav_msgs::msg::OccupancyGrid& map, const CellIndex& cell) const;

    double distance(const CellIndex& a, const CellIndex& b) const;

    std::vector<CellIndex> getNeighbours(const CellIndex& cell) const;

    nav_msgs::msg::Path reconstructPath(const nav_msgs::msg::OccupancyGrid& map, const std::unordered_map<CellIndex, CellIndex, CellIndexHash>& came_from, const CellIndex& start, const CellIndex& goal, const std::string& frame_id) const;
};

}

#endif
