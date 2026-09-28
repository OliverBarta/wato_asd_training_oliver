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

// A cell in the occupancy grid, by column (x) and row (y).
struct CellIndex {
  int x;
  int y;

  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}

  bool operator==(const CellIndex& other) const { return x == other.x && y == other.y; }
  bool operator!=(const CellIndex& other) const { return !(*this == other); }
};

// Hash for CellIndex so it can be used as a key in std::unordered_map / unordered_set.
struct CellIndexHash {
  std::size_t operator()(const CellIndex& idx) const {
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};

// An entry in the A* open list: a cell plus its f = g + h score.
struct AStarNode {
  CellIndex index;
  double f_score;

  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

// Comparator for std::priority_queue. priority_queue pops the "largest" element,
// so returning a > b here makes it pop the node with the SMALLEST f_score first.
struct CompareF {
  bool operator()(const AStarNode& a, const AStarNode& b) const {
    return a.f_score > b.f_score;
  }
};

class PlannerCore {
  public:
    explicit PlannerCore(const rclcpp::Logger& logger);

    // Runs A* on `map` from (start_x, start_y) to (goal_x, goal_y), both in world
    // metres in the map's frame. Returns the path as a list of poses with
    // header.frame_id = frame_id. Returns an empty Path if the start/goal is
    // off the map or blocked, or if no path exists.
    nav_msgs::msg::Path planPath(const nav_msgs::msg::OccupancyGrid& map,
                                 double start_x, double start_y,
                                 double goal_x, double goal_y,
                                 const std::string& frame_id);

  private:
    rclcpp::Logger logger_;

    // Cell values at or above this count as an obstacle and can't be entered.
    // Unknown cells (-1) are also treated as blocked.
    static constexpr int8_t OBSTACLE_THRESHOLD = 30;

    // Converts a world position (metres) to a grid cell using map.info.origin
    // and map.info.resolution. Writes the result into `cell` and returns false
    // if the position falls outside the grid.
    bool worldToGrid(const nav_msgs::msg::OccupancyGrid& map,
                     double wx, double wy, CellIndex& cell) const;

    // Converts a grid cell back to the world position (metres) of its centre.
    void gridToWorld(const nav_msgs::msg::OccupancyGrid& map,
                     const CellIndex& cell, double& wx, double& wy) const;

    // True if `cell` is inside the grid.
    bool inBounds(const nav_msgs::msg::OccupancyGrid& map, const CellIndex& cell) const;

    // True if `cell` is inside the grid and its value is known and below
    // OBSTACLE_THRESHOLD, i.e. the robot can drive through it.
    bool isFree(const nav_msgs::msg::OccupancyGrid& map, const CellIndex& cell) const;

    // Returns the cost value of `cell` from map.data (index = y * width + x).
    int8_t getCost(const nav_msgs::msg::OccupancyGrid& map, const CellIndex& cell) const;

    // A* heuristic h(n): straight-line (Euclidean) distance between two cells,
    // in cells.
    double distance(const CellIndex& a, const CellIndex& b) const;

    // Returns the 8 cells surrounding `cell` (straight and diagonal).
    // Doesn't check bounds or obstacles; isFree() does that.
    std::vector<CellIndex> getNeighbours(const CellIndex& cell) const;

    // Walks came_from backwards from `goal` to `start`, reverses it, and
    // converts each cell into a PoseStamped (via gridToWorld) in a Path message.
    nav_msgs::msg::Path reconstructPath(
        const nav_msgs::msg::OccupancyGrid& map,
        const std::unordered_map<CellIndex, CellIndex, CellIndexHash>& came_from,
        const CellIndex& start, const CellIndex& goal,
        const std::string& frame_id) const;
};

}

#endif
