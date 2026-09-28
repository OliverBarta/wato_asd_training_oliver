#include "planner_core.hpp"

#include <algorithm>
#include <cmath>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace robot
{

PlannerCore::PlannerCore(const rclcpp::Logger& logger)
: logger_(logger) {}

// finds the most optimal path using A* pathfinding algorithm, returns a nav_msgs::msg::Path with the path from start to goal
nav_msgs::msg::Path PlannerCore::planPath(const nav_msgs::msg::OccupancyGrid& map, double start_x, double start_y, double goal_x, double goal_y, const std::string& frame_id) {
  CellIndex start, goal;

  if (!worldToGrid(map, start_x, start_y, start) || !worldToGrid(map, goal_x, goal_y, goal)) {
    RCLCPP_WARN(logger_, "Start or goal is off the map.");
    return nav_msgs::msg::Path();
  }
  if (!isFree(map, goal)) {
    RCLCPP_WARN(logger_, "Goal is isn't free.");
    return nav_msgs::msg::Path();
  }

  CellIndex current = start;

  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_list;// a priority queue where the top() always has the lowest f_score

  std::unordered_set<CellIndex, CellIndexHash> closed;// a list of cells that have already been viewed

  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;// a map of where a CellIndex came from

  std::unordered_map<CellIndex, double, CellIndexHash> g_score;// a map of the cost to reach a CellIndex from the start

  open_list.push(AStarNode(start, distance(start, goal)));

  g_score[start] = 0.0;

  while(!open_list.empty()) {
    AStarNode node = open_list.top();
    open_list.pop();
    current = node.index;
    if (closed.count(current)) {
      continue; // already been current
    }
    closed.insert(current);
    if (current == goal) {
      RCLCPP_INFO(logger_, "Path found");
      return reconstructPath(map, came_from, start, goal, frame_id);
    }

    std::vector<CellIndex> neightbours = getNeighbours(current);

    for (const CellIndex& neighbour : neightbours) {
      bool escaping = !isFree(map, current) && inBounds(map, neighbour) && getCost(map, neighbour) >= 0 && getCost(map, neighbour) < getCost(map, current);
      if (!isFree(map, neighbour) && !escaping) {
        continue;
      }
      if (closed.count(neighbour)) {
        continue;// already been viewed
      }

      double tentative_g_score = g_score[current] + distance(current, neighbour);
      if (!g_score.count(neighbour) || tentative_g_score < g_score[neighbour]) {
        came_from[neighbour] = current;
        g_score[neighbour] = tentative_g_score;
        open_list.push(AStarNode(neighbour, tentative_g_score + distance(neighbour, goal)));
      }
    }

  }

  RCLCPP_WARN(logger_, "No path found.");
  return nav_msgs::msg::Path();
}

bool PlannerCore::worldToGrid(const nav_msgs::msg::OccupancyGrid& map, double wx, double wy, CellIndex& cell) const {
  int grid_x = floor((wx - map.info.origin.position.x)/map.info.resolution);
  int grid_y = floor((wy - map.info.origin.position.y)/map.info.resolution);

  cell = {grid_x, grid_y};
  
  if (cell.x < 0 || cell.x >= static_cast<int>(map.info.width) ||
      cell.y < 0 || cell.y >= static_cast<int>(map.info.height)) {
    return false;// out of bounds
  }
  
  return true;
}

void PlannerCore::gridToWorld(const nav_msgs::msg::OccupancyGrid& map, const CellIndex& cell, double& wx, double& wy) const {
  wx = map.info.origin.position.x + (cell.x + 0.5) * map.info.resolution;
  wy = map.info.origin.position.y + (cell.y + 0.5) * map.info.resolution;
  
}

bool PlannerCore::inBounds(const nav_msgs::msg::OccupancyGrid& map, const CellIndex& cell) const {
  return cell.x >= 0 && cell.x < static_cast<int>(map.info.width) &&
         cell.y >= 0 && cell.y < static_cast<int>(map.info.height);
}

bool PlannerCore::isFree(const nav_msgs::msg::OccupancyGrid& map, const CellIndex& cell) const {
  if (!inBounds(map, cell)) {
    return false;// out of bounds
  }
  if (getCost(map, cell) < 0 || getCost(map, cell) >= OBSTACLE_THRESHOLD) {
    return false;// unknown (0) or blocked
  }
  return true;
}

// cost of a cell because .data is a squished 2d array
int8_t PlannerCore::getCost(const nav_msgs::msg::OccupancyGrid& map, const CellIndex& cell) const {
  return map.data[cell.y * map.info.width + cell.x];
}

// distance between cells
double PlannerCore::distance(const CellIndex& a, const CellIndex& b) const {
  return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

// the eight neighbours of a cell (straight and diagonal)
std::vector<CellIndex> PlannerCore::getNeighbours(const CellIndex& cell) const {
  std::vector<CellIndex> neighbours;

  neighbours.push_back({cell.x - 1, cell.y});// left
  neighbours.push_back({cell.x + 1, cell.y});// right
  neighbours.push_back({cell.x, cell.y + 1});// up
  neighbours.push_back({cell.x, cell.y - 1});// down
  neighbours.push_back({cell.x - 1, cell.y + 1});// top left
  neighbours.push_back({cell.x + 1, cell.y + 1});// top right
  neighbours.push_back({cell.x - 1, cell.y - 1});// bottom left
  neighbours.push_back({cell.x + 1, cell.y - 1});// bottom right

  return neighbours;
}

nav_msgs::msg::Path PlannerCore::reconstructPath(const nav_msgs::msg::OccupancyGrid& map, const std::unordered_map<CellIndex, CellIndex, CellIndexHash>& came_from, const CellIndex& start, const CellIndex& goal, const std::string& frame_id) const {
  // Walk the parent links back from goal to start
  std::vector<CellIndex> cells;
  CellIndex current = goal;
  while (current != start) {
    cells.push_back(current);
    current = came_from.at(current);
  }
  cells.push_back(start);

  // collected goal -> start, so flip it to start -> goal
  std::reverse(cells.begin(), cells.end());

  nav_msgs::msg::Path path;
  path.header.frame_id = frame_id;

  for (const CellIndex& cell : cells) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = frame_id;
    gridToWorld(map, cell, pose.pose.position.x, pose.pose.position.y);
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }

  return path;
}

}
