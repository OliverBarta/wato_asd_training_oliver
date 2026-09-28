#include "map_memory_core.hpp"
#include <algorithm>
#include <cmath>

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger) 
  : logger_(logger) {}

void MapMemoryCore::initializeGlobalMap(double resolution, int width, int height) {
  resolution_ = resolution;
  width_ = width;
  height_ = height;
  origin_x_ = -(width_ * resolution_) / 2.0;
  origin_y_ = -(height_ * resolution_) / 2.0;

  global_map_ = std::vector<std::vector<int8_t>>(height_, std::vector<int8_t>(width_, 0));

  RCLCPP_INFO(logger_, "Global map initialized: %d x %d cells at %.2f m/cell", width_, height_, resolution_);
}

void MapMemoryCore::integrateCostmap(const nav_msgs::msg::OccupancyGrid& costmap, double robot_x, double robot_y, double robot_theta) {
  int local_width = costmap.info.width;
  int local_height = costmap.info.height;
  double local_res = costmap.info.resolution;
  double local_origin_x = costmap.info.origin.position.x;
  double local_origin_y = costmap.info.origin.position.y;

  double cos_theta = std::cos(robot_theta);
  double sin_theta = std::sin(robot_theta);

  // Only trust cells inside the circle the lidar can see (the costmap's
  // half-width), not the square's corners
  double view_radius = std::min(local_width, local_height) * local_res / 2.0;

  // Loop over the global cells around the robot and look each one up in the
  // local costmap (global -> local), rather than pushing local cells out to the
  // global grid. Pushing a rotated grid out leaves some global cells never
  // written, which showed up as zero-cost "holes" inside obstacles.
  int gx_min = std::max(0, static_cast<int>(std::floor((robot_x - view_radius - origin_x_) / resolution_)));
  int gx_max = std::min(width_ - 1, static_cast<int>(std::floor((robot_x + view_radius - origin_x_) / resolution_)));
  int gy_min = std::max(0, static_cast<int>(std::floor((robot_y - view_radius - origin_y_) / resolution_)));
  int gy_max = std::min(height_ - 1, static_cast<int>(std::floor((robot_y + view_radius - origin_y_) / resolution_)));

  for (int gy = gy_min; gy <= gy_max; ++gy) {
    for (int gx = gx_min; gx <= gx_max; ++gx) {
      // Global cell centre -> offset from the robot in the global frame
      double dx = origin_x_ + (gx + 0.5) * resolution_ - robot_x;
      double dy = origin_y_ + (gy + 0.5) * resolution_ - robot_y;

      if (std::hypot(dx, dy) > view_radius) {
        continue;
      }

      // Rotate into the robot's (costmap's) frame
      double local_wx = dx * cos_theta + dy * sin_theta;
      double local_wy = -dx * sin_theta + dy * cos_theta;

      int lx = static_cast<int>(std::floor((local_wx - local_origin_x) / local_res));
      int ly = static_cast<int>(std::floor((local_wy - local_origin_y) / local_res));

      if (lx < 0 || lx >= local_width || ly < 0 || ly >= local_height) {
        continue;
      }

      int8_t value = costmap.data[ly * local_width + lx];
      if (value < 0) {
        continue;  // unknown, keep whatever the global map already has
      }

      // New scan overwrites old data, including free cells, so ghost obstacles
      // left by earlier bad merges get erased when the robot looks again
      global_map_[gy][gx] = value;
    }
  }
}

nav_msgs::msg::OccupancyGrid MapMemoryCore::getGlobalMapMessage(const std::string& frame_id, rclcpp::Time stamp) const {
  auto msg = nav_msgs::msg::OccupancyGrid();

  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;

  msg.info.resolution = resolution_;
  msg.info.width = width_;
  msg.info.height = height_;
  msg.info.origin.position.x = origin_x_;
  msg.info.origin.position.y = origin_y_;
  msg.info.origin.position.z = 0.0;
  msg.info.origin.orientation.w = 1.0;

  msg.data.resize(width_ * height_);
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      msg.data[y * width_ + x] = global_map_[y][x];
    }
  }

  return msg;
}

}