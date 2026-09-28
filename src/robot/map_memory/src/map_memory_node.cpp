#include "map_memory_node.hpp"
#include <algorithm>
#include <cmath>

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  map_memory_.initializeGlobalMap(0.1, 300, 300);  // bigger than the local costmap since it persists over the whole run

  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&MapMemoryNode::updateMap, this));
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  latest_costmap_ = *msg;
  costmap_received_ = true;

  // Snapshot the pose now, while it still matches the scan this costmap came
  // from. Reading it later in the timer lets the robot move/turn in between,
  // which smears rotated "ghost" copies of obstacles into the global map.
  costmap_x_ = robot_x_;
  costmap_y_ = robot_y_;
  costmap_theta_ = robot_theta_;
  costmap_yaw_rate_ = yaw_rate_;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;

  // Extract yaw (rotation around z) directly from the quaternion
  double qx = msg->pose.pose.orientation.x;
  double qy = msg->pose.pose.orientation.y;
  double qz = msg->pose.pose.orientation.z;
  double qw = msg->pose.pose.orientation.w;
  double theta = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));

  // Turn rate from consecutive odometry messages (angle difference wrapped to [-pi, pi])
  rclcpp::Time stamp(msg->header.stamp);
  if (!first_odom_) {
    double dt = (stamp - last_odom_stamp_).seconds();
    if (dt > 0.0) {
      double dtheta = std::atan2(std::sin(theta - robot_theta_), std::cos(theta - robot_theta_));
      yaw_rate_ = dtheta / dt;
    }
  }
  last_odom_stamp_ = stamp;
  robot_theta_ = theta;

  if (first_odom_) {
    last_update_x_ = robot_x_;
    last_update_y_ = robot_y_;
    first_odom_ = false;
  }
}

void MapMemoryNode::updateMap() {
  if (!costmap_received_ || first_odom_) {
    return;  // need both a costmap and a pose before we can place anything
  }

  double dx = costmap_x_ - last_update_x_;
  double dy = costmap_y_ - last_update_y_;
  double distance = std::sqrt(dx * dx + dy * dy);

  // While turning quickly, even a small timing error puts obstacles in the
  // wrong place, so wait until the turn is over (retried on the next tick)
  bool turning = std::abs(costmap_yaw_rate_) > max_integration_yaw_rate_;

  // Keep merging until the first costmap with obstacles in it arrives, so the
  // starting surroundings are captured even if the lidar starts up late
  if (!turning && (!initial_map_integrated_ || distance >= update_distance_threshold_)) {
    map_memory_.integrateCostmap(latest_costmap_, costmap_x_, costmap_y_, costmap_theta_);
    last_update_x_ = costmap_x_;
    last_update_y_ = costmap_y_;

    bool has_obstacles = std::any_of(latest_costmap_.data.begin(), latest_costmap_.data.end(),
                                     [](int8_t v) { return v > 0; });
    if (has_obstacles) {
      initial_map_integrated_ = true;
    }
  }

  auto map_msg = map_memory_.getGlobalMapMessage("sim_world", this->now());
  map_pub_->publish(map_msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}