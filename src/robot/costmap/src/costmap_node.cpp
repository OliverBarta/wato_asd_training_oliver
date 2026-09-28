#include <chrono>
#include <memory>
 
#include "costmap_node.hpp"
 
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // string_pub_ = this->create_publisher<std_msgs::msg::String>("/test_topic", 10);
  // timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CostmapNode::publishMessage, this));

  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10, std::bind(&CostmapNode::lidarCallback, this, std::placeholders::_1));

  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);

  costmap_.initializeCostmap();// needed if not already called elsewhere
}
 
// Define the timer to publish a message every 500ms
void CostmapNode::publishMessage() {
  auto message = std_msgs::msg::String();
  message.data = "Hello, ROS 2!";
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  string_pub_->publish(message);
}

void CostmapNode::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  costmap_.resetCostmap();

  for (size_t i = 0; i < msg->ranges.size(); ++i) {
    double range = msg->ranges[i];

    if (range < msg->range_min || range > msg->range_max) {
      continue;
    }

    double angle = msg->angle_min + i * msg->angle_increment;

    int x_grid, y_grid;
    costmap_.convertToGrid(range, angle, x_grid, y_grid);
    costmap_.markObstacle(x_grid, y_grid);
  }

  costmap_.inflateObstacles();
  publishCostmap();
}
 
void CostmapNode::publishCostmap() {
  auto msg = nav_msgs::msg::OccupancyGrid();

  // header
  msg.header.stamp = this->now();
  msg.header.frame_id = "sim_world";   // match your fixed frame — check what your other topics use

  // info
  msg.info.resolution = costmap_.getResolution();
  msg.info.width = costmap_.getWidth();
  msg.info.height = costmap_.getHeight();
  msg.info.origin.position.x = costmap_.getOriginX();
  msg.info.origin.position.y = costmap_.getOriginY();
  msg.info.origin.position.z = 0.0;
  msg.info.origin.orientation.w = 1.0;   // identity rotation

  // data: flatten 2D grid into 1D row-major array
  const auto& grid = costmap_.getGrid();
  msg.data.resize(costmap_.getWidth() * costmap_.getHeight());

  for (int y = 0; y < costmap_.getHeight(); ++y) {
    for (int x = 0; x < costmap_.getWidth(); ++x) {
      msg.data[y * costmap_.getWidth() + x] = static_cast<int8_t>(grid[y][x]);
    }
  }

  costmap_pub_->publish(msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}