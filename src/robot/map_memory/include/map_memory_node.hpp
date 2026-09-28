#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();

  private:
    robot::MapMemoryCore map_memory_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void updateMap();

    nav_msgs::msg::OccupancyGrid latest_costmap_;
    bool costmap_received_ = false;

    double robot_x_ = 0.0;
    double robot_y_ = 0.0;
    double robot_theta_ = 0.0;
    double yaw_rate_ = 0.0;
    rclcpp::Time last_odom_stamp_;

    double costmap_x_ = 0.0;
    double costmap_y_ = 0.0;
    double costmap_theta_ = 0.0;
    double costmap_yaw_rate_ = 0.0;

    double last_update_x_ = 0.0;
    double last_update_y_ = 0.0;
    bool first_odom_ = true;
    bool initial_map_integrated_ = false;

    const double update_distance_threshold_ = 1.5;
    const double max_integration_yaw_rate_ = 0.3;
};

#endif