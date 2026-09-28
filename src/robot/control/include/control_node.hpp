#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

// ControlNode: the ROS wrapper around ControlCore.
//
// Subscribes:
//   /path           nav_msgs::msg::Path        planned path from the planner
//   /odom/filtered  nav_msgs::msg::Odometry    robot's current pose
// Publishes:
//   /cmd_vel        geometry_msgs::msg::Twist  velocity command for the robot
// Timer:
//   Every 100 ms (10 Hz), computes and publishes a new command.

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "control_core.hpp"

class ControlNode : public rclcpp::Node {
  public:
    ControlNode();

  private:
    robot::ControlCore control_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // Publishes control_.computeCommand() once both a path and odometry have arrived.
    void timerCallback();

    nav_msgs::msg::Path path_;
    bool path_received_ = false;

    nav_msgs::msg::Odometry odom_;
    bool odom_received_ = false;
};

#endif
