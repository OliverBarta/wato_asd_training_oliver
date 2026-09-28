#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_


#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <cmath>
#include <optional>

namespace robot
{

class ControlCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    ControlCore(const rclcpp::Logger& logger);

    // Computes the velocity command to follow the path from the robot's current pose.
    geometry_msgs::msg::Twist computeCommand(const nav_msgs::msg::Path& path,
                                             const nav_msgs::msg::Odometry& odom) const;

  private:
    std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint(const nav_msgs::msg::Path& path, double robot_x, double robot_y) const;

    // Pure Pursuit steering towards the target from the robot's pose.
    geometry_msgs::msg::Twist computeVelocity(const geometry_msgs::msg::PoseStamped& target, double robot_x, double robot_y, double robot_yaw) const;

    static double computeDistance(double x1, double y1, double x2, double y2);

    static double extractYaw(const geometry_msgs::msg::Quaternion& q);

    rclcpp::Logger logger_;

    const double lookahead_distance_ = 1.5;
    const double goal_tolerance_ = 0.5;
    const double linear_speed_ = 1.0;
    const double max_angular_speed_ = 2.0;
    const double turn_in_place_angle_ = M_PI / 3.0;
};

}

#endif
