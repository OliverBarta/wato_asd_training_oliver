#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

// ControlCore: the Pure Pursuit logic, with no ROS pub/sub in here.
//
// Given the planned path and the robot's odometry, it picks a lookahead point
// on the path and returns the velocity command (Twist) that steers towards it.
// Returns a zero Twist when there is no path or the goal has been reached.

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
    // First pose on the path at least lookahead_distance_ from the robot,
    // or the last pose if none is that far. nullopt if the path is empty.
    std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint(
      const nav_msgs::msg::Path& path, double robot_x, double robot_y) const;

    // Pure Pursuit steering towards the target from the robot's pose.
    geometry_msgs::msg::Twist computeVelocity(const geometry_msgs::msg::PoseStamped& target,
                                              double robot_x, double robot_y, double robot_yaw) const;

    static double computeDistance(double x1, double y1, double x2, double y2);

    // Heading (rotation about z) from a quaternion.
    static double extractYaw(const geometry_msgs::msg::Quaternion& q);

    rclcpp::Logger logger_;

    const double lookahead_distance_ = 1.0;  // metres
    const double goal_tolerance_ = 0.3;      // metres, within the planner's 0.5 m
    const double linear_speed_ = 0.5;        // m/s
    const double max_angular_speed_ = 1.0;   // rad/s
    const double turn_in_place_angle_ = M_PI / 3.0;  // turn without driving if heading error is bigger than this
};

}

#endif
