#include "control_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger)
  : logger_(logger) {}

geometry_msgs::msg::Twist ControlCore::computeCommand(const nav_msgs::msg::Path& path,
                                                      const nav_msgs::msg::Odometry& odom) const {
  geometry_msgs::msg::Twist stop;// all zeros

  // Empty path
  if (path.poses.empty()) {
    return stop;
  }

  double robot_x = odom.pose.pose.position.x;
  double robot_y = odom.pose.pose.position.y;
  double robot_yaw = extractYaw(odom.pose.pose.orientation);

  const auto& goal = path.poses.back().pose.position;
  if (computeDistance(robot_x, robot_y, goal.x, goal.y) < goal_tolerance_) {
    return stop;
  }

  auto target = findLookaheadPoint(path, robot_x, robot_y);
  if (!target) {
    return stop;
  }

  return computeVelocity(*target, robot_x, robot_y, robot_yaw);
}

std::optional<geometry_msgs::msg::PoseStamped> ControlCore::findLookaheadPoint(
  const nav_msgs::msg::Path& path, double robot_x, double robot_y) const {
  if (path.poses.empty()) {
    return std::nullopt;
  }

  // start from the pose closest to the robot so we never pick a point behind it
  std::size_t closest = 0;
  double closest_dist = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i < path.poses.size(); ++i) {
    const auto& p = path.poses[i].pose.position;
    double d = computeDistance(robot_x, robot_y, p.x, p.y);
    if (d < closest_dist) {
      closest_dist = d;
      closest = i;
    }
  }

  for (std::size_t i = closest; i < path.poses.size(); ++i) {
    const auto& p = path.poses[i].pose.position;
    if (computeDistance(robot_x, robot_y, p.x, p.y) >= lookahead_distance_) {
      return path.poses[i];
    }
  }

  // Near the end of the path, aim for the goal itself
  return path.poses.back();
}

geometry_msgs::msg::Twist ControlCore::computeVelocity(const geometry_msgs::msg::PoseStamped& target,
                                                       double robot_x, double robot_y, double robot_yaw) const {
  geometry_msgs::msg::Twist cmd;

  double dx = target.pose.position.x - robot_x;
  double dy = target.pose.position.y - robot_y;
  double angle_to_target = std::atan2(dy, dx);

  // Heading error wrapped into [-pi, pi] so the robot turns the short way
  double error = angle_to_target - robot_yaw;
  error = std::atan2(std::sin(error), std::cos(error));

  // Pure Pursuit curvature: 2 * sin(alpha) / L, angular = v * curvature
  double angular = 2.0 * linear_speed_ * std::sin(error) / lookahead_distance_;

  if (std::abs(error) > turn_in_place_angle_) {
    // Target is well off to the side or behind: turn on the spot first
    cmd.linear.x = 0.0;
    angular = std::copysign(max_angular_speed_, error);
  } else {
    cmd.linear.x = linear_speed_;
  }

  cmd.angular.z = std::clamp(angular, -max_angular_speed_, max_angular_speed_);
  return cmd;
}

double ControlCore::computeDistance(double x1, double y1, double x2, double y2) {
  return std::hypot(x2 - x1, y2 - y1);
}

double ControlCore::extractYaw(const geometry_msgs::msg::Quaternion& q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

}
