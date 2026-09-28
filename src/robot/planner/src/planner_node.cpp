#include "planner_node.hpp"

#include <cmath>

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));

  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  current_map_ = *msg;
  map_received_ = true;

  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    planPath();
  }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  goal_ = *msg;
  goal_received_ = true;
  plan_start_time_ = this->now();
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;

  RCLCPP_INFO(this->get_logger(), "New goal received: (%.2f, %.2f)", goal_.point.x, goal_.point.y);
  planPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;
  odom_received_ = true;
}

void PlannerNode::timerCallback() {
  if (state_ != State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    return;
  }

  if (goalReached()) {
    RCLCPP_INFO(this->get_logger(), "Goal reached, waiting for a new goal.");
    state_ = State::WAITING_FOR_GOAL;
    goal_received_ = false;
    return;
  }

  double elapsed = (this->now() - plan_start_time_).seconds();
  if (elapsed > timeout_seconds_) {
    RCLCPP_WARN(this->get_logger(), "Timed out after %.0f s trying to reach goal, giving up.", elapsed);
    state_ = State::WAITING_FOR_GOAL;
    goal_received_ = false;

    nav_msgs::msg::Path empty_path;
    empty_path.header.stamp = this->now();
    empty_path.header.frame_id = current_map_.header.frame_id;
    path_pub_->publish(empty_path);
  }
}

bool PlannerNode::goalReached() const {
  double dx = goal_.point.x - robot_x_;
  double dy = goal_.point.y - robot_y_;
  return std::sqrt(dx * dx + dy * dy) < goal_tolerance_;
}

void PlannerNode::planPath() {
  if (!map_received_ || !goal_received_ || !odom_received_) {
    RCLCPP_WARN(this->get_logger(), "Can't plan yet: missing map, goal or odometry.");
    return;
  }

  nav_msgs::msg::Path path = planner_.planPath(
    current_map_, robot_x_, robot_y_, goal_.point.x, goal_.point.y, current_map_.header.frame_id);

  path.header.stamp = this->now();
  for (auto& pose : path.poses) {
    pose.header.stamp = path.header.stamp;
  }

  path_pub_->publish(path);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
