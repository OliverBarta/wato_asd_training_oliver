#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include "planner_core.hpp"

class PlannerNode : public rclcpp::Node {
  public:
    PlannerNode();

  private:
    enum class State { WAITING_FOR_GOAL, WAITING_FOR_ROBOT_TO_REACH_GOAL };

    robot::PlannerCore planner_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Stores the latest map. If currently following a goal, replans, since the
    // new map may have revealed an obstacle on the old path.
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    // Stores the new goal, records when planning started (for the timeout),
    // switches to WAITING_FOR_ROBOT_TO_REACH_GOAL and plans a path.
    void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);

    // Stores the robot's current x/y position.
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // Runs every ~500 ms. In WAITING_FOR_ROBOT_TO_REACH_GOAL, goes back to
    // WAITING_FOR_GOAL if the goal was reached or the timeout passed.
    void timerCallback();

    // True if the robot is within goal_tolerance_ of the goal.
    bool goalReached() const;

    // Checks there's a map and a goal, calls planner_.planPath() with the robot
    // position and goal, stamps the result and publishes it on /path.
    void planPath();

    State state_ = State::WAITING_FOR_GOAL;

    nav_msgs::msg::OccupancyGrid current_map_;
    bool map_received_ = false;

    geometry_msgs::msg::PointStamped goal_;
    bool goal_received_ = false;

    double robot_x_ = 0.0;
    double robot_y_ = 0.0;
    bool odom_received_ = false;

    rclcpp::Time plan_start_time_;

    const double goal_tolerance_ = 0.5;   // metres
    const double timeout_seconds_ = 90.0;
};

#endif
