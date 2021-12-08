#ifndef TRACTOR_SIM__SEND_GOAL_DEMO_HPP_
#define TRACTOR_SIM__SEND_GOAL_DEMO_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include "nav2_msgs/action/follow_path.hpp"

#include <chrono>
#include <iostream>
#include <vector>

class FollowPathClient : public rclcpp::Node {
 public:
  using FollowPath = nav2_msgs::action::FollowPath;
  using GoalHandleFollowPath = rclcpp_action::ClientGoalHandle<FollowPath>;

  explicit FollowPathClient(const rclcpp::NodeOptions &options);

  void send_goal();

 private:
  void goal_response_callback(std::shared_future<GoalHandleFollowPath::SharedPtr> future);
  void feedback_callback(GoalHandleFollowPath::SharedPtr,
                         const std::shared_ptr<const FollowPath::Feedback> feedback);
  void result_callback(const GoalHandleFollowPath::WrappedResult &result);

  rclcpp_action::Client<FollowPath>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]);

#endif  // TRACTOR_SIM__SEND_GOAL_DEMO_HPP_