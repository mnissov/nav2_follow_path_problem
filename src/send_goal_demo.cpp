#include "tractor_navigation/send_goal_demo.hpp"

inline double DEG_2_RAD(const double &deg) { return deg * M_PI / 180.0; }

FollowPathClient::FollowPathClient(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
    : Node("send_goal_demo", options) {
  client_ptr_ = rclcpp_action::create_client<FollowPath>(this, "follow_path");

  timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                   std::bind(&FollowPathClient::send_goal, this));
}

void FollowPathClient::send_goal() {
  timer_->cancel();

  if (!client_ptr_->wait_for_action_server()) {
    RCLCPP_ERROR(this->get_logger(), "Action server not available after watiting.");
    rclcpp::shutdown();
  }

  auto goal_msg = FollowPath::Goal();
  goal_msg.controller_id = "FollowPath";
  goal_msg.path.header.frame_id = "map";

  geometry_msgs::msg::PoseStamped pose;
  tf2::Quaternion quat;
  pose.header.frame_id = goal_msg.path.header.frame_id;
  pose.pose.position.z = 0.0;
  pose.pose.orientation.x = pose.pose.orientation.y = pose.pose.orientation.z = 0.0;
  pose.pose.orientation.w = 1.0;

  // ADD MORE POSES
  pose.pose.position.x = 10.0;
  pose.pose.position.y = 0.0;
  quat.setRPY(0, 0, 0);
  pose.pose.orientation = tf2::toMsg(quat);
  goal_msg.path.poses.push_back(pose);

  // pose.pose.position.x = 10.0 + 5.0 * std::sin(DEG_2_RAD(45.0));
  // pose.pose.position.y = 5.0 + 5.0 * std::cos(DEG_2_RAD(45.0));
  // quat.setRPY(0, 0, DEG_2_RAD(45.0));
  // pose.pose.orientation = tf2::toMsg(quat);
  // goal_msg.path.poses.push_back(pose);

  // pose.pose.position.x = 10.0 + 5.0;
  // pose.pose.position.y = 5.0;
  // quat.setRPY(0, 0, DEG_2_RAD(90.0));
  // pose.pose.orientation = tf2::toMsg(quat);
  // goal_msg.path.poses.push_back(pose);

  // pose.pose.position.x = 10.0 + 5.0 * std::sin(DEG_2_RAD(45.0));
  // pose.pose.position.y = 5.0 + 5.0 * std::cos(DEG_2_RAD(45.0));
  // quat.setRPY(0, 0, DEG_2_RAD(135.0));
  // pose.pose.orientation = tf2::toMsg(quat);
  // goal_msg.path.poses.push_back(pose);

  // pose.pose.position.x = 10.0;
  // pose.pose.position.y = 10.0;
  // quat.setRPY(0, 0, DEG_2_RAD(180.0));
  // pose.pose.orientation = tf2::toMsg(quat);
  // goal_msg.path.poses.push_back(pose);

  RCLCPP_INFO(this->get_logger(), "Sending goal, compose of %d poses", goal_msg.path.poses.size());

  auto send_goal_options = rclcpp_action::Client<FollowPath>::SendGoalOptions();
  send_goal_options.goal_response_callback =
      std::bind(&FollowPathClient::goal_response_callback, this, std::placeholders::_1);
  send_goal_options.feedback_callback = std::bind(&FollowPathClient::feedback_callback, this,
                                                  std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback =
      std::bind(&FollowPathClient::result_callback, this, std::placeholders::_1);
  client_ptr_->async_send_goal(goal_msg, send_goal_options);
}

void FollowPathClient::goal_response_callback(
    std::shared_future<GoalHandleFollowPath::SharedPtr> future) {
  auto goal_handle = future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  } else {
    RCLCPP_INFO(this->get_logger(), "Goal was accepted by server, waiting for result.");
  }
}

void FollowPathClient::feedback_callback(
    GoalHandleFollowPath::SharedPtr, const std::shared_ptr<const FollowPath::Feedback> feedback) {
  std::cout << "[Feedback]" << std::endl;

  std::cout << "\tDistance: " << feedback->distance_to_goal << "\tSpeed: " << feedback->speed
            << std::endl;
}

void FollowPathClient::result_callback(const GoalHandleFollowPath::WrappedResult &result) {
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Uknown result code.");
      return;
  }
  rclcpp::shutdown();
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto action_client = std::make_shared<FollowPathClient>();
  rclcpp::spin(action_client);

  rclcpp::shutdown();
}