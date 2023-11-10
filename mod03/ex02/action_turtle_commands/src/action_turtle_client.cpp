#include <memory>
#include <thread>
#include <future>
#include <string>
#include <sstream>
#include <iostream>
#include <unistd.h>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "action_turtle_commands/visibility_control.h"
#include "action_turtle_commands1/action/message_turtle_commands.hpp"

namespace action_turtle_commands
{
class ActionTurtleClient : public rclcpp::Node
{
public:
  using MessageTurtleCommands = action_turtle_commands1::action::MessageTurtleCommands;
  using GoalHandleTurtle = rclcpp_action::ClientGoalHandle<MessageTurtleCommands>;

  ACTION_TURTLE_COMMANDS_PUBLIC
  explicit ActionTurtleClient(const rclcpp::NodeOptions & options)
  : Node("action_turtle_client", options)
  {
    std::cout << "Initialization...\n" << std::endl;
    this->client_ptr_ = rclcpp_action::create_client<MessageTurtleCommands>(
      this->get_node_base_interface(),
      this->get_node_graph_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "message_turtle_commands");

    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&ActionTurtleClient::send_goal, this));
  }

  ACTION_TURTLE_COMMANDS_PUBLIC
  void send_goal()
  {
    using namespace std::placeholders;
    std::cout << "In send_goal()\n";

    this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = MessageTurtleCommands::Goal();
    goal_msg.command = "forward";
    goal_msg.s = 1;
    auto goal_msg1 = MessageTurtleCommands::Goal();
    goal_msg1.command = "forward";
    goal_msg1.s = 1;
    auto goal_msg2 = MessageTurtleCommands::Goal();
    goal_msg2.command = "turn_right";
    goal_msg2.angle = 90;
    auto goal_msg3 = MessageTurtleCommands::Goal();
    goal_msg3.command = "forward";
    goal_msg3.s = 1;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<MessageTurtleCommands>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&ActionTurtleClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&ActionTurtleClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&ActionTurtleClient::result_callback, this, _1);
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    std::cout << "Send one\n";
    // sleep(5);
    this->client_ptr_->async_send_goal(goal_msg1, send_goal_options);
    std::cout << "Send two\n";
    // sleep(5);
    this->client_ptr_->async_send_goal(goal_msg2, send_goal_options);
    std::cout << "Send three\n";
    // sleep(5);
    this->client_ptr_->async_send_goal(goal_msg3, send_goal_options);
    std::cout << "Send four\n";
  }

private:
  rclcpp_action::Client<MessageTurtleCommands>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;

  ACTION_TURTLE_COMMANDS_LOCAL
  void goal_response_callback(const GoalHandleTurtle::SharedPtr & goal_handle)
  {
    // std::cout << "In goal_response_callback()\n";
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  ACTION_TURTLE_COMMANDS_LOCAL
  void feedback_callback(
    GoalHandleTurtle::SharedPtr,
    const std::shared_ptr<const MessageTurtleCommands::Feedback> feedback)
  {
    std::stringstream ss;
    ss << "Feedback: " << feedback->odom;
    // for (auto number : feedback->odom) {
    //   ss << number << " ";
    // }
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }

  ACTION_TURTLE_COMMANDS_LOCAL
  void result_callback(const GoalHandleTurtle::WrappedResult & result)
  {
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
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
    std::stringstream ss;
    ss << "Result received: ";
    // for (auto number : result.result->result) {
    //   ss << number << " ";
    // }
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    std::cout << result.result << "\n";
    rclcpp::shutdown();
  }
};  // class ActionTurtleClient

}  // namespace action_tutorials_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(action_turtle_commands::ActionTurtleClient)
