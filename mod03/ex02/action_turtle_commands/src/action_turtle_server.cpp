#include <cmath>
#include <memory>
#include <thread>
#include <iostream>
#include <unistd.h>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "action_turtle_commands/visibility_control.h"
#include "action_turtle_commands1/action/message_turtle_commands.hpp"

namespace action_turtle_commands
{
class ActionTurtleServer : public rclcpp::Node
{
public:
  using MessageTurtleCommands = action_turtle_commands1::action::MessageTurtleCommands;
  using GoalHandleTurtle = rclcpp_action::ServerGoalHandle<MessageTurtleCommands>;

  ACTION_TURTLE_COMMANDS_PUBLIC
  explicit ActionTurtleServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("action_turtle_server", options)
  {
    using namespace std::placeholders;
    std::cout << "Initialization...\n" << std::endl;
    this->action_server_ = rclcpp_action::create_server<MessageTurtleCommands>(
      this->get_node_base_interface(),
      this->get_node_clock_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "message_turtle_commands",
      std::bind(&ActionTurtleServer::handle_goal, this, _1, _2),
      std::bind(&ActionTurtleServer::handle_cancel, this, _1),
      std::bind(&ActionTurtleServer::handle_accepted, this, _1));
      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
      start_x = 5.5;
      start_y = 5.5;
      // subscription_ = this->create_subscription<turtlesim::msg::Pose>(
      // "cmd_text", 10,  std::bind(&ActionTurtleServer::publish_callback,this,_1));
  }

private:
  rclcpp_action::Server<MessageTurtleCommands>::SharedPtr action_server_;

  ACTION_TURTLE_COMMANDS_LOCAL
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MessageTurtleCommands::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with order %s", goal->command);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  ACTION_TURTLE_COMMANDS_LOCAL
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleTurtle> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  ACTION_TURTLE_COMMANDS_LOCAL
  void handle_accepted(const std::shared_ptr<GoalHandleTurtle> goal_handle)
  {
    using namespace std::placeholders;
    // // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    // std::thread{std::bind(&ActionTurtleServer::execute, this, _1), goal_handle}.detach();
    execute(goal_handle);
  }

  ACTION_TURTLE_COMMANDS_LOCAL
  void publish_callback(const turtlesim::msg::Pose & msg)
  {
    this->pose = msg;
  }

  ACTION_TURTLE_COMMANDS_LOCAL
  void execute(const std::shared_ptr<GoalHandleTurtle> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<MessageTurtleCommands::Feedback>();
    auto & sequence = feedback->odom;
    auto result = std::make_shared<MessageTurtleCommands::Result>();

    geometry_msgs::msg::Twist twist;
    this->pose.x;
    this->pose.y;
    this->pose.theta;
    std::cout << goal->command;
    if(goal->command=="turn_right")
              twist.angular.z = -1.57;
    else if(goal->command=="turn_left")
              twist.angular.z = 1.57;
    else if(goal->command=="forward")
              twist.linear.x = 1;
    else if(goal->command=="backward")
              twist.linear.x = -1;
    std::cout << "Before publish\n";
    this->publisher_->publish(twist);
    std::cout << "In execute(), after publish\n" << std::endl;

    for (int i = 1; (i < 2) && rclcpp::ok(); ++i)
    {
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->result = true;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }
      // Update sequence
      // sequence.push_back(sequence[i] + sequence[i - 1]);
      // Publish feedback
      int ans=0;
      if(goal->command=="forward")
      {
        if(this->flag==0)this->odomX++;
        else this->odomY++;
      }
      else this->flag++;
      ans = int(std::sqrt((0 - this->odomX)*(0 - this->odomX)+(0 - this->odomY)*(0 - this->odomY)));
      feedback->odom +=ans;
      sleep(1);
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback: %d", feedback->odom);

      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->result = sequence;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }
  // rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  turtlesim::msg::Pose pose;
  float start_x, start_y;
  int odomX, odomY, flag;

};  // class FibonacciActionServer

}  // namespace action_tutorials_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(action_turtle_commands::ActionTurtleServer)
