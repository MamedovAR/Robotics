#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "action_turtle_commands/action/message_turtle_commands.hpp"
#include "std_msgs/msg/string.hpp"

class ActionTurtleClient : public rclcpp::Node
{
public:
  ActionTurtleClient()
    : Node("action_turtle_client"), distance_traveled_(0.0)
  {
    action_client_ = rclcpp_action::create_client<action_turtle_commands::action::MessageTurtleCommands>(
      this,
      "message_turtle_commands"
    );

    // Wait for the action server to become available
    if (!action_client_->wait_for_action_server(std::chrono::seconds(10)))
    {
      RCLCPP_ERROR(this->get_logger(), "Action server not available.");
      rclcpp::shutdown();
    }

    sendGoal();
  }

private:
  void sendGoal()
  {
    auto goal = action_turtle_commands::action::MessageTurtleCommands::Goal();
    auto feedback = std::make_shared<action_turtle_commands::action::MessageTurtleCommands::Feedback>();

    // Forward 2 meters
    goal.command = "forward";
    goal.s = 2;
    goal.angle = 0;

    auto send_goal_options = rclcpp_action::Client<action_turtle_commands::action::MessageTurtleCommands>::SendGoalOptions();
    send_goal_options.feedback_callback =
      [this](auto, auto feedback_msg) {
        feedbackReceived(feedback_msg);
      };

    auto future_goal_handle = action_client_->async_send_goal(goal, send_goal_options);
    rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_goal_handle);

    if (future_goal_handle.get()->result->result)
    {
      RCLCPP_INFO(this->get_logger(), "Forward goal succeeded.");
      rclcpp::sleep_for(std::chrono::seconds(1));
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Forward goal failed.");
      rclcpp::shutdown();
      return;
    }

    // Turn right 90 degrees
    goal.command = "turn_right";
    goal.s = 0;
    goal.angle = 90;

    future_goal_handle = action_client_->async_send_goal(goal, send_goal_options);
    rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_goal_handle);

    if (future_goal_handle.get()->result->result)
    {
      RCLCPP_INFO(this->get_logger(), "Turn right goal succeeded.");
      rclcpp::sleep_for(std::chrono::seconds(1));
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Turn right goal failed.");
      rclcpp::shutdown();
      return;
    }

    // Forward 1 meter
    goal.command = "forward";
    goal.s = 1;
    goal.angle = 0;

    future_goal_handle = action_client_->async_send_goal(goal, send_goal_options);
    rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_goal_handle);

    if (future_goal_handle.get()->result->result)
    {
      RCLCPP_INFO(this->get_logger(), "Forward goal succeeded.");
      rclcpp::shutdown();
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Forward goal failed.");
      rclcpp::shutdown();
      return;
    }
  }

  void feedbackReceived(const action_turtle_commands::action::MessageTurtleCommands::Feedback::SharedPtr feedback_msg)
  {
    distance_traveled_ = feedback_msg->odom;
    RCLCPP_INFO(this->get_logger(), "Distance Traveled: %.2f meters", distance_traveled_);
  }

  rclcpp_action::Client<action_turtle_commands::action::MessageTurtleCommands>::SharedPtr action_client_;
  double distance_traveled_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ActionTurtleClient>());
  rclcpp::shutdown();
  return 0;
}

