#include <functional>
#include <memory>
#include <thread>
#include <future>
#include <string>
#include <sstream>
#include "action_turtle_commands1/action/message_turtle_commands.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/string.hpp"

namespace action_turtle_client
{
class ActionTurtleClient : public rclcpp::Node
{
public:
  using MessageTurtleCommands = action_turtle_commands1::action::MessageTurtleCommands;
  using GoalHandleTurtle = rclcpp_action::ClientGoalHandle<MessageTurtleCommands>;

  explicit ActionTurtleClient(const rclcpp::NodeOptions & options)
  : Node("action_turtle_client", options)
  {
    this->client_ptr_ = rclcpp_action::create_client<MessageTurtleCommands>(
      this,
      "message_turtle_commands");

    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&ActionTurtleClient::send_goal, this));
  }

  void send_goal()
  {
    using namespace std::placeholders;

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
    this->client_ptr_->async_send_goal(goal_msg1, send_goal_options);
    this->client_ptr_->async_send_goal(goal_msg2, send_goal_options);
    this->client_ptr_->async_send_goal(goal_msg3, send_goal_options);
  }

private:
  rclcpp_action::Client<MessageTurtleCommands>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;

  void goal_response_callback(const GoalHandleTurtle::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
    GoalHandleTurtle::SharedPtr,
    const std::shared_ptr<const MessageTurtleCommands::Feedback> feedback)
  {
    std::stringstream ss;
    // ss << "Next number in sequence received: ";
    // for (auto number : feedback->odom) {
    //   ss << number << " ";
    // }
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }

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
    // ss << "Result received: ";
    // for (auto number : result.result->result) {
    //   ss << number << " ";
    // }
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    rclcpp::shutdown();
  }
};  // class ActionTurtleClient

}  // namespace action_tutorials_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(action_turtle_client::ActionTurtleClient)
// class ActionTurtleClient : public rclcpp::Node
// {
// public:
//   ActionTurtleClient()
//     : Node("action_turtle_client"), distance_traveled_(0.0)
//   {
//     action_client_ = rclcpp_action::create_client<action_turtle_commands1::action::MessageTurtleCommands>(
//       this,
//       "message_turtle_commands"
//     );

//     // Wait for the action server to become available
//     if (!action_client_->wait_for_action_server(std::chrono::seconds(10)))
//     {
//       RCLCPP_ERROR(this->get_logger(), "Action server not available.");
//       rclcpp::shutdown();
//     }

//     sendGoal();
//   }

// private:
//   void sendGoal()
//   {
//     auto goal = action_turtle_commands1::action::MessageTurtleCommands::Goal();
//     auto feedback = std::make_shared<action_turtle_commands1::action::MessageTurtleCommands::Feedback>();

//     // Forward 2 meters
//     goal.command = "forward";
//     goal.s = 2;
//     goal.angle = 0;

//     auto send_goal_options = rclcpp_action::Client<action_turtle_commands1::action::MessageTurtleCommands>::SendGoalOptions();
//     &send_goal_options.feedback_callback =
//       &[this](auto, std::shared_ptr<action_turtle_commands1::action::MessageTurtleCommands_Feedback> feedback_msg) {
//         feedbackReceived(feedback_msg);
//       };

//     auto future_goal_handle = action_client_->async_send_goal(goal, send_goal_options);
//     rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_goal_handle);

//     if (future_goal_handle.get()->result->result)
//     {
//       RCLCPP_INFO(this->get_logger(), "Forward goal succeeded.");
//       rclcpp::sleep_for(std::chrono::seconds(1));
//     }
//     else
//     {
//       RCLCPP_ERROR(this->get_logger(), "Forward goal failed.");
//       rclcpp::shutdown();
//       return;
//     }

//     // Turn right 90 degrees
//     goal.command = "turn_right";
//     goal.s = 0;
//     goal.angle = 90;

//     future_goal_handle = action_client_->async_send_goal(goal, send_goal_options);
//     rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_goal_handle);

//     if (future_goal_handle.get()->result->result)
//     {
//       RCLCPP_INFO(this->get_logger(), "Turn right goal succeeded.");
//       rclcpp::sleep_for(std::chrono::seconds(1));
//     }
//     else
//     {
//       RCLCPP_ERROR(this->get_logger(), "Turn right goal failed.");
//       rclcpp::shutdown();
//       return;
//     }

//     // Forward 1 meter
//     goal.command = "forward";
//     goal.s = 1;
//     goal.angle = 0;

//     future_goal_handle = action_client_->async_send_goal(goal, send_goal_options);
//     rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_goal_handle);

//     if (future_goal_handle.get()->result->result)
//     {
//       RCLCPP_INFO(this->get_logger(), "Forward goal succeeded.");
//       rclcpp::shutdown();
//     }
//     else
//     {
//       RCLCPP_ERROR(this->get_logger(), "Forward goal failed.");
//       rclcpp::shutdown();
//       return;
//     }
//   }

//   void feedbackReceived(const action_turtle_commands1::action::MessageTurtleCommands::Feedback::SharedPtr feedback_msg)
//   {
//     distance_traveled_ = feedback_msg->odom;
//     RCLCPP_INFO(this->get_logger(), "Distance Traveled: %.2f meters", distance_traveled_);
//   }

//   rclcpp_action::Client<action_turtle_commands1::action::MessageTurtleCommands>::SharedPtr action_client_;
//   double distance_traveled_;
// };

// int main(int argc, char * argv[])
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<ActionTurtleClient>());
//   rclcpp::shutdown();
//   return 0;
// }

