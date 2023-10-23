#include <functional>
#include <memory>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "action_turtle_commands1/action/message_turtle_commands.hpp"
#include "turtlesim/msg/pose.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace action_turtele_server
{
class ActionTurtleServer : public rclcpp::Node
{
public:
  using MessageTurtleCommands = action_turtle_commands1::action::MessageTurtleCommands;
  using GoalHandleTurtle = rclcpp_action::ServerGoalHandle<MessageTurtleCommands>;

  explicit ActionTurtleServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("action_turtle_server", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<MessageTurtleCommands>(
      this,
      "message_turtle_commands",
      std::bind(&ActionTurtleServer::handle_goal, this, _1, _2),
      std::bind(&ActionTurtleServer::handle_cancel, this, _1),
      std::bind(&ActionTurtleServer::handle_accepted, this, _1));
  }

private:
  rclcpp_action::Server<MessageTurtleCommands>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MessageTurtleCommands::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with order %s", goal->command);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleTurtle> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleTurtle> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&ActionTurtleServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleTurtle> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<MessageTurtleCommands::Feedback>();
    auto & sequence = feedback->odom;
    sequence+=(0);
    sequence+=(1);
    auto result = std::make_shared<MessageTurtleCommands::Result>();

    geometry_msgs::msg::Twist twist;
    if(goal->command=="turn_right")
              twist.angular.z = -1.5;
    else if(goal->command=="turn_left")
              twist.angular.z = 1.5;
    else if(goal->command=="move_forward")
              twist.linear.x = 1;
    else if(goal->command=="move_backward")
              twist.linear.x = -1;
    this->publisher_->publish(twist);

    for (int i = 1; (i < 2) && rclcpp::ok(); ++i) {
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
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback");

      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->result = sequence;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

};  // class FibonacciActionServer

}  // namespace action_tutorials_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(action_turtele_server::ActionTurtleServer)
// using namespace std::chrono_literals;

// class ActionTurtleServer : public rclcpp::Node
// {
// public:
//   ActionTurtleServer()
//     : Node("action_turtle_server"), odom_(0)
//   {
//     using namespace std::placeholders;

//     cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
//     pose_subscription_ = this->create_subscription<turtlesim::msg::Pose>(
//       "/turtle1/pose", 10, std::bind(&ActionTurtleServer::poseCallback, this, _1)
//     );

//     action_server_ = rclcpp_action::create_server<action_turtle_commands1::action::MessageTurtleCommands>(
//       this,
//       "message_turtle_commands",
//       std::bind(&ActionTurtleServer::handleGoal, this, _1, _2),
//       std::bind(&ActionTurtleServer::handleCancel, this, _1),
//       std::bind(&ActionTurtleServer::handleAccepted, this, _1)
//     );
//   }

// private:
//   void poseCallback(const turtlesim::msg::Pose::SharedPtr msg)
//   {
//     odom_ = msg->x;
//   }

//   rclcpp_action::GoalResponse handleGoal(
//     const rclcpp_action::GoalUUID & uuid,
//     std::shared_ptr<const action_turtle_commands1::action::MessageTurtleCommands::Goal> goal)
//   {
//     (void)uuid; // Unused
//     RCLCPP_INFO(this->get_logger(), "Received goal: %s", goal->command.c_str());

//     // Process goal and control turtle here
//     if (goal->command == "forward")
//     {
//       controlTurtle(goal->s, 0);
//     }
//     else if (goal->command == "turn_left")
//     {
//       controlTurtle(0, goal->angle);
//     }
//     else if (goal->command == "turn_right")
//     {
//       controlTurtle(0, -goal->angle);
//     }

//     return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
//   }

//   void handleAccepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<action_turtle_commands1::action::MessageTurtleCommands>> goal_handle)
//   {
//     std::thread{std::bind(&ActionTurtleServer::execute, this, std::placeholders::_1), goal_handle}.detach();
//   }

//   void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<action_turtle_commands1::action::MessageTurtleCommands>> goal_handle)
//   {
//     auto feedback = std::make_shared<action_turtle_commands1::action::MessageTurtleCommands::Feedback>();
//     auto & goal = goal_handle->get_goal();

//     while (rclcpp::ok())
//     {
//       // Calculate current distance traveled
//       feedback->odom = odom_ - initial_odom_;

//       // Publish feedback
//       goal_handle->publish_feedback(feedback);

//       // Check if the goal has been canceled
//       if (goal_handle->is_canceling())
//       {
//         // goal_handle->canceled();
//         RCLCPP_INFO(this->get_logger(), "Goal canceled");
//         return;
//       }

//       // Check if the goal has been achieved
//       if (feedback->odom >= goal->s)
//       {
//         controlTurtle(0, 0); // Stop the turtle
//         auto result = std::make_shared<action_turtle_commands1::action::MessageTurtleCommands::Result>();
//         result->result = true; // Goal achieved
//         goal_handle->succeed(result);
//         RCLCPP_INFO(this->get_logger(), "Goal succeeded");
//         return;
//       }

//       rclcpp::sleep_for(100ms); // Sleep for 100 milliseconds
//     }
//   }

//   void handleCancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<action_turtle_commands1::action::MessageTurtleCommands>> goal_handle)
//   {
//     (void)goal_handle; // Unused
//     RCLCPP_INFO(this->get_logger(), "Goal canceled");
//   }

//   void controlTurtle(double linear, double angular)
//   {
//     geometry_msgs::msg::Twist twist;
//     twist.linear.x = linear;
//     twist.angular.z = angular;
//     cmd_vel_publisher_->publish(twist);
//   }

//   rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
//   rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscription_;
//   rclcpp_action::Server<action_turtle_commands1::action::MessageTurtleCommands>::SharedPtr action_server_;
//   double odom_;
//   double initial_odom_;
// };

// int main(int argc, char * argv[])
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<ActionTurtleServer>());
//   rclcpp::shutdown();
//   return 0;
// }

