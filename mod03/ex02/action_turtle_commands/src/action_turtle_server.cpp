#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include "action_turtle_commands/action/message_turtle_commands.hpp"

using namespace std::chrono_literals;

class ActionTurtleServer : public rclcpp::Node
{
public:
  ActionTurtleServer()
    : Node("action_turtle_server"), odom_(0)
  {
    using namespace std::placeholders;

    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    pose_subscription_ = this->create_subscription<turtlesim::msg::Pose>(
      "/turtle1/pose", 10, std::bind(&ActionTurtleServer::poseCallback, this, _1)
    );

    action_server_ = rclcpp_action::create_server<action_turtle_commands::action::MessageTurtleCommands>(
      this,
      "message_turtle_commands",
      std::bind(&ActionTurtleServer::handleGoal, this, _1, _2),
      std::bind(&ActionTurtleServer::handleCancel, this, _1),
      std::bind(&ActionTurtleServer::handleAccepted, this, _1)
    );
  }

private:
  void poseCallback(const turtlesim::msg::Pose::SharedPtr msg)
  {
    odom_ = msg->x;
  }

  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const action_turtle_commands::action::MessageTurtleCommands::Goal> goal)
  {
    (void)uuid; // Unused
    RCLCPP_INFO(this->get_logger(), "Received goal: %s", goal->command.c_str());

    // Process goal and control turtle here
    if (goal->command == "forward")
    {
      controlTurtle(goal->s, 0);
    }
    else if (goal->command == "turn_left")
    {
      controlTurtle(0, goal->angle);
    }
    else if (goal->command == "turn_right")
    {
      controlTurtle(0, -goal->angle);
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  void handleAccepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<action_turtle_commands::action::MessageTurtleCommands>> goal_handle)
  {
    std::thread{std::bind(&ActionTurtleServer::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<action_turtle_commands::action::MessageTurtleCommands>> goal_handle)
  {
    auto feedback = std::make_shared<action_turtle_commands::action::MessageTurtleCommands::Feedback>();
    auto & goal = goal_handle->get_goal();

    while (rclcpp::ok())
    {
      // Calculate current distance traveled
      feedback->odom = odom_ - initial_odom_;

      // Publish feedback
      goal_handle->publish_feedback(feedback);

      // Check if the goal has been canceled
      if (goal_handle->is_canceling())
      {
        goal_handle->canceled();
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      // Check if the goal has been achieved
      if (feedback->odom >= goal->s)
      {
        controlTurtle(0, 0); // Stop the turtle
        auto result = std::make_shared<action_turtle_commands::action::MessageTurtleCommands::Result>();
        result->result = true; // Goal achieved
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        return;
      }

      rclcpp::sleep_for(100ms); // Sleep for 100 milliseconds
    }
  }

  void handleCancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<action_turtle_commands::action::MessageTurtleCommands>> goal_handle)
  {
    (void)goal_handle; // Unused
    RCLCPP_INFO(this->get_logger(), "Goal canceled");
  }

  void controlTurtle(double linear, double angular)
  {
    geometry_msgs::msg::Twist twist;
    twist.linear.x = linear;
    twist.angular.z = angular;
    cmd_vel_publisher_->publish(twist);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscription_;
  rclcpp_action::Server<action_turtle_commands::action::MessageTurtleCommands>::SharedPtr action_server_;
  double odom_;
  double initial_odom_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ActionTurtleServer>());
  rclcpp::shutdown();
  return 0;
}

