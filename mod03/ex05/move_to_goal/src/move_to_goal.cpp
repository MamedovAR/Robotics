#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/Twist.hpp"

class MoveToGoalNode : public rclcpp::Node
{
public:
  MoveToGoalNode()
    : Node("move_to_goal")
  {
    this->declare_parameters("goal");
    this->get_parameters("goal", goal_params_);

    double x, y, theta;
    if (goal_params_.get_parameter("x", x) &&
        goal_params_.get_parameter("y", y) &&
        goal_params_.get_parameter("theta", theta))
    {
      goal_reached_ = false;
      goal_x_ = x;
      goal_y_ = y;
      goal_theta_ = theta;

      cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("turtleX/cmd_vel", 10);

      // Define a rate at which to send control commands
      auto timer_callback = [this]() {
        if (!goal_reached_) {
          geometry_msgs::msg::Twist cmd_vel_msg;
          // Calculate the desired velocity commands to reach the goal
          // Implement your control algorithm here
          // You can use goal_x_, goal_y_, goal_theta_ to calculate the desired velocity

          // Publish the velocity commands
          cmd_vel_pub_->publish(cmd_vel_msg);
        }
      };

      timer_ = this->create_wall_timer(std::chrono::milliseconds(100), timer_callback);
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to get goal parameters.");
    }
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Node::SharedPtr goal_params_;

  bool goal_reached_ = true;
  double goal_x_, goal_y_, goal_theta_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MoveToGoalNode>());
  rclcpp::shutdown();
  return 0;
}

