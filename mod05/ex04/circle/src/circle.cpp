#include <cmath>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class MoveToGoalNode : public rclcpp::Node
{
public:
  MoveToGoalNode()
    : Node("my_robot_control")
  {
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100),std::bind(&MoveToGoalNode::move2goal,this));
  }

  void move2goal()
  {
        geometry_msgs::msg::Twist msg = geometry_msgs::msg::Twist();
        msg.angular.z = -2.0;
        msg.linear.x = 5.0;
        RCLCPP_INFO(this->get_logger(), "Next moving: %f %f", msg.angular.z, msg.linear.x);
        cmd_vel_pub_->publish(msg); 
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Node::SharedPtr goal_params_;
  rclcpp::TimerBase::SharedPtr timer_;
  // rclcpp::Rate rate;

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