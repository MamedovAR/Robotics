#include <cmath>
#include <cstdlib>
#include <unistd.h>
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
    // timer_ = this->create_wall_timer(std::chrono::milliseconds(1000),std::bind(&MoveToGoalNode::move2goal,this));
    move2goal();
  }

  void stopper(geometry_msgs::msg::Twist msg, double n)
  {
    for(double i = n; i>0; i-=0.001)
    {
      msg.linear.x = i;
      this->cmd_vel_pub_->publish(msg);
      usleep(10000);
    }
  }

  void move2goal()
  {
    geometry_msgs::msg::Twist msg = geometry_msgs::msg::Twist();
    // while(true)
    // {
    //   msg.angular.z = double(std::rand()%2);
    //   msg.linear.x = double(std::rand()%6);
    //   RCLCPP_INFO(this->get_logger(), "Next moving: %f %f", msg.angular.z, msg.linear.x);
    //   cmd_vel_pub_->publish(msg);
    //   sleep(1);
    // }
    // msg.linear.x = 1;
    // cmd_vel_pub_->publish(msg);
    for(int i=0; i<5; i++)
    {
      sleep(1);
      msg.linear.x = 0.1;
      msg.angular.z = 0;
      cmd_vel_pub_->publish(msg);
      sleep(5);
      msg.linear.x = 0;
      msg.angular.z = -0.15;
      cmd_vel_pub_->publish(msg);
      sleep(5);
      msg.linear.x = 0;
      msg.angular.z = 0;
      this->cmd_vel_pub_->publish(msg);
      sleep(1);
    }
    for(int i=0; i<5; i++)
    {
      sleep(1);
      msg.linear.x = 0.1;
      msg.angular.z = 0;
      cmd_vel_pub_->publish(msg);
      sleep(5);
      msg.linear.x = 0;
      msg.angular.z = 0.15;
      cmd_vel_pub_->publish(msg);
      sleep(5);
      msg.linear.x = 0;
      msg.angular.z = 0;
      this->cmd_vel_pub_->publish(msg);
      sleep(1);
    }
    for(int i=0; i<10; i++)
    {
      sleep(1);
      msg.linear.x = 0.1;
      msg.angular.z = 0.1;
      cmd_vel_pub_->publish(msg);
      sleep(5);
      msg.linear.x = 0.1;
      msg.angular.z = -0.15;
      cmd_vel_pub_->publish(msg);
      sleep(7);
    }
    msg.linear.x = 0;
    msg.angular.z = 0;
    this->cmd_vel_pub_->publish(msg);
    sleep(1);
    // msg.linear.x = 0;
    // // msg.linear.y = 2;
    // msg.angular.z = -1;
    // cmd_vel_pub_->publish(msg);
    // sleep(1);
    // msg.linear.x = 0;
    // msg.angular.z = 0;
    // this->cmd_vel_pub_->publish(msg);
    // sleep(1);
    // msg.linear.x = 0.5;
    // cmd_vel_pub_->publish(msg);
    // sleep(2);
    // msg.linear.x = 0.15;
    // // msg.linear.y = 2;
    // msg.angular.z = -1.5;
    // cmd_vel_pub_->publish(msg);
    // sleep(2);
    // msg.linear.x = 0;
    // msg.angular.z = 0;
    // this->cmd_vel_pub_->publish(msg);
    // sleep(1);
    // msg.linear.x = 0.5;
    // msg.angular.z = 0.5;
    // cmd_vel_pub_->publish(msg);
    // sleep(3);
    // msg.linear.x = 0.1;
    // msg.angular.z = 0.1;
    // this->cmd_vel_pub_->publish(msg);
    // sleep(4);
    // stopper(msg,0.1);
    // msg.linear.x = 0;
    // msg.angular.z = 0;
    // this->cmd_vel_pub_->publish(msg);
    // sleep(1);
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