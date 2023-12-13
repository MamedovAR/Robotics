#include <cmath>
#include <cstdlib>
#include <unistd.h>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class MoveToGoalNode : public rclcpp::Node
{
public:
  MoveToGoalNode()
    : Node("my_robot_control")
  {
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/robot/cmd_vel", 10);
    subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>("/robot/scan", 10, [](const sensor_msgs::msg::LaserScan msg){});
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000),std::bind(&MoveToGoalNode::move2goal,this));
    RCLCPP_INFO(this->get_logger(), "Succesfully created node");
    move2goal();
  }


  void getsensor(sensor_msgs::msg::LaserScan* msg)
  {
    rclcpp::MessageInfo minfo;
    while(!subscriber->take(*msg, minfo));
  }

  void move2goal()
  {
    bool a;
    geometry_msgs::msg::Twist msg = geometry_msgs::msg::Twist();
    // while(true)
    // {
      sensor_msgs::msg::LaserScan lidar;
      getsensor(&lidar);

      geometry_msgs::msg::Twist vel_msg; 

      std::cout << lidar.ranges[0] << "\n";

      vel_msg.linear.x = 0.1;
      cmd_vel_pub_->publish(vel_msg);

      for(size_t i = 0; i < lidar.ranges.size(); i += 1)
      {
        if(lidar.ranges[i] < 1)
        {
          vel_msg.linear.x = 0;
          cmd_vel_pub_->publish(vel_msg);
          a = true;
          break;
        }
      }
    // }
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber;
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