#include <cmath>
#include <vector>
#include <cstdlib>
#include <unistd.h>
#include <iostream>
#include <SFML/Graphics.hpp>
#include "cv_bridge/cv_bridge.h"
// #include "/opt/ros/humble/include/cv_bridge/cv_bridge/cv_bridge.h"
#include <opencv2/highgui.hpp>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/twist.hpp"

// std::vector<unsigned char> imageToVector(const cv::Mat& image) {
//   std::vector<unsigned char> buffer;
//   std::vector<int> params{cv::IMWRITE_JPEG_QUALITY, 90}; // Настройка параметров кодирования (JPEG, качество 90%)

//   // Кодирование изображения в буфер
//   cv::imencode(".jpg", image, buffer, params);

//   return buffer;
// }

class MoveToGoalNode : public rclcpp::Node
{
public:
  MoveToGoalNode()
    : Node("my_robot_control")
  {
    // bridge = std::make_shared<cv_bridge::CvImage>();
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/robot/cmd_vel", 10);
    subscriber = this->create_subscription<sensor_msgs::msg::Image>("/depth/image", 10, std::bind(&MoveToGoalNode::move2goal, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100),std::bind(&MoveToGoalNode::timer_callback,this));
    RCLCPP_INFO(this->get_logger(), "Succesfully created node");
    // move2goal();
  }

  void timer_callback()
  {
    geometry_msgs::msg::Twist twist;
    if(this->flag)
      twist.linear.x = 0;
    else
      twist.linear.x = 0.1;
    this->cmd_vel_pub_->publish(twist);
  }

  void move2goal(sensor_msgs::msg::Image msg)
  {
      cv::Mat depth_image = cv_bridge::toCvCopy(msg, msg.encoding)->image;
      int h = depth_image.rows;
      int w = depth_image.cols;
      // depth_image->at(h*240+w/2);
      RCLCPP_INFO(this->get_logger(), "Depth Image Size: %dx%d", h, w);
      std::cout << sizeof(((float*)(msg.data.data()))) << "\n";
      float* arr = depth_image.ptr<float>();
      RCLCPP_INFO(this->get_logger(), "Create array");
      int k = 1;
      float sr = 0;
      for(int i = -1; i <= 1; i++)
        for(int j = -1; j <= 1; j++)
        {
          sr += arr[(h*(240 + i)+w/2 + j)];
          k++;
        }
      sr /= float(k);
      RCLCPP_INFO(this->get_logger(), "Distance: %f\n\n\n", sr);
      flag = (sr < 1);
  };

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber;
  rclcpp::Node::SharedPtr goal_params_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool flag;
  std::shared_ptr<cv_bridge::CvImage> bridge;
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