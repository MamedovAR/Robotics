#include <cmath>
#include <vector>
#include <cstdlib>
#include <unistd.h>
#include <iostream>
#include <SFML/Graphics.hpp>
#include "cv_bridge/cv_bridge.h"
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
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/robot/cmd_vel", 10);
    subscriber = this->create_subscription<sensor_msgs::msg::Image>("/depth/image", 10, std::bind(&MoveToGoalNode::move2goal, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000),std::bind(&MoveToGoalNode::timer_callback,this));
    RCLCPP_INFO(this->get_logger(), "Succesfully created node");
    // move2goal();
  }

  void timer_callback()
  {
    geometry_msgs::msg::Twist twist;
    if(this->flag)
      twist.linear.x = 0;
    else
      twist.linear.x = 1;
    this->cmd_vel_pub_->publish(twist);
  }

  void getsensor(sensor_msgs::msg::Image* msg)
  {
    rclcpp::MessageInfo minfo;
    while(!subscriber->take(*msg, minfo));
  }

  void move2goal(sensor_msgs::msg::Image msg)
  {
    bool a;
    geometry_msgs::msg::Twist msg = geometry_msgs::msg::Twist();
    // while(true)
    // {
      sensor_msgs::msg::Image camera;
      getsensor(&camera);
      try {
        // Преобразование из сообщения ROS в формат OpenCV
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        // Здесь можно выполнять операции с изображением с использованием OpenCV

        // Пример: получение ширины и высоты изображения
        int width = cv_ptr->image.cols;
        int height = cv_ptr->image.rows;
        // std::vector<unsigned char> imageBuffer = imageToVector(cv_ptr->image);
        
        this->flag = cv_ptr->image.at<int>(width*width/2 + height/2) < 3;//imageBuffer[width*width/2 + height/2];

      } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("cv_bridge_examples"), "cv_bridge exception: " << e.what());
      }

      // int height = camera.height;
      // int width = camera.data.size()/height;

      // std::cout << height << width << "\n";

      // sf::RenderWindow window(sf::VideoMode(width, height), "Display Image");

      // // Создание текстуры и загрузка данных из вашего вектора
      // sf::Texture texture;
      // texture.create(width, height);
      // texture.update(&camera.data[0]);

      // // Создание спрайта для отображения текстуры
      // sf::Sprite sprite(texture);

      // while (window.isOpen()) {
      //     sf::Event event;
      //     while (window.pollEvent(event)) {
      //         if (event.type == sf::Event::Closed)
      //             window.close();
      //     }

      //     window.clear();
      //     window.draw(sprite);
      //     window.display();
      // }

      // geometry_msgs::msg::Twist vel_msg; 

      // std::cout << camera.data[0] << "\n";

      // for(size_t i = 0; i < camera.data.size(); i += 1)
      // {
      //   if(camera.data[i] < 1)
      //   {
      //     a = true;
      //     break;
      //   }
      // }

      // if(a)
      // {
      //   vel_msg.linear.x = 0;
      //   cmd_vel_pub_->publish(vel_msg);
      // }
      // else
      // {
      //   vel_msg.linear.x = 0.1;
      //   cmd_vel_pub_->publish(vel_msg);
      // }
    // }
    // msg.linear.x = 1;
    // cmd_vel_pub_->publish(msg);
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber;
  rclcpp::Node::SharedPtr goal_params_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool flag;
  cv_bridge::CvImage bridge;
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