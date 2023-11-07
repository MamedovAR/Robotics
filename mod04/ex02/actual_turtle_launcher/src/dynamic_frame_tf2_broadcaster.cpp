// Copyright 2021 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <functional>
#include <memory>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

const double PI = 3.141592653589793238463;

class DynamicFrameBroadcaster : public rclcpp::Node
{
public:
  DynamicFrameBroadcaster()
  : Node("dynamic_frame_tf2_broadcaster")
  {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    // timer_ = this->create_wall_timer(
    //   100ms, std::bind(&DynamicFrameBroadcaster::broadcast_timer_callback, this));
    this->declare_parameter("radius", 5.0);
    this->declare_parameter("direction_of_rotation", 1);
    this->get_parameter("radius", radius_);
    this->get_parameter("direction_of_rotation", direction_);
    rotateCarrot();
  }

private:
  // void broadcast_timer_callback()
  // {
  //   rclcpp::Time now = this->get_clock()->now();
  //   double x = now.seconds() * PI;

  //   geometry_msgs::msg::TransformStamped t;
  //   t.header.stamp = now;
  //   t.header.frame_id = "turtle1";
  //   t.child_frame_id = "carrot1";
  //   t.transform.translation.x = 2 * sin(x);
  //   t.transform.translation.y = 2 * cos(x);
  //   t.transform.translation.z = 0.0;
  //   t.transform.rotation.x = 0.0;
  //   t.transform.rotation.y = 0.0;
  //   t.transform.rotation.z = 0.0;
  //   t.transform.rotation.w = 1.0;

  //   tf_broadcaster_->sendTransform(t);
  // }
  void rotateCarrot()
  {
    this->angle_ = 0;
    rclcpp::Rate rate(10);  // Частота обновления трансформации
    while (rclcpp::ok())
    {
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = now();
      transform.header.frame_id = "turtle1";
      transform.child_frame_id = "carrot1";

      transform.transform.translation.x = this->radius_ * std::cos(this->angle_);
      transform.transform.translation.y = this->radius_ * std::sin(this->angle_);
      transform.transform.translation.z = 0.0;
      transform.transform.rotation.x = 0.0;
      transform.transform.rotation.y = 0.0;
      transform.transform.rotation.z = 0.0;
      transform.transform.rotation.w = 1.0;

      tf_broadcaster_->sendTransform(transform);

      angle_ += 0.5 * this->direction_;  // Угол поворота морковки (может быть настроен)

      rate.sleep();
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  double radius_, angle_;
  int direction_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DynamicFrameBroadcaster>());
  rclcpp::shutdown();
  return 0;
}
