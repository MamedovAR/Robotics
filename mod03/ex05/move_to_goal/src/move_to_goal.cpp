#include <cmath>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

class MoveToGoalNode : public rclcpp::Node
{
public:
  MoveToGoalNode()
    : Node("move_to_goal")
  {
    this->declare_parameter("x", 0.0);
    this->declare_parameter("y", 0.0);
    this->declare_parameter("theta", 0.0);

    double x = this->get_parameter("x").as_double();
    double y = this->get_parameter("y").as_double();
    double theta = this->get_parameter("theta").as_double();

    goal_reached_ = false;
    goal_x_ = x;
    goal_y_ = y;
    goal_theta_ = theta;
    if (this->get_parameter("x", x) &&
        this->get_parameter("y", y) &&
        this->get_parameter("theta", theta))
    {
      goal_reached_ = false;
      goal_x_ = x;
      goal_y_ = y;
      goal_theta_ = theta;

      cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
      cmd_vel_sub_ = this->create_subscription<turtlesim::msg::Pose>("turtle1/pose", 10, std::bind(&MoveToGoalNode::update_pose,this,std::placeholders::_1));

      // Define a rate at which to send control commands
      // auto timer_callback = [this]() {
      //   if (!goal_reached_) {
      //     geometry_msgs::msg::Twist cmd_vel_msg;
      //     double d;
      //     d = sqrt((this->goal_x_ - this->cmd_vel_sub_));
      //     cmd_vel_msg.linear.x = this->goal_x_;
      //     cmd_vel_msg.linear.y = this->goal_y_;
      //     cmd_vel_msg.angular.z = this->goal_theta_;
      //     cmd_vel_pub_->publish(cmd_vel_msg);
      //   }
      // };

      timer_ = this->create_wall_timer(std::chrono::milliseconds(100),std::bind(&MoveToGoalNode::move2goal,this));
      // move2goal();
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to get goal parameters.");
    }
  }

  void update_pose(turtlesim::msg::Pose msg)
  {
   this->pose = msg;
  }

  double euclidean_distance(turtlesim::msg::Pose goal_pose)
  {
    return std::sqrt((goal_pose.x - this->pose.x)*(goal_pose.x - this->pose.x) + (goal_pose.y - this->pose.y)*(goal_pose.y - this->pose.y));
  }

  double linear_vel(turtlesim::msg::Pose goal_pose)
  {
    return 1.5*this->euclidean_distance(goal_pose);
  }

  double steering_angle(turtlesim::msg::Pose goal_pose)
  {
    return std::atan2(goal_pose.y - this->pose.y, goal_pose.x - this->pose.x);
  }

  double angular_vel(turtlesim::msg::Pose goal_pose)
  {
    return 6 * (this->steering_angle(goal_pose) - this->pose.theta);
  }

  void move2goal()
  {
    turtlesim::msg::Pose goal_pose;
    goal_pose.x = this->goal_x_;
    goal_pose.y = this->goal_y_;
    double distance_tolerance = 0.1;
    geometry_msgs::msg::Twist vel_msg;

    if(this->euclidean_distance(goal_pose) >= distance_tolerance)
    {
      vel_msg.linear.x = this->linear_vel(goal_pose);
      vel_msg.linear.y = 0;
      vel_msg.linear.z = 0;

      vel_msg.angular.x = 0;
      vel_msg.angular.y = 0;
      vel_msg.angular.z = this->angular_vel(goal_pose);

      this->cmd_vel_pub_->publish(vel_msg);
      for(int i=0; i<=10; i++)
        this->cmd_vel_pub_->publish(vel_msg);
      vel_msg.linear.x = 0;
      vel_msg.angular.z = 0;
      this->cmd_vel_pub_->publish(vel_msg);
      // this->rate.sleep();
    }
    else
      this->timer_->cancel();
  }

private:
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Node::SharedPtr goal_params_;
  turtlesim::msg::Pose pose;
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
