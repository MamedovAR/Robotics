#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "full_name_message/srv/full_name_sum_service.hpp"

class ServiceNameNode : public rclcpp::Node
{
public:
    ServiceNameNode() : Node("service_name")
    {
        service_ = create_service<full_name_message::srv::FullNameSumService>(
            "SumFullName",
            std::bind(&ServiceNameNode::calculateFullName, this,
                      std::placeholders::_1, std::placeholders::_2));
    }

private:
    void calculateFullName(
        const std::shared_ptr<full_name_message::srv::FullNameSumService::Request> request,
        std::shared_ptr<full_name_message::srv::FullNameSumService::Response> response)
    {   
        response->full_name = request->last_name + ' ' + request->name + ' ' + request->first_name;
        std::cout << response->full_name << "\n";
        RCLCPP_INFO(this->get_logger(), "%s", response->full_name);
    }

    rclcpp::Service<full_name_message::srv::FullNameSumService>::SharedPtr service_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ServiceNameNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
