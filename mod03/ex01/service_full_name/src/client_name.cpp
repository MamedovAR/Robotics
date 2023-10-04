#include <iostream>
#include <string.h>
#include "rclcpp/rclcpp.hpp"
#include "full_name_message/srv/full_name_sum_service.hpp"

class ClientNameNode : public rclcpp::Node
{
public:
    ClientNameNode() : Node("client_name")
    {
        client_ = create_client<full_name_message::srv::FullNameSumService>("SumFullName");
    }

    void run(const std::string &last_name, const std::string &name, const std::string &first_name)
    {
        auto request = std::make_shared<full_name_message::srv::FullNameSumService::Request>();
        request->last_name = last_name;
        request->name = name;
        request->first_name = first_name;

        while (!client_->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(get_logger(), "Service not available, trying again...");
        }

        auto future_result = client_->async_send_request(request);
    }

private:
    rclcpp::Client<full_name_message::srv::FullNameSumService>::SharedPtr client_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ClientNameNode>();
    std::string last_name, first_name, name;
    std::cout << argc << "\n";
    for(int i=argc-1; i>=argc-3; i--)
    {
        char k=0;
        for(int j=0; j<strlen(argv[i]); j++)
        {
            if(argv[i][2]=='f')
            {
                if(k==1)
                {
                    first_name+=(argv[i][j]);
                    // break;
                }
                else if(argv[i][j]=='=')
                    k++;
            }
            else if(argv[i][2]=='l')
            {
                if(k==1)
                {
                    first_name+=(argv[i][j]);
                    // break;
                }
                else if(argv[i][j]=='=')
                    k++;
            }
            else if(argv[i][2]=='n')
            {
                if(k==1)
                {
                    first_name+=(argv[i][j]);
                    // break;
                }
                else if(argv[i][j]=='=')
                    k++;
            }
            else
            {
                std::cout << "Use --first_name=, --last_name=, --name=\n";
                exit(0);
            }
        }
    }
    std::cout << last_name << name << first_name;
    node->run(last_name, name, first_name);
    rclcpp::shutdown();
    return 0;
}
