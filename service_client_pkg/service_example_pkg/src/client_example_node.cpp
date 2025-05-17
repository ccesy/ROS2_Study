#include <rclcpp/rclcpp.hpp>
#include <service_client_pkg/srv/service_client_ex_msg.hpp>
#include <iostream>
#include <string>

using namespace std::chrono_literals;

class ClientExampleNode : public rclcpp::Node
{
public:
    ClientExampleNode()
        : Node("client_example_node")
    {
        // 创建服务客户端
        client_ = this->create_client<service_client_pkg::srv::ServiceClientExMsg>("info_inquiry_byname");
        RCLCPP_INFO(this->get_logger(), "Client node started.");
    }

    void send_request(const std::string &name)
    {
        // 等待服务端可用
        while (!client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }

        // 创建请求
        auto request = std::make_shared<service_client_pkg::srv::ServiceClientExMsg::Request>();
        request->name = name;

        // 发送请求
        auto result = client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            // 处理响应
            auto response = result.get();
            if (response->in_class) {
                std::cout << request->name << " is " << (response->boy ? "boy" : "girl") << ";" << std::endl;
                std::cout << request->name << " is " << response->age << " years old;" << std::endl;
                std::cout << request->name << " is " << response->personality << "." << std::endl;
            } else {
                std::cout << request->name << " is not in class." << std::endl;
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service info_inquiry_byname");
        }
    }

private:
    rclcpp::Client<service_client_pkg::srv::ServiceClientExMsg>::SharedPtr client_;
};

int main(int argc, char **argv)
{
    // 初始化ROS 2
    rclcpp::init(argc, argv);

    // 创建节点
    auto node = std::make_shared<ClientExampleNode>();

    // 用户输入循环
    std::string input_name;
    while (rclcpp::ok()) {
        std::cout << std::endl;
        std::cout << "Enter a name (q to quit): ";
        std::cin >> input_name;
        if (input_name.compare("q") == 0) {
            break;
        }
        node->send_request(input_name);
    }

    // 关闭ROS 2
    rclcpp::shutdown();
    return 0;
}