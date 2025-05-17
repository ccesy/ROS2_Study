#include <rclcpp/rclcpp.hpp>
#include <service_client_pkg/srv/service_client_ex_msg.hpp>
#include <iostream>
#include <string>

using namespace std::placeholders;

class ServiceExampleNode : public rclcpp::Node
{
public:
    ServiceExampleNode()
        : Node("service_example_node")
    {
        // 创建服务端
        service_ = this->create_service<service_client_pkg::srv::ServiceClientExMsg>(
            "info_inquiry_byname",
            std::bind(&ServiceExampleNode::infoinquiry, this, _1, _2));
        RCLCPP_INFO(this->get_logger(), "Ready to inquiry names.");
    }

private:
    // 服务回调函数
    void infoinquiry(
        const service_client_pkg::srv::ServiceClientExMsg::Request::SharedPtr request,
        service_client_pkg::srv::ServiceClientExMsg::Response::SharedPtr response)
    {
        RCLCPP_INFO(this->get_logger(), "Callback activated.");
        std::string input_name(request->name);
        response->in_class = false;

        if (input_name.compare("Tom") == 0)
        {
            RCLCPP_INFO(this->get_logger(), "Student information about Tom");
            response->in_class = true;
            response->boy = true;
            response->age = 20;
            response->personality = "outgoing";
        }
        else if (input_name.compare("Mary") == 0)
        {
            RCLCPP_INFO(this->get_logger(), "Student information about Mary");
            response->in_class = true;
            response->boy = false;
            response->age = 21;
            response->personality = "introverted";
        }
    }

    rclcpp::Service<service_client_pkg::srv::ServiceClientExMsg>::SharedPtr service_;
};

int main(int argc, char **argv)
{
    // 初始化ROS 2
    rclcpp::init(argc, argv);

    // 创建节点并进入循环
    auto node = std::make_shared<ServiceExampleNode>();
    rclcpp::spin(node);

    // 关闭ROS 2
    rclcpp::shutdown();
    return 0;
}