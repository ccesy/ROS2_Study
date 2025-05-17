#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

using std::placeholders::_1;

class TurtleVelReceNode : public rclcpp::Node
{
public:
    TurtleVelReceNode()
        : Node("turtle_vel_rece_node")
    {
        // 创建订阅者，订阅 "/turtle1/cmd_vel" 话题，队列大小为 10
        sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/turtle1/cmd_vel", 10, std::bind(&TurtleVelReceNode::callback, this, _1));
    }

private:
    // 回调函数，处理接收到的消息
    void callback(const geometry_msgs::msg::Twist::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "Received a /cmd_vel message!");
        RCLCPP_INFO(this->get_logger(), "Linear Velocity: [%f, %f, %f]",
                    msg->linear.x, msg->linear.y, msg->linear.z);
        RCLCPP_INFO(this->get_logger(), "Angular Velocity: [%f, %f, %f]",
                    msg->angular.x, msg->angular.y, msg->angular.z);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
};

int main(int argc, char **argv)
{
    // 初始化 ROS 2
    rclcpp::init(argc, argv);

    // 创建节点并进入自旋状态
    auto node = std::make_shared<TurtleVelReceNode>();
    rclcpp::spin(node);

    // 关闭 ROS 2
    rclcpp::shutdown();
    return 0;
}