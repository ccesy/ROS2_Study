#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

using namespace std::chrono_literals;

class TurtleVelCtrlNode : public rclcpp::Node
{
public:
    TurtleVelCtrlNode()
        : Node("turtle_vel_ctrl_node")
    {
        // 创建发布者，发布到 "/turtle1/cmd_vel" 话题，队列大小为 10
        vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

        // 创建一个定时器，每100ms发布一次速度命令
        timer_ = this->create_wall_timer(100ms, std::bind(&TurtleVelCtrlNode::timer_callback, this));
    }

private:
    void timer_callback()
    {
        // 创建 Twist 消息并设置线速度和角速度
        auto vel_cmd = geometry_msgs::msg::Twist();
        vel_cmd.linear.x = 0.1;
        vel_cmd.linear.y = 0.0;
        vel_cmd.linear.z = 0.0;
        vel_cmd.angular.x = 0.0;
        vel_cmd.angular.y = 0.0;
        vel_cmd.angular.z = 0.0;

        // 发布速度命令
        vel_pub_->publish(vel_cmd);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    // 初始化ROS 2
    rclcpp::init(argc, argv);

    // 创建节点并进入自旋状态
    auto node = std::make_shared<TurtleVelCtrlNode>();
    rclcpp::spin(node);

    // 关闭ROS 2
    rclcpp::shutdown();
    return 0;
}