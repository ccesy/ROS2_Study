#include "rclcpp/rclcpp.hpp"

class GetParameterNode : public rclcpp::Node
{
public:
    GetParameterNode() : Node("get_parameter_node")
    {
        // 声明参数并设置默认值
        this->declare_parameter("kinect_height", 0.0);
        this->declare_parameter("kinect_pitch", 0.0);

        // 获取参数值
        double kinect_height_getting = this->get_parameter("kinect_height").as_double();
        double kinect_pitch_getting = this->get_parameter("kinect_pitch").as_double();

        // 打印参数值
        RCLCPP_INFO(this->get_logger(), "kinect_height set to %f", kinect_height_getting);
        RCLCPP_INFO(this->get_logger(), "kinect_pitch set to %f", kinect_pitch_getting);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GetParameterNode>());
    rclcpp::shutdown();
    return 0;
}