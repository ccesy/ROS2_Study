#include <memory>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

class TFBroadcaster : public rclcpp::Node {
public:
    TFBroadcaster() : Node("tf_broadcaster") {
        // 初始化静态变换关系
        geometry_msgs::msg::TransformStamped transform;
        transform.transform.translation.x = 0.1;
        transform.transform.translation.y = 0.0;
        transform.transform.translation.z = 0.2;
        transform.transform.rotation.w = 1.0;
        transform.header.frame_id = "base_link";
        transform.child_frame_id = "base_laser";
        transform.header.stamp = now();

        // 发布静态变换到/tf_static
        static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        static_broadcaster_->sendTransform(transform);
    }

private:
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TFBroadcaster>());
    rclcpp::shutdown();
    return 0;
}