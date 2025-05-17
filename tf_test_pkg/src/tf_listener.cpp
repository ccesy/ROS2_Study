// private:
//     void timer_callback() {
//         laser_pos_.header.stamp = now();
        
//         try {
//             // 直接进行坐标变换（自动处理时间戳）
//             auto base_pos = tf_buffer_->transform(laser_pos_, "base_link");
            
//             RCLCPP_INFO(get_logger(), "base_laser坐标点: (%.2f, %.2f, %.2f)",
//                 laser_pos_.point.x, laser_pos_.point.y, laser_pos_.point.z);
//             RCLCPP_INFO(get_logger(), "base_link坐标点: (%.2f, %.2f, %.2f)",
//                 base_pos.point.x, base_pos.point.y, base_pos.point.z);

//             // 获取变换参数（使用数据的时间戳）
//             auto transform = tf_buffer_->lookupTransform(
//                 "base_link", "base_laser", laser_pos_.header.stamp);
//             RCLCPP_INFO(get_logger(), "变换参数 - X:%.2f Y:%.2f Z:%.2f",
//                 transform.transform.translation.x,
//                 transform.transform.translation.y,
//                 transform.transform.translation.z);
//         } catch (const tf2::TransformException &ex) {
//             RCLCPP_WARN(get_logger(), "坐标变换异常: %s", ex.what());
//         }
//     }

//     geometry_msgs::msg::PointStamped laser_pos_;
//     std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
//     std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
//     rclcpp::TimerBase::SharedPtr timer_;
// };

// int main(int argc, char **argv) {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<TFListener>());
//     rclcpp::shutdown();
//     return 0;
// }

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

class TFListener : public rclcpp::Node {
public:
    TFListener() : Node("tf_listener") {
        // 初始化Buffer和Listener（关键：自动订阅/tf和/tf_static）
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);
        
        timer_ = create_wall_timer(100ms, [this]() { timer_callback(); });
        
        laser_pos_.header.frame_id = "base_laser";
        laser_pos_.point.x = 0.3;
        laser_pos_.point.y = 0.0;
        laser_pos_.point.z = 0.0;
    }

private:
    void timer_callback() {
        laser_pos_.header.stamp = now();
        
        try {
            // 使用显式时间戳和超时
            auto base_pos = tf_buffer_->transform(
                laser_pos_, "base_link", tf2::durationFromSec(1.0)
            );
            
            RCLCPP_INFO(get_logger(), "base_link坐标点: (%.2f, %.2f, %.2f)",
                base_pos.point.x, base_pos.point.y, base_pos.point.z);

            // 获取静态变换参数
            auto transform = tf_buffer_->lookupTransform(
                "base_link", "base_laser",
                tf2::TimePointZero,  // 显式请求静态变换
                tf2::durationFromSec(1.0)
            );
            RCLCPP_INFO(get_logger(), "静态变换参数 - X:%.2f Y:%.2f Z:%.2f",
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(get_logger(), "坐标变换异常: %s", ex.what());
        }
    }

    geometry_msgs::msg::PointStamped laser_pos_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TFListener>());
    rclcpp::shutdown();
    return 0;
}