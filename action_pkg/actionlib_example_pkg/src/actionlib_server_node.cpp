#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "actionlib_pkg/action/actionlib_ex_msg.hpp"

using namespace std::chrono_literals;

class ActionServerNode : public rclcpp::Node {
public:
    using ActionType = actionlib_pkg::action::ActionlibExMsg;
    using GoalHandle = rclcpp_action::ServerGoalHandle<ActionType>;

    explicit ActionServerNode() : Node("actionlib_server_node") {
        // 创建 Action Server
        action_server_ = rclcpp_action::create_server<ActionType>(
            this,
            "moving_forward",
            std::bind(&ActionServerNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ActionServerNode::handle_cancel, this, std::placeholders::_1),
            std::bind(&ActionServerNode::handle_accepted, this, std::placeholders::_1));
    }

private:
    rclcpp_action::Server<ActionType>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const ActionType::Goal> goal) {
        (void)uuid;
        RCLCPP_INFO(this->get_logger(), "Received new goal: %d meters", goal->whole_distance);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandle> goal_handle) {
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(), "Goal cancellation request received");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle) {
        std::thread{std::bind(&ActionServerNode::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandle> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Executing goal...");
        
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<ActionType::Feedback>();
        auto result = std::make_shared<ActionType::Result>();
        rclcpp::Rate rate(0.5);  // 0.5Hz（2秒/次）

        for (int i = 1; i <= goal->whole_distance && rclcpp::ok(); ++i) {
            if (goal_handle->is_canceling()) {
                result->is_finish = false;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                return;
            }

            feedback->moving_meter = i;
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(this->get_logger(), "Progress: %d meters", i);
            
            rate.sleep();  // 等待2秒
        }

        if (rclcpp::ok()) {
            result->is_finish = true;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ActionServerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}