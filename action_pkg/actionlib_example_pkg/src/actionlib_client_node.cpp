#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "actionlib_pkg/action/actionlib_ex_msg.hpp"

class ActionClientNode : public rclcpp::Node {
public:
    using ActionType = actionlib_pkg::action::ActionlibExMsg;
    using GoalHandle = rclcpp_action::ClientGoalHandle<ActionType>;

    ActionClientNode() : Node("actionlib_client_node") {
        client_ = rclcpp_action::create_client<ActionType>(this, "moving_forward");
        RCLCPP_INFO(this->get_logger(), "Waiting for action server to start...");
        if (!client_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available!");
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Action server started");
        send_goal();
    }

private:
    rclcpp_action::Client<ActionType>::SharedPtr client_;

    void send_goal() {
        auto goal_msg = ActionType::Goal();
        goal_msg.whole_distance = 10;

        auto send_goal_options = rclcpp_action::Client<ActionType>::SendGoalOptions();

        send_goal_options.goal_response_callback = 
            [this](const GoalHandle::SharedPtr &goal_handle) {
                if (!goal_handle) {
                    RCLCPP_ERROR(this->get_logger(), "Goal rejected by server");
                } else {
                    RCLCPP_INFO(this->get_logger(), "Goal accepted, executing...");
                }
            };

        send_goal_options.feedback_callback = 
            [this](GoalHandle::SharedPtr /*goal_handle*/, 
                   const std::shared_ptr<const ActionType::Feedback> feedback) {
                RCLCPP_INFO(this->get_logger(), "Progress: %d meters", feedback->moving_meter);
            };

        send_goal_options.result_callback = 
            [this](const GoalHandle::WrappedResult &result) {
                switch (result.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        RCLCPP_INFO(this->get_logger(), "Task succeeded!");
                        break;
                    case rclcpp_action::ResultCode::ABORTED:
                        RCLCPP_ERROR(this->get_logger(), "Task aborted");
                        break;
                    case rclcpp_action::ResultCode::CANCELED:
                        RCLCPP_WARN(this->get_logger(), "Task canceled");
                        break;
                    default:
                        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                }
                rclcpp::shutdown();
            };

        client_->async_send_goal(goal_msg, send_goal_options);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ActionClientNode>();
    rclcpp::spin(node);
    return 0;
}