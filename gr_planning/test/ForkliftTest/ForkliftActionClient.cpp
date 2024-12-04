#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "mima_agv_services/action/get_pallet_local_location.hpp"

class ForkliftActionClient : public rclcpp::Node {
public:
    using ForkliftAction = mima_agv_services::action::GetPalletLocalLocation;

    ForkliftActionClient() : Node("forklift_action_client") {
        client_ = rclcpp_action::create_client<ForkliftAction>(this, "agv/get_pallet_local_position");

        // timer_ = this->create_wall_timer(
        //     std::chrono::milliseconds(50000),
        //     std::bind(&ForkliftActionClient::send_goal, this));

        // RCLCPP_INFO(this->get_logger(), "ForkliftActionClient is running.");
        send_goal();
    }

private:
    rclcpp_action::Client<ForkliftAction>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;

    void send_goal() {
        if (!client_->wait_for_action_server(std::chrono::seconds(1))) {
            RCLCPP_ERROR(this->get_logger(), "ForkliftPath Action Service NOT Available!");
            return;
        }

        auto goal = ForkliftAction::Goal();
        goal.pallet_types = 1;
        goal.work_type = 1;
        goal.check_pallet_alignment = false;

        RCLCPP_INFO(this->get_logger(), "Sending goal...");

        rclcpp_action::Client<ForkliftAction>::SendGoalOptions options;
        options.feedback_callback =
            [this](std::shared_ptr<rclcpp_action::ClientGoalHandle<ForkliftAction>>,
                   const std::shared_ptr<const ForkliftAction::Feedback> feedback) {
                RCLCPP_INFO(this->get_logger(), "Progress: %.2f", feedback->progress);
            };

        options.result_callback =
            [this](const rclcpp_action::ClientGoalHandle<ForkliftAction>::WrappedResult &result) {
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result->success) {
                    RCLCPP_INFO(this->get_logger(), "Received result with %zu poses.",
                                result.result->path.poses.size());
                    auto last_pose = result.result->path.poses.back();
                    RCLCPP_INFO(this->get_logger(), "Last pose - x: %f, y: %f", last_pose.pose.position.x, last_pose.pose.position.y);
                    RCLCPP_INFO(this->get_logger(), "is_line:", result.result->is_pallet_aligned);
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Goal failed or was canceled.");
                }
            };

        client_->async_send_goal(goal, options);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ForkliftActionClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}