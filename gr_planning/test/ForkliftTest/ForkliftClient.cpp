#include "rclcpp/rclcpp.hpp"
#include "gr_planning_interface/srv/test_path.hpp"

class ForkliftClient : public rclcpp::Node {
public:
    ForkliftClient() : Node("ForkliftClient") {
        client_ = this->create_client<gr_planning_interface::srv::TestPath>("forklift_test");

        int p = 1, w = 1;
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            [this, p, w]() {
                this->send_request(p, w);
            }
        );

        send_request(1, 1);
    }

    void send_request(int p, int w) {
        if (!client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "Waiting for service...");
            return;
        }
        auto request = std::make_shared<gr_planning_interface::srv::TestPath::Request>();
        
        // 设置请求参数
        request->pallet_types = p;
        request->work_type = w;

        RCLCPP_INFO(this->get_logger(), "Sending request to service...");

        using ResponseFuture = rclcpp::Client<gr_planning_interface::srv::TestPath>::SharedFuture;
        auto response_received_callback = [this](ResponseFuture future) {
            try {
                auto response = future.get();
                // 清空并存储路径响应
                RCLCPP_INFO(this->get_logger(), "Received path with %zu poses.", response->path.poses.size());

                // 遍历路径的每个位置点
                for (size_t i = 0; i < response->path.poses.size(); ++i) {
                    const auto& pose = response->path.poses[i];

                    RCLCPP_INFO(this->get_logger(), "Pose - x: %f, y: %f.", 
                                pose.pose.position.x, pose.pose.position.y);
                }

            } catch(const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Exception while getting service response: %s.", e.what());
            }
            
        };

        auto future = client_->async_send_request(request, response_received_callback);
    }

private:
    rclcpp::Client<gr_planning_interface::srv::TestPath>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ForkliftClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
