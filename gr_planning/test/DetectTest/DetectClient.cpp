#include "rclcpp/rclcpp.hpp"
#include "ridar_service_interface/srv/detect_point_cloud.hpp"

class DetectClient : public rclcpp::Node {
public:
    DetectClient() : Node("DetectClient") {
        client_ = this->create_client<ridar_service_interface::srv::DetectPointCloud>("ridar_detection_service");
    }

    void send_request() {
        if (!client_->service_is_ready()) {
            RCPCPP_ERROR(this->get_logger(), "Can't connect service!");
            return;
        }

        auto request = std::make_shared<ridar_service_interface::srv::DetectPointCloud::Request>();
        
        // 设置请求参数
        request->pallet_types = 1;

        RCLCPP_INFO(this->get_logger(), "Sending request to service...");

        using ResponseFuture = rclcpp::Client<ridar_service_interface::srv::DetectPointCloud>::SharedFuture;
        auto response_received_callback = [this](ResponseFuture future) {
            try {
                auto response = future.get();
                // 清空并存储路径响应
                RCLCPP_INFO(this->get_logger(), "Received path with %d poses", response->success.data);

                RCLCPP_INFO(this->get_logger(), "Pose - x: %f, y: %f, theta: %f", 
                            response->x.data, response->y.data, response->theta.data);
            } catch(const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Exception while getting service response: %s", e.what());
            }
            
        };

        auto future = client_->async_send_request(request, response_received_callback);
    }

private:
    rclcpp::Client<ridar_service_interface::srv::DetectPointCloud>::SharedPtr client_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DetectClient>();

    node->send_request();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
