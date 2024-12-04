#include "rclcpp/rclcpp.hpp"
#include "gr_planning_interface/srv/get_path.hpp"
#include "gr_planning_interface/srv/test_path.hpp"
#include "ridar_service_interface/srv/detect_point_cloud.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>
#include <mutex>
#include <condition_variable>

class ForkliftCorrectClient : public rclcpp::Node {
public:
    ForkliftCorrectClient() : Node("PlanningPath") {
        forklift_service_ = this->create_service<gr_planning_interface::srv::TestPath>(
            "forklift_test", std::bind(&ForkliftCorrectClient::handle_request, this, std::placeholders::_1, std::placeholders::_2));
        // 检测客户端
        detect_client_ = this->create_client<ridar_service_interface::srv::DetectPointCloud>("ridar_detection_service");
        // 路径客户端
        path_client_ = this->create_client<gr_planning_interface::srv::GetPath>("planning_path");

        RCLCPP_INFO(this->get_logger(), "ForkliftPath Client is Ready.");
    }
private:
    rclcpp::Client<gr_planning_interface::srv::GetPath>::SharedPtr path_client_;
    rclcpp::Client<ridar_service_interface::srv::DetectPointCloud>::SharedPtr detect_client_;
    rclcpp::Service<gr_planning_interface::srv::TestPath>::SharedPtr forklift_service_;

    std::mutex mutex_;
    std::condition_variable cv_;
    bool result_ready_ = false;

    void handle_request(const std::shared_ptr<gr_planning_interface::srv::TestPath::Request> request,
                        std::shared_ptr<gr_planning_interface::srv::TestPath::Response> response) {
        RCLCPP_INFO(this->get_logger(), "The pallet type and work type is: %d, %d.", request->pallet_types, request->work_type);

        // 转发 forklift client 的请求到 ridar service
        auto detect_request = std::make_shared<ridar_service_interface::srv::DetectPointCloud::Request>();
        detect_request->pallet_types = request->pallet_types;

        if (!detect_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "Waiting for detect service...");
            return;
        }

        // 从ridar service中获取托盘位置
        auto detect_future = detect_client_->async_send_request(detect_request,
            [this, response](rclcpp::Client<ridar_service_interface::srv::DetectPointCloud>::SharedFuture detect_future) {
                try {
                    auto detect_response = detect_future.get();
                    RCLCPP_INFO(this->get_logger(), "[Detect Client] The pallet pose in lidar: %f, %f, %f.",
                                detect_response->x.data, detect_response->y.data, detect_response->theta.data);
                    // 路径计算
                    process_detect_response(detect_response, response);

                    RCLCPP_INFO(this->get_logger(), "[Forklift client2] Received path with %zu poses.", response->path.poses.size());
                } catch(const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Exception while getting service response: %s.", e.what());
                }
                
            }
        );       
    }

    void process_detect_response(const ridar_service_interface::srv::DetectPointCloud::Response::SharedPtr detect_response,
                                 std::shared_ptr<gr_planning_interface::srv::TestPath::Response> response) {
        auto path_request = std::make_shared<gr_planning_interface::srv::GetPath::Request>();
        path_request->lidar_pallet_pose_x = detect_response->x.data;
        path_request->lidar_pallet_pose_y = detect_response->y.data;
        path_request->lidar_pallet_pose_theta = detect_response->theta.data;
        path_request->success = detect_response->success.data;

        if (!path_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "Waiting for path planning service...");
            return;
        }

        // path_client_->async_send_request(path_request,
        //     [this, &response](rclcpp::Client<gr_planning_interface::srv::GetPath>::SharedFuture path_future) {
        //         try {
        //             auto path_response = path_future.get();
        //             RCLCPP_INFO(this->get_logger(), "[Path Client] Received path with %zu poses.", path_response->path.poses.size());
        //             // 将计算结果返回给 forklift client
        //             response.path = path_response->path;
        //             RCLCPP_INFO(this->get_logger(), "[Forklift client] Received path with %zu poses.", response.path.poses.size());
        //             // for (size_t i = 0; i < path_response->path.poses.size(); ++i) {
        //             //     const auto &pose = path_response->path.poses[i];
        //             //     double angle = path_response->angle[i];
        //             //     RCLCPP_INFO(this->get_logger(), "Pose - x: %f, y: %f, angle: %f.", 
        //             //                 pose.pose.position.x, pose.pose.position.y, angle);
        //             // }
        //         } catch (const std::exception &e) {
        //             RCLCPP_ERROR(this->get_logger(), "Exception while processing path response: %s.", e.what());
        //         }
        //     });

        // 使用条件变量和互斥锁等待路径计算完成
        std::thread([this, path_request, response]() {
            std::unique_lock<std::mutex> lock(mutex_);

            // 异步发送请求
            auto future = path_client_->async_send_request(path_request);
            future.wait();

            if (future.valid()) {
                auto path_response = future.get();
                RCLCPP_INFO(this->get_logger(), "[Path Client] Received path with %zu poses.", path_response->path.poses.size());

                // 更新 response 并唤醒主线程
                response->path = path_response->path;
                result_ready_ = true;
                cv_.notify_one();
            } else {
                RCLCPP_INFO(this->get_logger(), "Failed to call path planning service.");
            }
        }).detach();

        // 等待路径计算完成
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [this]() {return result_ready_;});

        RCLCPP_INFO(this->get_logger(), "[Forklift Service] Response is ready with %zu poses.", response->path.poses.size());
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ForkliftCorrectClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
