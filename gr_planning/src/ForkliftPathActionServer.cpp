#include "ForkliftPathActionServer.hpp"

ForkliftActionServer::ForkliftActionServer() : Node("forklift_action_server") {
    // 叉车服务
    action_server_ = rclcpp_action::create_server<GetPalletLocal>(
        this,
        "agv/get_pallet_local_position",
        std::bind(&ForkliftActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&ForkliftActionServer::handle_cancel, this, std::placeholders::_1),
        std::bind(&ForkliftActionServer::handle_accepted, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "ForkliftActionServer is READY!");

    // 检测客户端
    detect_client_ = this->create_client<ridar_service_interface::srv::DetectPointCloud>("ridar_detection_service");
    // 纠偏客户端
    path_client_ = rclcpp_action::create_client<gr_planning_interface::action::GetPath>(this, "planning_path");
}

// 获取目标值
rclcpp_action::GoalResponse ForkliftActionServer::handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const GetPalletLocal::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "ForkliftActionServer received goal request: pallet_types = %d, work_type = %d",
                goal->pallet_types, goal->work_type);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// action是否取消
rclcpp_action::CancelResponse ForkliftActionServer::handle_cancel(const std::shared_ptr<GoalHandleGetPalletLocal> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "ForkliftActionServer received request to cancel goal.");
    return rclcpp_action::CancelResponse::ACCEPT;
}

// action接受目标后
void ForkliftActionServer::handle_accepted(const std::shared_ptr<GoalHandleGetPalletLocal> goal_handle) {
    std::thread{std::bind(&ForkliftActionServer::execute, this, goal_handle)}.detach();
}

void ForkliftActionServer::execute(const std::shared_ptr<GoalHandleGetPalletLocal> goal_handle) {
    auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<GetPalletLocal::Feedback>();
    auto result = std::make_shared<GetPalletLocal::Result>();
    // 设置任务成功标志
    result->success = true;

    RCLCPP_INFO(this->get_logger(), "ForkliftActionServer executing goal: pallet_type = %d, work_type = %d",
                goal->pallet_types, goal->work_type);

    // 1. 调用检测服务，将forklift的请求转发给检测服务
    auto detect_request = std::make_shared<ridar_service_interface::srv::DetectPointCloud::Request>();
    detect_request->pallet_types = goal->pallet_types;

    if (!detect_client_->wait_for_service(std::chrono::seconds(1))) {
        result->success = false;
        goal_handle->abort(result);
        RCLCPP_ERROR(this->get_logger(), "Detect service not available!");
        return;
    }

    auto detect_future = detect_client_->async_send_request(detect_request, 
        [this, goal_handle, result, feedback](rclcpp::Client<ridar_service_interface::srv::DetectPointCloud>::SharedFuture detect_future) {
            try {
                // 获取检测结果
                auto detect_response = detect_future.get();
                RCLCPP_INFO(this->get_logger(), "Detect result: x = %f, y = %f, theta = %f",
                detect_response->x.data, detect_response->y.data, detect_response->theta.data);

                feedback->progress = 0.5;  // 反馈进度
                goal_handle->publish_feedback(feedback);

                // 设置纠偏目标
                auto path_goal = gr_planning_interface::action::GetPath::Goal();
                path_goal.lidar_pallet_pose_x = detect_response->x.data;
                path_goal.lidar_pallet_pose_y = detect_response->y.data;
                path_goal.lidar_pallet_pose_theta = detect_response->theta.data;
                path_goal.success = detect_response->success.data;

                if (!path_client_->wait_for_action_server(std::chrono::seconds(1))) {
                    result->success = false;
                    goal_handle->abort(result);
                    RCLCPP_ERROR(this->get_logger(), "ForkliftCorrectServer not available!");
                    return;
                }

                // 纠偏的反馈和结果回调
                rclcpp_action::Client<gr_planning_interface::action::GetPath>::SendGoalOptions options;
                options.feedback_callback = 
                    [this](std::shared_ptr<rclcpp_action::ClientGoalHandle<gr_planning_interface::action::GetPath>>,
                            const std::shared_ptr<const gr_planning_interface::action::GetPath::Feedback> path_feedback) {
                        RCLCPP_INFO(this->get_logger(), "ForkliftCorrectServer Progress: %.2f.", path_feedback->progress);
                    };

                options.result_callback = 
                    [this, goal_handle, result, feedback](const rclcpp_action::ClientGoalHandle<gr_planning_interface::action::GetPath>::WrappedResult &path_result) {
                        if (path_result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                            RCLCPP_INFO(this->get_logger(), "ForkliftCorrectServer received path with %zu poses.", path_result.result->path.poses.size());
                            RCLCPP_INFO(this->get_logger(), "The Goal locate is: (%f, %f).", path_result.result->path.poses.back().pose.position.x, path_result.result->path.poses.back().pose.position.y);
                            
                            // 更新结果
                            result->path = path_result.result->path;
                            feedback->progress = 1.0;
                            goal_handle->publish_feedback(feedback);

                            // 任务完成
                            goal_handle->succeed(result);
                            RCLCPP_INFO(this->get_logger(), "ForkliftActionServer goal execution completed.");
                            return;
                        } else {
                            result->success = false;
                            RCLCPP_INFO(this->get_logger(), "ForkliftActionServer goal failed or was canceled.");
                            return;
                        }
                    };
                // 判断当前角度是否符合要求，如果不对，则再重新根据检测结果运行
                if (detect_response->y.data < 8.0 || 
                    (detect_response->x.data != 0.0 && detect_response->y.data != 0.0 && detect_response->theta.data != 0.0)) {
                    // tolerance_lidar_x 是当雷达偏移到能进入托盘的极限距离 && (detect_response->x.data * detect_response->theta.data < 0)
                    if ((abs(detect_response->x.data) < tolerance_lidar_y) && 
                        (abs(detect_response->theta.data) < tolerance_theta) && 
                        (atan2(abs(detect_response->x.data), 2) * 180.0f / M_PI < tolerance_theta)) {
                        // 如果符合要求就结束纠偏
                        result->success = true;
                        result->is_pallet_aligned = true;
                        result->distance_to_pallet = detect_response->y.data;
                        goal_handle->succeed(result);
                        RCLCPP_INFO(this->get_logger(), "The forklift is line. The distance is: %f", result->distance_to_pallet);
                    } else { // 若不符合要求，则调用纠偏规划
                        result->is_pallet_aligned = false;
                        result->distance_to_pallet = detect_response->y.data;
                        RCLCPP_INFO(this->get_logger(), "The forklift not is line. The distance is: %f", result->distance_to_pallet);
                        // 2. 调用纠偏路径计算
                        path_client_->async_send_goal(path_goal, options);
                    }
                } else {
                    result->success = false;
                    goal_handle->abort(result);
                    RCLCPP_ERROR(this->get_logger(), "Detect distance is: %f. Detect data was wrong!", detect_response->y.data);
                }
            } catch(const std::exception& e) {
                result->success = false;
                goal_handle->abort(result);
                RCLCPP_ERROR(this->get_logger(), "Exception while getting service response: %s.", e.what());
            }
        }
    );
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ForkliftActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}