#include "ForkliftCorrectServer.hpp"

ForkliftCorrectServer::ForkliftCorrectServer() : Node("gr_planning_forklift_correct_server") {
    this->declare_parameter<std::string>("topic.planning_service_topic", "/gr_planning_forklift_correct_server");
    this->get_parameter<std::string>("topic.planning_service_topic", plan_topic_);

    this->declare_parameter<double>("lidar.delta_x", 0);
    this->get_parameter<double>("lidar.delta_x", delta_x);
    this->declare_parameter<double>("lidar.delta_y", 0);
    this->get_parameter<double>("lidar.delta_y", delta_y);
    this->declare_parameter<double>("lidar.delta_theta", 0);
    this->get_parameter<double>("lidar.delta_theta", delta_theta);

    RCLCPP_INFO(this->get_logger(), "Get Parameter plan_topic is: %s", plan_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Get Parameter delta_x, delta_y, delta_theta : %f, %f, %f.", delta_x, delta_y, delta_theta);

    // 创建一个服务
    service_ = rclcpp_action::create_server<ForkliftPathAction>(
        this,
        plan_topic_,
        std::bind(&ForkliftCorrectServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&ForkliftCorrectServer::handle_cancel, this, std::placeholders::_1),
        std::bind(&ForkliftCorrectServer::handle_accepted, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "ForkliftCorrectServer is READY!");
    // 创建路径发布器
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("forklift_path", 10);

    planner_ = std::make_unique<ForkliftPath>();
}

rclcpp_action::GoalResponse ForkliftCorrectServer::handle_goal(const rclcpp_action::GoalUUID &uuid,
                                                                std::shared_ptr<const ForkliftPathAction::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "ForkliftCorrectServer received goal request.");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ForkliftCorrectServer::handle_cancel(const std::shared_ptr<GoalHandleForkliftPath> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "ForkliftCorrectServer received request to cancel goal.");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void ForkliftCorrectServer::handle_accepted(const std::shared_ptr<GoalHandleForkliftPath> goal_handle) {
    std::thread(std::bind(&ForkliftCorrectServer::planning, this, goal_handle)).detach();
}

void ForkliftCorrectServer::planning(const std::shared_ptr<GoalHandleForkliftPath> goal_handle) {
    auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<ForkliftPathAction::Feedback>();
    auto result = std::make_shared<ForkliftPathAction::Result>();

    nav_msgs::msg::Path path_;

    if (!goal->success) {
        RCLCPP_WARN(this->get_logger(), "No pallet data!");
        goal_handle->abort(result);
        return;
    }

    // 获取雷达下的托盘位姿
    geometry_msgs::msg::Pose2D lidar_pallet;
    lidar_pallet.x = goal->lidar_pallet_pose_x;
    lidar_pallet.y = goal->lidar_pallet_pose_y;
    lidar_pallet.theta = goal->lidar_pallet_pose_theta * M_PI /180.0;

    RCLCPP_INFO(this->get_logger(), "Lidar_pallet info: %f, %f, %f.", lidar_pallet.x, lidar_pallet.y, lidar_pallet.theta * 180 / M_PI);

    gtsam::Pose2 forklift;
    // 根据测量实际情况进行计算坐标系
    // 计算托盘坐标系下，雷达的坐标
    double p_l_x, p_l_y, p_l_theta;
    p_l_x = -(lidar_pallet.y + lidar_pallet.x * tan(lidar_pallet.theta)) * cos(lidar_pallet.theta);
    p_l_y = -(lidar_pallet.x - lidar_pallet.y * tan(lidar_pallet.theta)) * cos(lidar_pallet.theta);
    p_l_theta = lidar_pallet.theta;
    // 计算托盘坐标系下，叉车的坐标
    double p_f_x, p_f_y, p_f_theta;
    p_f_x = p_l_x + delta_x;
    p_f_y = p_l_y + delta_y;
    p_f_theta = p_l_theta;
    forklift = gtsam::Pose2(p_f_x, p_f_y, p_f_theta);

    gtsam::Pose2 pallet;
    // 根据测量实际情况进行计算坐标系
    if (delta_x == 0 && delta_y == 0 && delta_theta == 0) {    // 都为0,即在雷达坐标系下
        pallet = gtsam::Pose2(-lidar_pallet.y, lidar_pallet.x, -lidar_pallet.theta);
    } else {
        Eigen::Matrix3d T_FL = transformMatrix(delta_y, delta_x, delta_theta * M_PI / 180.0);      // 雷达到叉车的转换矩阵
        Eigen::Vector3d P(lidar_pallet.x, lidar_pallet.y, 1);    // 托盘在雷达下的奇次坐标
        // 计算出叉车的坐标中心及托盘在叉车下的坐标
        Eigen::Vector3d P_F = T_FL * P;
        // 由于叉车和雷达坐标系x轴和y轴是反的，故转换即可
        pallet = gtsam::Pose2(-P_F(1, 0), P_F(0, 0), -lidar_pallet.theta + delta_theta * M_PI / 180.0);
    }
    
    RCLCPP_INFO(this->get_logger(), "Start planning...");
    path_ = planner_->PlanPath(pallet, lidar_pallet.y);
    feedback->progress = 0.5;
    goal_handle->publish_feedback(feedback);
    path_pub_->publish(path_);
    RCLCPP_INFO(this->get_logger(), "Planning is over.");

    if (!path_.poses.empty()) {
        // 将计算好的角度返回给客户端
        for (const auto& pose : path_.poses) {
            double angle = planner_->quaternion2Euler(pose.pose.orientation);
            result->angle.push_back(angle);
        }
        // 将生成的路径返回给客户端
        result->path = path_;

        auto last_pose = result->path.poses.back();  // 获取最后一个位置
        double yaw;
        yaw = planner_->quaternion2Euler(last_pose.pose.orientation);
        RCLCPP_INFO(this->get_logger(), "Path generated from (%f, %f, %f) to (%f, %f, %f)", 
                    0.0, 0.0, 0.0,
                    last_pose.pose.position.x, last_pose.pose.position.y, yaw);
        RCLCPP_INFO(this->get_logger(), "The poses: %zu.", result->path.poses.size());

        feedback->progress = 1.0;  // 完成
        goal_handle->publish_feedback(feedback);
        goal_handle->succeed(result);

        RCLCPP_INFO(this->get_logger(), "ForkliftCorrectServer goal execution completed.");
        return;
    } else {
        RCLCPP_WARN(this->get_logger(), "No poses found in the path.");
        return;
    }
}

Eigen::Matrix3d ForkliftCorrectServer::transformMatrix(double x, double y, double theta) {
    Eigen::Matrix3d transform_matrix = Eigen::Matrix3d::Identity();
    // 平移矩阵
    transform_matrix(0, 2) = x;
    transform_matrix(1, 2) = y;
    // 旋转矩阵
    transform_matrix(0, 0) = cos(theta);
    transform_matrix(0, 1) = -sin(theta);
    transform_matrix(1, 0) = sin(theta);
    transform_matrix(1, 1) = cos(theta);

    return transform_matrix;
}

int main(int argc, char**argv) {
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Start Forklift Correct Server.");
    auto node = std::make_shared<ForkliftCorrectServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
