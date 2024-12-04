#include "ForkliftPath.hpp"

ForkliftPath::ForkliftPath(){
    pos_ = std::make_shared<PlanOutput>();
}

nav_msgs::msg::Path ForkliftPath::PlanPath(gtsam::Pose2 input_pallet, double init_dist) {
    // 创建path消息
    nav_msgs::msg::Path path_msg;
    // 路径可视化
    path_msg.header.frame_id = "world";
    path_msg.header.stamp = rclcpp::Clock().now();    

    // gtsam::Pose2 input_pallet(0, 0, 0);
    gtsam::Pose2 input_forklift(0, 0, 0);
    // 设置叉车和托盘的位置与角度
    forklift.setPosition(input_forklift.x(), input_forklift.y());
    forklift.setAngle(input_forklift.theta() * 180 / M_PI);
    pallet.setPosition(input_pallet.x(), input_pallet.y());
    pallet.setAngle(input_pallet.theta() * 180 / M_PI);
    // 将起始点添加进路径
    path_msg.poses.push_back(Output2Pose(forklift));

    // 角度范围
    // forklift.angle_ = normalizeAngle(forklift.angle_);
    // pallet.angle_ = normalizeAngle(pallet.angle_);

    pos_->setAngle(pallet.angle_);
    pos_->setPosition(pallet.x_ + l * cos(pallet.angle_ * M_PI / 180.0f), pallet.y_ + l * sin(pallet.angle_ * M_PI / 180.0f));

    RCLCPP_INFO(rclcpp::get_logger("ForkliftPath"), "Forklift pose: %f, %f, %f",
                forklift.x_, forklift.y_, forklift.angle_ );
    RCLCPP_INFO(rclcpp::get_logger("ForkliftPath"), "Pallet pose: %f, %f, %f",
                pallet.x_, pallet.y_, pallet.angle_);
    RCLCPP_INFO(rclcpp::get_logger("ForkliftPath"), "Goal pose: %f, %f, %f",
                pos_->x_, pos_->y_, pos_->angle_);

    double turn_angle = 0;
    int stop = 1, step = 1, i = 0;
    while (stop==1) {
        plan(forklift, pos_, turn_angle, step, stop, init_dist);
        path_msg.poses.push_back(Output2Pose(forklift));
        i += 1;
    }

    return path_msg;
}

// 规划纠偏路径
void ForkliftPath::plan(PlanOutput& forklift, std::shared_ptr<PlanOutput> target,
                        double& turn_angle, int& step, int& stop, double dist) {
    double distance_per_t = backward / t;

    if (step == 1) {
        PlanOutput pre_forklift;
        pre_forklift.setPosition(forklift.x_ + backward * cos(forklift.angle_ * M_PI / 180.0f), forklift.y_ + backward * sin(forklift.angle_ * M_PI / 180.0f));
        pre_forklift.setAngle(atan2(pre_forklift.y_ - target->y_, pre_forklift.x_ - target->x_));
        // double init_angle = atan2(forklift.y_ - target->y_, forklift.x_ - target->x_);
        double init_angle = atan2(target->y_, target->x_);
        init_angle = normalizeAngle(init_angle);
        std::cout<<"init_angle:" << init_angle * 180.0f / M_PI << std::endl;
        pre_forklift.angle_ = normalizeAngle(pre_forklift.angle_);

        std::cout<<"pre_angle:" << pre_forklift.angle_ * 180.0f / M_PI << std::endl;
        
        if (abs(pre_forklift.angle_) < abs(init_angle) && abs(init_angle) > 30 * M_PI / 180.0f || dist < 2.01) {
            forklift.x_ += backward * cos(forklift.angle_ * M_PI / 180.0f);
            forklift.y_ += backward * sin(forklift.angle_ * M_PI / 180.0f);
            turn_angle = pre_forklift.angle_;
        } else {
            turn_angle = init_angle;
        }

        step = 2;
    } else if (step == 2) { // 转弯至目标点
        forklift.angle_ = turn_angle * 180.0f / M_PI;

        step = 3;
    } else if (step == 3) { // 行驶至目标点
        forklift.x_ = target->x_;
        forklift.y_ = target->y_;

        step = 4;
    } else if (step == 4) { // 旋转至目标位置
        forklift.angle_ = target->angle_;
        
        step = 0;
        stop = 0;
    }
}

// 输出为PoseStamped类型
geometry_msgs::msg::PoseStamped ForkliftPath::Output2Pose(const PlanOutput position) {
    // 位置
    geometry_msgs::msg::PoseStamped pose_;
    pose_.pose.position.x = position.x_;
    pose_.pose.position.y = position.y_;
    // 方向
    tf2::Quaternion q;
    q.setRPY(0, 0, position.angle_ * M_PI / 180.0f);
    pose_.pose.orientation.x = q.x();
    pose_.pose.orientation.y = q.y();
    pose_.pose.orientation.z = q.z();
    pose_.pose.orientation.w = q.w();

    return pose_;
}

// 用于将四元数转换为欧拉角，便于计算，返回的角度单位为度数
double ForkliftPath::quaternion2Euler(geometry_msgs::msg::Quaternion orientation) {
        tf2::Quaternion tf2_quat;
        tf2::fromMsg(orientation, tf2_quat);

        double roll, pitch, yaw;

        tf2::Matrix3x3(tf2_quat).getRPY(roll, pitch, yaw);
        return yaw * 180.0f / M_PI;
}

double ForkliftPath::normalizeAngle(double angle) {
    // if (angle > 180) angle -= 360.0f;
    // if (angle < -180) angle += 360.0f;
    if (angle > M_PI/2) angle -= M_PI;
    if (angle < -M_PI/2) angle += M_PI;

    return angle;
}