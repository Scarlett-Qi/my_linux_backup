#pragma once

#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include <cmath>
#include <random>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <gtsam/geometry/Pose2.h>

#include "../include/PlanOutput.hpp"


class ForkliftPath {
    public:
        ForkliftPath();
        ~ForkliftPath()=default;

        nav_msgs::msg::Path PlanPath(gtsam::Pose2 input_pallet, double init_dist);

        double quaternion2Euler(geometry_msgs::msg::Quaternion orientation);

    private:
        void plan(PlanOutput& forklift, std::shared_ptr<PlanOutput> target, double& turn_angle, int& step, int& stop, double dist);
        
        PlanOutput forklift, pallet;

        // // 创建path消息
        // nav_msgs::msg::Path path_msg;

        std::shared_ptr<PlanOutput> pos_;
        geometry_msgs::msg::PoseStamped Output2Pose(const PlanOutput position);

        double normalizeAngle(double angle);    // 将角度转化为坐标系角度
        double backward = 1, t = 20;            // t 秒后退 backward 米
        double l = 2;                         // 设置目标点距离托盘位置，单位 米
};