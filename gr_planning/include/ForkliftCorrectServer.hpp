#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <string>
#include <vector>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <gtsam/geometry/Pose2.h>
#include <Eigen/Dense> 

#include "gr_planning_interface/srv/get_path.hpp"
#include "gr_planning_interface/action/get_path.hpp"
#include "../drivers/ForkliftPath/ForkliftPath.cpp"

class ForkliftCorrectServer : public rclcpp::Node {
    public:
        ForkliftCorrectServer();
        ~ForkliftCorrectServer()=default;

        using ForkliftPathAction = gr_planning_interface::action::GetPath;
        using GoalHandleForkliftPath = rclcpp_action::ServerGoalHandle<ForkliftPathAction>;
    private:
        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid,
                                                std::shared_ptr<const ForkliftPathAction::Goal> goal);
        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleForkliftPath> goal_handle);
        void handle_accepted(const std::shared_ptr<GoalHandleForkliftPath> goal_handle);
        void planning(const std::shared_ptr<GoalHandleForkliftPath> goal_handle);

        rclcpp_action::Server<ForkliftPathAction>::SharedPtr service_;

        std::string plan_topic_;

        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

        std::unique_ptr<ForkliftPath> planner_;
        // nav_msgs::msg::Path path_;

        Eigen::Matrix3d transformMatrix(double x, double y, double theta);
        // 雷达偏移量
        double delta_x, delta_y, delta_theta;
};