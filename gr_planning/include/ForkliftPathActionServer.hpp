#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "mima_agv_services/action/get_pallet_local_location.hpp"
#include "ridar_service_interface/srv/detect_point_cloud.hpp"
#include "gr_planning_interface/srv/get_path.hpp"
#include "gr_planning_interface/action/get_path.hpp"

class ForkliftActionServer : public rclcpp::Node {
    public:
        ForkliftActionServer();
        ~ForkliftActionServer()=default;

        using GetPalletLocal = mima_agv_services::action::GetPalletLocalLocation;
        using GoalHandleGetPalletLocal = rclcpp_action::ServerGoalHandle<GetPalletLocal>;
    private:
        rclcpp_action::Server<GetPalletLocal>::SharedPtr action_server_;                            // action服务
        rclcpp::Client<ridar_service_interface::srv::DetectPointCloud>::SharedPtr detect_client_;   // 检测客户端
        rclcpp_action::Client<gr_planning_interface::action::GetPath>::SharedPtr path_client_;      // 路径规划客户端

        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const GetPalletLocal::Goal> goal);
        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleGetPalletLocal> goal_handle);
        void handle_accepted(const std::shared_ptr<GoalHandleGetPalletLocal> goal_handle);

        void execute(const std::shared_ptr<GoalHandleGetPalletLocal> goal_handle);
};
