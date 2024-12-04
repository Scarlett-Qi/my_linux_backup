#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

class ForkliftCorrectNode : public rclcpp::Node {
  public:
    ForkliftCorrectNode();
    ~ForkliftCorrectNode()=default;

  private:

    void TargetCallback();

    // 发布规划轨迹

    // 订阅目标位置
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr target_sub_;

    // 订阅当前定位

};

