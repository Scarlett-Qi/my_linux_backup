#include "ForkliftCorrectNode.hpp"

ForkliftCorrectNode::ForkliftCorrectNode() : Node("gr_planning_forklift_correct_node") {

}



int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ForkliftCorrectNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
