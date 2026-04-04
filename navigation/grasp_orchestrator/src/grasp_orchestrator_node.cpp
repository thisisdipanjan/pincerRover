#include <grasp_orchestrator/grasp_orchestrator.hpp>

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("grasp_orchestrator");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}