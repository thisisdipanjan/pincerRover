#include "pincer_rover_base/ackermann_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AckermannNode>());
    rclcpp::shutdown();
    return 0;
}
