#include <drone_landing_planner/landing_pad_controller.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LandingPadController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}