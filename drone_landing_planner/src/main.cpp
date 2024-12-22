#include <drone_landing_planner/descend_and_center.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DescendAndCenter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}