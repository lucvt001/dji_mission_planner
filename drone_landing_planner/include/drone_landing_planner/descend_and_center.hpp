#ifndef LANDING_PAD_CONTROLLER_HPP
#define LANDING_PAD_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <psdk_interfaces/msg/velocity_command.hpp>
#include <dji_mission_interfaces/action/descend_and_center.hpp>
#include <pid_package/pid.h>

using VelocityCommand = psdk_interfaces::msg::VelocityCommand;
using Point = geometry_msgs::msg::Point;
using DescendAndCenterAction = dji_mission_interfaces::action::DescendAndCenter;

class DescendAndCenter : public rclcpp::Node
{
public:
    DescendAndCenter();

private:
    void positionCallback(const Point::SharedPtr msg);
    void retrievePidParameters();
    void initializeDJIFlightControl();

    rclcpp_action::Server<DescendAndCenterAction>::SharedPtr descend_and_center_action_server_;
    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<DescendAndCenterAction>> goal_handle);
    bool is_action_called_ = false;
    bool is_completed_ = false;

    rclcpp::Subscription<Point>::SharedPtr landing_pad_position_sub_;
    rclcpp::Publisher<VelocityCommand>::SharedPtr velocity_pub_;

    std::string position_topic_;
    std::string velocity_command_topic_;

    // In the camera frame when it is looking down
    // positive x is to the right
    // positive y is backwards
    // positive z is up
    PID x_pid_ = PID(0,0,0,0,0,0);
    PID y_pid_ = PID(0,0,0,0,0,0);
    PID z_pid_ = PID(0,0,0,0,0,0);
    float vertical_setpoint_;

    float horizontal_deviation_threshold_;
    float vertical_deviation_threshold_;
    std::optional<std::chrono::steady_clock::time_point> prev_goal_reached_time_;
};

#endif // LANDING_PAD_CONTROLLER_HPP