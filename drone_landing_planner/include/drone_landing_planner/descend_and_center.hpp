#ifndef LANDING_PAD_CONTROLLER_HPP
#define LANDING_PAD_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <psdk_interfaces/msg/velocity_command.hpp>
#include <psdk_interfaces/srv/obtain_joystick_authority.hpp>
#include <psdk_interfaces/srv/release_joystick_authority.hpp>
#include <psdk_interfaces/srv/set_joystick_mode.hpp>
#include <pid_package/pid.h>

using ObtainJoystickAuthority = psdk_interfaces::srv::ObtainJoystickAuthority;
using ReleaseJoystickAuthority = psdk_interfaces::srv::ReleaseJoystickAuthority;
using SetJoystickMode = psdk_interfaces::srv::SetJoystickMode;
using VelocityCommand = psdk_interfaces::msg::VelocityCommand;
using Point = geometry_msgs::msg::Point;

class DescendAndCenter : public rclcpp::Node
{
public:
    DescendAndCenter();
    ~DescendAndCenter();

private:
    void positionCallback(const Point::SharedPtr msg);
    void retrievePidParameters();
    void initializeDJIFlightControl();

    rclcpp::Subscription<Point>::SharedPtr landing_pad_position_sub_;
    rclcpp::Publisher<VelocityCommand>::SharedPtr velocity_pub_;

    rclcpp::Client<ObtainJoystickAuthority>::SharedPtr obtain_joystick_authority_client_;
    rclcpp::Client<ReleaseJoystickAuthority>::SharedPtr release_joystick_authority_client_;
    rclcpp::Client<SetJoystickMode>::SharedPtr set_joystick_mode_client_;

    std::string position_topic_;
    std::string velocity_command_topic_;
    std::string obtain_joystick_authority_server_;
    std::string release_joystick_authority_server_;
    std::string set_joystick_mode_server_;

    // In the camera frame when it is looking down
    // positive x is to the right
    // positive y is backwards
    // positive z is up
    PID x_pid_ = PID(0,0,0,0,0,0);
    PID y_pid_ = PID(0,0,0,0,0,0);
    PID z_pid_ = PID(0,0,0,0,0,0);
    float z_setpoint_;
};

#endif // LANDING_PAD_CONTROLLER_HPP