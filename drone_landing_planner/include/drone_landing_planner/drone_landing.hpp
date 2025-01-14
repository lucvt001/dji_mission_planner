#ifndef LANDING_PAD_CONTROLLER_HPP
#define LANDING_PAD_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <psdk_interfaces/msg/velocity_command.hpp>
#include <psdk_interfaces/srv/obtain_joystick_authority.hpp>
#include <psdk_interfaces/srv/set_joystick_mode.hpp>
#include <dji_mission_interfaces/action/land_on_pad.hpp>
#include <pid_package/pid.h>

using Point = geometry_msgs::msg::Point;
using VelocityCommand = psdk_interfaces::msg::VelocityCommand;
using ObtainJoystickAuthority = psdk_interfaces::srv::ObtainJoystickAuthority;
using SetJoystickMode = psdk_interfaces::srv::SetJoystickMode;
using LandOnPad = dji_mission_interfaces::action::LandOnPad;
using namespace std;
using namespace rclcpp_action;

class DroneLander : public rclcpp::Node
{
public:
    /*An action server that subscribes to landing_pad_position (in meters, relative to camera)
    Compute the PID output for x, y axes for the drone based on this position
    Compute the PID output for z axis based on vertical_setpoint_ (settable from param file)
    Upon centering and reaching sufficient height, it will activate PSDK land function.*/
    DroneLander();

private:

    rclcpp::Publisher<VelocityCommand>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<Point>::SharedPtr landing_pad_position_sub_;

    // All of the PID compute and checking if target is reached is done in this callback
    void positionCallback(const Point::SharedPtr msg);

    Server<LandOnPad>::SharedPtr land_on_pad_action_server_;

    // This action callback is only for checking if the goal is cancelled
    void execute(const std::shared_ptr<ServerGoalHandle<LandOnPad>> goal_handle);
    bool is_action_called_ = false;
    bool is_completed_ = false;

    rclcpp::Client<ObtainJoystickAuthority>::SharedPtr obtain_joystick_authority_client_;
    rclcpp::Client<SetJoystickMode>::SharedPtr set_joystick_mode_client_;
    void obtainJoystickAuthority();
    void setJoystickMode();     // Set joystick mode to velocity control

    // In the camera frame when it is looking down
    // positive x is to the right
    // positive y is backwards
    // positive z is up
    void initPID(const string &axis, PID &pid);
    PID x_pid_ = PID(0, 0, 0, 0, 0, 0); // Forward
    PID y_pid_ = PID(0, 0, 0, 0, 0, 0); // Right
    PID z_pid_ = PID(0, 0, 0, 0, 0, 0); // Down
    float vertical_setpoint_;

    float horizontal_deviation_threshold_;
    float vertical_deviation_threshold_;
    std::optional<std::chrono::steady_clock::time_point> prev_goal_reached_time_;
};

#endif // LANDING_PAD_CONTROLLER_HPP