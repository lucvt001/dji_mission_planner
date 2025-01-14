#ifndef FOLLOW_RELATIVE_NED_HPP
#define FOLLOW_RELATIVE_NED_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <psdk_interfaces/msg/velocity_command.hpp>
#include <psdk_interfaces/srv/obtain_joystick_authority.hpp>
#include <psdk_interfaces/srv/set_joystick_mode.hpp>
#include <dji_mission_interfaces/action/follow_vehicle.hpp>
#include <pid_package/pid.h>
#include "gps_to_ned.hpp"

using Float32 = std_msgs::msg::Float32;
using Point = geometry_msgs::msg::Point;
using NavSatFix = sensor_msgs::msg::NavSatFix;
using VelocityCommand = psdk_interfaces::msg::VelocityCommand;
using ObtainJoystickAuthority = psdk_interfaces::srv::ObtainJoystickAuthority;
using SetJoystickMode = psdk_interfaces::srv::SetJoystickMode;
using FollowVehicle = dji_mission_interfaces::action::FollowVehicle;
using namespace std;
using namespace rclcpp_action;

class VehicleFollower : public rclcpp::Node
{
public:
    /*An action server that subscribes to car_gps and drone_gps
    Compute the relative NED position from car to drone
    Compute the PID output based on this relative position
    Yaw and height control can be done separately with data from topics yaw_control and height_control.*/
    VehicleFollower();

private:

    rclcpp::Subscription<NavSatFix>::SharedPtr car_gps_sub_;
    rclcpp::Subscription<NavSatFix>::SharedPtr drone_gps_sub_;
    rclcpp::Subscription<Float32>::SharedPtr yaw_control_sub_;
    rclcpp::Subscription<Float32>::SharedPtr height_control_sub_;
    rclcpp::Publisher<VelocityCommand>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<Point>::SharedPtr car_ned_pub_;   // For logging purpose

    NavSatFix car_gps_data_;
    NavSatFix drone_gps_data_;
    float yaw_control_data_ = 0;
    float height_control_data_ = 0;

    rclcpp::Client<ObtainJoystickAuthority>::SharedPtr obtain_joystick_authority_client_;
    rclcpp::Client<SetJoystickMode>::SharedPtr set_joystick_mode_client_;
    void obtainJoystickAuthority();
    void setJoystickMode();     // Set joystick mode to velocity control

    Server<FollowVehicle>::SharedPtr follow_vehicle_action_server_;

    // This is a long-running action that will always be RUNNING unless interrupted
    // Before starting the control loop, the drone will obtain joystick authority and set joystick mode automatically
    // Each time, it will publish the feedback horizontal_distance
    void execute(const shared_ptr<ServerGoalHandle<FollowVehicle>> goal_handle);

    // A very important flag to ensure that we are always processing new messages in each loop
    bool is_new_msg_received_ = false;

    void initPID(const string &axis, PID &pid);
    PID x_pid_ = PID(0, 0, 0, 0, 0, 0); // North
    PID y_pid_ = PID(0, 0, 0, 0, 0, 0); // East
    PID z_pid_ = PID(0, 0, 0, 0, 0, 0); // Down
};

#endif // FOLLOW_RELATIVE_NED_HPP
