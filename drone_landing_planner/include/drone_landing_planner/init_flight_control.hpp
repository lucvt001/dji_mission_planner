#ifndef INIT_FLIGHT_CONTROL_HPP
#define INIT_FLIGHT_CONTROL_HPP

#include <rclcpp/rclcpp.hpp>
#include <psdk_interfaces/srv/obtain_joystick_authority.hpp>
#include <psdk_interfaces/srv/set_joystick_mode.hpp>

using ObtainJoystickAuthority = psdk_interfaces::srv::ObtainJoystickAuthority;
using SetJoystickMode = psdk_interfaces::srv::SetJoystickMode;

bool initialize_flight_control(const std::shared_ptr<rclcpp::Node>& node);

#endif // INIT_FLIGHT_CONTROL_HPP