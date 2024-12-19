#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include <payload_sdk_ros2_interfaces/srv/obtain_joystick_authority.hpp>
#include <payload_sdk_ros2_interfaces/srv/release_joystick_authority.hpp>
#include <payload_sdk_ros2_interfaces/srv/set_joystick_mode.hpp>
#include <payload_sdk_ros2_interfaces/action/take_off.hpp>
#include <payload_sdk_ros2_interfaces/action/move_to_position.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("initial_testing");

    auto obtain_joystick_authority_client = node->create_client<payload_sdk_ros2_interfaces::srv::ObtainJoystickAuthority>("/psdk_wrapper_node/obtain_joystick_authority_service");
    auto release_joystick_authority_client = node->create_client<payload_sdk_ros2_interfaces::srv::ReleaseJoystickAuthority>("/psdk_wrapper_node/release_joystick_authority_service");
    auto set_joystick_mode_client = node->create_client<payload_sdk_ros2_interfaces::srv::SetJoystickMode>("/psdk_wrapper_node/set_joystick_mode_service");

    auto take_off_action_client = rclcpp_action::create_client<payload_sdk_ros2_interfaces::action::TakeOff>(node, "/psdk_wrapper_node/take_off");
    auto move_to_position_action_client = rclcpp_action::create_client<payload_sdk_ros2_interfaces::action::MoveToPosition>(node, "/psdk_wrapper_node/move_to_position");

    auto obtain_joystick_authority_request = std::make_shared<payload_sdk_ros2_interfaces::srv::ObtainJoystickAuthority::Request>();
    auto obtain_joystick_authority_result = obtain_joystick_authority_client->async_send_request(obtain_joystick_authority_request);
    if (rclcpp::spin_until_future_complete(node->get_node_base_interface(), obtain_joystick_authority_result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        if (obtain_joystick_authority_result.get()->is_success)
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Obtained joystick authority");
        else
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to obtain joystick authority");
    } else
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to obtain joystick authority");


    auto set_joystick_mode_request = std::make_shared<payload_sdk_ros2_interfaces::srv::SetJoystickMode::Request>();
    set_joystick_mode_request->horizontal_control_mode = 2; // position control
    set_joystick_mode_request->vertical_control_mode = 1;   // position control
    set_joystick_mode_request->yaw_control_mode = 0;        // yaw angle control
    set_joystick_mode_request->horizontal_coordinate = 0;   // ground frame
    set_joystick_mode_request->stable_control_mode = 1;     // stable mode enabled

    auto set_joystick_mode_result = set_joystick_mode_client->async_send_request(set_joystick_mode_request);
    if (rclcpp::spin_until_future_complete(node, set_joystick_mode_result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        if (set_joystick_mode_result.get()->is_success)
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Set joystick mode successfully");
        else
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to set joystick mode");
    } else
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to set joystick mode");

    auto take_off_goal = payload_sdk_ros2_interfaces::action::TakeOff::Goal();
    auto send_goal_options = rclcpp_action::Client<payload_sdk_ros2_interfaces::action::TakeOff>::SendGoalOptions();
    // send_goal_options.goal_response_callback = [](std::shared_future<rclcpp_action::ClientGoalHandle<payload_sdk_ros2_interfaces::action::TakeOff>::SharedPtr> future) {
    //     auto goal_handle = future.get();
    //     if (!goal_handle) {
    //         RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "TakeOff goal was rejected by server");
    //     } else {
    //         RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "TakeOff goal accepted by server, waiting for result");
    //     }
    // };
    send_goal_options.result_callback = [](const rclcpp_action::ClientGoalHandle<payload_sdk_ros2_interfaces::action::TakeOff>::WrappedResult &result) {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "TakeOff succeeded");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "TakeOff was aborted");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "TakeOff was canceled");
                break;
            default:
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Unknown result code");
                break;
        }
    };

    take_off_action_client->async_send_goal(take_off_goal, send_goal_options);

    rclcpp::shutdown();
    return 0;
}