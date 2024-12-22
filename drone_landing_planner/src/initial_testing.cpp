#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <psdk_interfaces/srv/obtain_joystick_authority.hpp>
#include <psdk_interfaces/srv/release_joystick_authority.hpp>
#include <psdk_interfaces/srv/set_joystick_mode.hpp>
#include <psdk_interfaces/action/take_off.hpp>
#include <psdk_interfaces/action/move_to_position.hpp>

using ObtainJoystickAuthority = psdk_interfaces::srv::ObtainJoystickAuthority;
using ReleaseJoystickAuthority = psdk_interfaces::srv::ReleaseJoystickAuthority;
using SetJoystickMode = psdk_interfaces::srv::SetJoystickMode;
using TakeOff = psdk_interfaces::action::TakeOff;
using MoveToPosition = psdk_interfaces::action::MoveToPosition;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("initial_testing");

    // Create clients for the services and actions
    auto obtain_joystick_authority_client = node->create_client<ObtainJoystickAuthority>("/psdk_wrapper_node/obtain_joystick_authority_service");
    auto release_joystick_authority_client = node->create_client<ReleaseJoystickAuthority>("/psdk_wrapper_node/release_joystick_authority_service");
    auto set_joystick_mode_client = node->create_client<SetJoystickMode>("/psdk_wrapper_node/set_joystick_mode_service");

    auto take_off_action_client = rclcpp_action::create_client<TakeOff>(node, "/psdk_wrapper_node/takeoff_action");
    auto move_to_position_action_client = rclcpp_action::create_client<MoveToPosition>(node, "/psdk_wrapper_node/move_to_position_action");

    // Obtain joystick authority
    auto obtain_joystick_authority_request = std::make_shared<ObtainJoystickAuthority::Request>();
    auto obtain_joystick_authority_result = obtain_joystick_authority_client->async_send_request(obtain_joystick_authority_request);
    if (rclcpp::spin_until_future_complete(node->get_node_base_interface(), obtain_joystick_authority_result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        if (obtain_joystick_authority_result.get()->is_success)
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Obtained joystick authority");
        else
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to obtain joystick authority");
    } else
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to obtain joystick authority");

    // Set joystick mode
    auto set_joystick_mode_request = std::make_shared<SetJoystickMode::Request>();
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

    // Take off and move to position
    auto take_off_goal = TakeOff::Goal();
    auto take_off_send_goal_options = rclcpp_action::Client<TakeOff>::SendGoalOptions();
    auto future_result_take_off = take_off_action_client->async_send_goal(take_off_goal, take_off_send_goal_options);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "TakeOff action sent");
    // Wait for 5 seconds for takeoff to complete
    // This should not be the way for dealing with actions
    // It is just a lazy way to make sure the takeoff action is completed before sending the move to position action
    // The correct way is to use BehaviorTree.ROS2
    std::this_thread::sleep_for(std::chrono::seconds(5));   
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Assume that TakeOff action completed");


    auto move_to_position_goal = MoveToPosition::Goal();
    move_to_position_goal.x = 0.0;
    move_to_position_goal.y = 0.0;
    move_to_position_goal.z = 10.0;
    move_to_position_goal.timeout = 15.0;   // 15 second
    auto move_to_position_send_goal_options = rclcpp_action::Client<MoveToPosition>::SendGoalOptions();
    auto future_result_move_to_position = move_to_position_action_client->async_send_goal(move_to_position_goal, move_to_position_send_goal_options);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "MoveToPosition action sent");
    std::this_thread::sleep_for(std::chrono::seconds(15));   

    rclcpp::shutdown();
    return 0;
}