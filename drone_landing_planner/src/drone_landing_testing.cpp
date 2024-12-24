#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <psdk_interfaces/action/take_off.hpp>
#include <psdk_interfaces/action/move_to_position.hpp>
#include <psdk_interfaces/action/land.hpp>
#include <psdk_interfaces/srv/obtain_joystick_authority.hpp>
#include <psdk_interfaces/srv/set_joystick_mode.hpp>
#include <psdk_interfaces/srv/set_gimbal_angle.hpp>
#include <drone_landing_planner/init_flight_control.hpp>
#include <drone_landing_planner/descend_and_center.hpp>

using TakeOff = psdk_interfaces::action::TakeOff;
using Land = psdk_interfaces::action::Land;
using MoveToPosition = psdk_interfaces::action::MoveToPosition;
using ObtainJoystickAuthority = psdk_interfaces::srv::ObtainJoystickAuthority;
using SetJoystickMode = psdk_interfaces::srv::SetJoystickMode;
using SetGimbalAngle = psdk_interfaces::srv::SetGimbalAngle;
using DescendAndCenterAction = dji_mission_interfaces::action::DescendAndCenter;

void handle_descend_and_center_result(const rclcpp_action::ClientGoalHandle<DescendAndCenterAction>::WrappedResult & result)
{
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "DescendAndCenter action succeeded");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "DescendAndCenter action was aborted");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "DescendAndCenter action was canceled");
            break;
        default:
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Unknown result code");
            break;
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("drone_landing_testing");
    
    std::string obtain_joystick_authority_server_;
    std::string set_joystick_mode_server_;

    node->declare_parameter<std::string>("obtain_joystick_authority_server", "/psdk_wrapper_node/obtain_joystick_authority_service");
    node->declare_parameter<std::string>("set_joystick_mode_server", "/psdk_wrapper_node/set_joystick_mode_service");
    node->get_parameter("obtain_joystick_authority_server", obtain_joystick_authority_server_);
    node->get_parameter("set_joystick_mode_server", set_joystick_mode_server_);

    // Create service clients
    auto obtain_authority_client = node->create_client<ObtainJoystickAuthority>(obtain_joystick_authority_server_);
    auto set_joystick_mode_client = node->create_client<SetJoystickMode>(set_joystick_mode_server_);

    // Wait for the services to be available
    if (!obtain_authority_client->wait_for_service(std::chrono::seconds(10))) {
        RCLCPP_ERROR(node->get_logger(), "ObtainJoystickAuthority service not available");
        return false;
    }

    if (!set_joystick_mode_client->wait_for_service(std::chrono::seconds(10))) {
        RCLCPP_ERROR(node->get_logger(), "SetJoystickMode service not available");
        return false;
    }

    // Create request objects
    auto obtain_authority_request = std::make_shared<ObtainJoystickAuthority::Request>();
    auto obtain_authority_result = obtain_authority_client->async_send_request(obtain_authority_request);
    if (rclcpp::spin_until_future_complete(node, obtain_authority_result) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "Failed to call ObtainJoystickAuthority service");
        return false;
    }

    // Check the result of ObtainJoystickAuthority service
    if (!obtain_authority_result.get()->is_success) {
        RCLCPP_ERROR(node->get_logger(), "ObtainJoystickAuthority service call failed");
        return false;
    }
    

    // Create clients for the services and actions
    auto take_off_action_client = rclcpp_action::create_client<TakeOff>(node, "/psdk_wrapper_node/takeoff_action");
    auto move_to_position_action_client = rclcpp_action::create_client<MoveToPosition>(node, "/psdk_wrapper_node/move_to_position_action");
    auto descend_and_center_action_client = rclcpp_action::create_client<DescendAndCenterAction>(node, "/descend_and_center_node/descend_and_center_action");

    // Take off and move to position
    auto take_off_goal = TakeOff::Goal();
    auto take_off_send_goal_options = rclcpp_action::Client<TakeOff>::SendGoalOptions();
    auto future_result_take_off = take_off_action_client->async_send_goal(take_off_goal, take_off_send_goal_options);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "TakeOff action sent");
    // Wait for 5 seconds for takeoff to complete
    // This should not be the way for dealing with actions
    // It is just a lazy way to make sure the takeoff action is completed before sending the move to position action
    // The correct way is to use BehaviorTree.ROS2
    std::this_thread::sleep_for(std::chrono::seconds(10));   
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Assume that TakeOff action completed");


    auto move_to_position_goal = MoveToPosition::Goal();
    move_to_position_goal.x = 0.0;
    move_to_position_goal.y = 0.0;
    move_to_position_goal.z = 5.0;
    move_to_position_goal.timeout = 10.0;   // 15 second
    auto move_to_position_send_goal_options = rclcpp_action::Client<MoveToPosition>::SendGoalOptions();
    auto future_result_move_to_position = move_to_position_action_client->async_send_goal(move_to_position_goal, move_to_position_send_goal_options);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "MoveToPosition action sent");
    std::this_thread::sleep_for(std::chrono::seconds(10));   

    // Set the desired joystick mode 
    auto set_joystick_mode_request = std::make_shared<SetJoystickMode::Request>();
    set_joystick_mode_request->horizontal_control_mode = 1; // velocity control
    set_joystick_mode_request->vertical_control_mode = 0;   // velocity control
    set_joystick_mode_request->yaw_control_mode = 0;        // yaw angle control
    set_joystick_mode_request->horizontal_coordinate = 1;   // body frame
    set_joystick_mode_request->stable_control_mode = 1;     // stable mode enabled

    // Call SetJoystickMode service
    auto set_joystick_mode_result = set_joystick_mode_client->async_send_request(set_joystick_mode_request);
    if (rclcpp::spin_until_future_complete(node, set_joystick_mode_result) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "Failed to call SetJoystickMode service");
        return false;
    }

    // Check the result of SetJoystickMode service
    if (!set_joystick_mode_result.get()->is_success) {
        RCLCPP_ERROR(node->get_logger(), "SetJoystickMode service call failed");
        return false;
    }

    auto set_gimbal_client = node->create_client<SetGimbalAngle>("/psdk_wrapper_node/set_gimbal_angle_service");
    auto set_gimbal_request = std::make_shared<SetGimbalAngle::Request>();
    set_gimbal_request->pitch = -90.0;
    set_gimbal_request->roll = 0.0;
    set_gimbal_request->yaw = 0.0;
    auto set_gimbal_result = set_gimbal_client->async_send_request(set_gimbal_request);
    if (rclcpp::spin_until_future_complete(node, set_gimbal_result) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "Failed to call SetGimbalAngle service");
        return false;
    }

    // Descend and center
    auto descend_and_center_goal = DescendAndCenterAction::Goal();
    descend_and_center_goal.timeout = 20;
    auto descend_and_center_send_goal_options = rclcpp_action::Client<DescendAndCenterAction>::SendGoalOptions();
    auto goal_handle_descend_and_center_future = descend_and_center_action_client->async_send_goal(descend_and_center_goal, descend_and_center_send_goal_options);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "DescendAndCenter action sent");

    if (rclcpp::spin_until_future_complete(node, goal_handle_descend_and_center_future) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "Send goal to DescendAndCenter failed:(");
        rclcpp::shutdown();
        return 1;
    }

    auto goal_handle_descend_and_center = goal_handle_descend_and_center_future.get();
    if (!goal_handle_descend_and_center) {
        RCLCPP_ERROR(node->get_logger(), "Goal was rejected by the action server");
        rclcpp::shutdown();
        return 1;
    }
    auto result_future_descend_and_center = descend_and_center_action_client->async_get_result(goal_handle_descend_and_center);

    RCLCPP_INFO(node->get_logger(), "Waiting for result");
    if (rclcpp::spin_until_future_complete(node, result_future_descend_and_center) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "get result call failed :(");
        return 1;
    }

    auto wrapped_result = result_future_descend_and_center.get();

    if (wrapped_result.result->error_code == 0) {
        RCLCPP_INFO(node->get_logger(), "DescendAndCenter action succeeded. Landing now..");
    } else if (wrapped_result.result->error_code == 1) {
        RCLCPP_ERROR(node->get_logger(), "DescendAndCenter action timeout. Trying to land if possible..");
    }
    
    auto land_action_client = rclcpp_action::create_client<Land>(node, "/psdk_wrapper_node/land_action");
    auto land_goal = Land::Goal();
    auto land_send_goal_options = rclcpp_action::Client<Land>::SendGoalOptions();
    auto future_result_land = land_action_client->async_send_goal(land_goal, land_send_goal_options);

    if (rclcpp::spin_until_future_complete(node, future_result_land) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "Failed to send goal to Land action");
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}