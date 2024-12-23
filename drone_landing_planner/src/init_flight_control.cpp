#include <drone_landing_planner/init_flight_control.hpp>

bool initialize_flight_control(const std::shared_ptr<rclcpp::Node>& node)
{
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

    RCLCPP_INFO(node->get_logger(), "Intialized Flight control, obtained joystick authority and set joystick mode to velocity control, body frame, stable mode enabled");
    return true;
}