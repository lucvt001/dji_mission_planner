#include <drone_landing_planner/landing_pad_controller.hpp>

LandingPadController::LandingPadController() : Node("landing_pad_controller")
{
    this->declare_parameter<std::string>("landing_pad_position_topic", "/landing_pad_position");
    this->declare_parameter<std::string>("velocity_command_topic", "/drone_velocity_command");
    this->declare_parameter<std::string>("obtain_joystick_authority_server", "/obtain_joystick_authority_service");
    this->declare_parameter<std::string>("release_joystick_authority_server", "/release_joystick_authority_service");
    this->declare_parameter<std::string>("set_joystick_mode_server", "/set_joystick_mode_service");

    this->get_parameter("landing_pad_position_topic", position_topic_);
    this->get_parameter("velocity_command_topic", velocity_command_topic_);
    this->get_parameter("obtain_joystick_authority_server", obtain_joystick_authority_server_);
    this->get_parameter("release_joystick_authority_server", release_joystick_authority_server_);
    this->get_parameter("set_joystick_mode_server", set_joystick_mode_server_);

    landing_pad_position_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
        position_topic_, 10, std::bind(&LandingPadController::positionCallback, this, std::placeholders::_1));

    velocity_pub_ = this->create_publisher<psdk_interfaces::msg::VelocityCommand>(velocity_command_topic_, 10);

    obtain_joystick_authority_client_ = this->create_client<psdk_interfaces::srv::ObtainJoystickAuthority>(obtain_joystick_authority_server_);
    release_joystick_authority_client_ = this->create_client<psdk_interfaces::srv::ReleaseJoystickAuthority>(release_joystick_authority_server_);
    set_joystick_mode_client_ = this->create_client<psdk_interfaces::srv::SetJoystickMode>(set_joystick_mode_server_);

    retrievePidParameters();
    this->declare_parameter<float>("z_setpoint", 0.7);
    this->get_parameter("z_setpoint", z_setpoint_);
    std::cout << "Z setpoint: " << z_setpoint_ << std::endl;

    initializeDJIFlightControl();
}

void LandingPadController::positionCallback(const geometry_msgs::msg::Point::SharedPtr msg)
{
    auto velocity_command = psdk_interfaces::msg::VelocityCommand();

    // Calculate pid from (setpoint, current)
    // For velocity command, the coordinate system in body frame is Forward-Right-Down (FRD), as per PSDK documentation
    // It differs from the camera frame which is Right-Backward-Down, so we have to be careful
    // Error is always setpoint - current, so you can check for the signs of the error to see if the drone is moving in the correct direction
    float x_deviation_camera_frame = (msg->x - 0.5) * msg->z;
    float y_deviation_camera_frame = (msg->y - 0.5) * msg->z;

    velocity_command.x = x_pid_.calculate(0, y_deviation_camera_frame);
    velocity_command.y = y_pid_.calculate(0, x_deviation_camera_frame);
    velocity_command.z = z_pid_.calculate(z_setpoint_, msg->z);
    velocity_command.yaw = 0.0;

    velocity_pub_->publish(velocity_command);
    RCLCPP_INFO(this->get_logger(), "Foward: %f, Right: %f, Up: %f", velocity_command.x, velocity_command.y, velocity_command.z);
}

void LandingPadController::initializeDJIFlightControl()
{
    auto obtain_joystick_authority_request = std::make_shared<psdk_interfaces::srv::ObtainJoystickAuthority::Request>();
    auto obtain_joystick_authority_result = obtain_joystick_authority_client_->async_send_request(obtain_joystick_authority_request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), obtain_joystick_authority_result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        if (obtain_joystick_authority_result.get()->is_success)
            RCLCPP_INFO(this->get_logger(), "Obtained joystick authority");
        else
            RCLCPP_ERROR(this->get_logger(), "Failed to obtain joystick authority");
    } else
        RCLCPP_ERROR(this->get_logger(), "Failed to obtain joystick authority");

    auto set_joystick_mode_request = std::make_shared<psdk_interfaces::srv::SetJoystickMode::Request>();
    set_joystick_mode_request->horizontal_control_mode = 1; // velocity control
    set_joystick_mode_request->vertical_control_mode = 0;   // velocity control
    set_joystick_mode_request->yaw_control_mode = 0;        // yaw angle control
    set_joystick_mode_request->horizontal_coordinate = 1;   // body frame
    set_joystick_mode_request->stable_control_mode = 1;     // stable mode enabled

    auto set_joystick_mode_result = set_joystick_mode_client_->async_send_request(set_joystick_mode_request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), set_joystick_mode_result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        if (set_joystick_mode_result.get()->is_success)
            RCLCPP_INFO(this->get_logger(), "Set joystick mode successfully");
        else
            RCLCPP_ERROR(this->get_logger(), "Failed to set joystick mode");
    } else
        RCLCPP_ERROR(this->get_logger(), "Failed to set joystick mode");
}

void LandingPadController::retrievePidParameters()
{
    double x_kp, x_ki, x_kd, x_dt, x_max, x_min;
    double y_kp, y_ki, y_kd, y_dt, y_max, y_min;
    double z_kp, z_ki, z_kd, z_dt, z_max, z_min;
    double yaw_kp, yaw_ki, yaw_kd, yaw_dt, yaw_max, yaw_min;

    this->declare_parameter<double>("x_pid.kp", 3.0);
    this->declare_parameter<double>("x_pid.ki", 0.0);
    this->declare_parameter<double>("x_pid.kd", 0.5);
    this->declare_parameter<double>("x_pid.dt", 0.1);
    this->declare_parameter<double>("x_pid.max", 2.0);
    this->declare_parameter<double>("x_pid.min", -2.0);

    this->declare_parameter<double>("y_pid.kp", 3.0);
    this->declare_parameter<double>("y_pid.ki", 0.0);
    this->declare_parameter<double>("y_pid.kd", 0.5);
    this->declare_parameter<double>("y_pid.dt", 0.1);
    this->declare_parameter<double>("y_pid.max", 2.0);
    this->declare_parameter<double>("y_pid.min", -2.0);

    this->declare_parameter<double>("z_pid.kp", 3.0);
    this->declare_parameter<double>("z_pid.ki", 0.0);
    this->declare_parameter<double>("z_pid.kd", 0.5);
    this->declare_parameter<double>("z_pid.dt", 0.1);
    this->declare_parameter<double>("z_pid.max", 2.0);
    this->declare_parameter<double>("z_pid.min", -2.0);

    this->get_parameter("x_pid.kp", x_kp);
    this->get_parameter("x_pid.ki", x_ki);
    this->get_parameter("x_pid.kd", x_kd);
    this->get_parameter("x_pid.dt", x_dt);
    this->get_parameter("x_pid.max", x_max);
    this->get_parameter("x_pid.min", x_min);

    this->get_parameter("y_pid.kp", y_kp);
    this->get_parameter("y_pid.ki", y_ki);
    this->get_parameter("y_pid.kd", y_kd);
    this->get_parameter("y_pid.dt", y_dt);
    this->get_parameter("y_pid.max", y_max);
    this->get_parameter("y_pid.min", y_min);

    this->get_parameter("z_pid.kp", z_kp);
    this->get_parameter("z_pid.ki", z_ki);
    this->get_parameter("z_pid.kd", z_kd);
    this->get_parameter("z_pid.dt", z_dt);
    this->get_parameter("z_pid.max", z_max);
    this->get_parameter("z_pid.min", z_min);

    x_pid_ = PID(x_kp, x_ki, x_kd, x_dt, x_max, x_min);
    y_pid_ = PID(y_kp, y_ki, y_kd, y_dt, y_max, y_min);
    z_pid_ = PID(z_kp, z_ki, z_kd, z_dt, z_max, z_min);

    std::cout << "X PID: " << x_kp << " " << x_ki << " " << x_kd << " " << x_dt << " " << x_max << " " << x_min << std::endl;
    std::cout << "Y PID: " << y_kp << " " << y_ki << " " << y_kd << " " << y_dt << " " << y_max << " " << y_min << std::endl;
    std::cout << "Z PID: " << z_kp << " " << z_ki << " " << z_kd << " " << z_dt << " " << z_max << " " << z_min << std::endl;
}

LandingPadController::~LandingPadController()
{
    auto release_joystick_authority_request = std::make_shared<psdk_interfaces::srv::ReleaseJoystickAuthority::Request>();
    auto release_joystick_authority_result = release_joystick_authority_client_->async_send_request(release_joystick_authority_request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), release_joystick_authority_result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        if (release_joystick_authority_result.get()->is_success)
            RCLCPP_INFO(this->get_logger(), "Released joystick authority upon destructor called");
        else
            RCLCPP_ERROR(this->get_logger(), "Failed to release joystick authority");
    } else
        RCLCPP_ERROR(this->get_logger(), "Failed to release joystick authority");
}
