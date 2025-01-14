#include <drone_landing_planner/drone_landing.hpp>

DroneLander::DroneLander() : Node("drone_landing_node")
{
    string landing_pad_position_topic, velocity_command_topic, land_on_pad_action_name;
    string obtain_joystick_authority_server_name, set_joystick_mode_server_name;
    this->declare_parameter<string>("publishers.velocity_command", "");
    this->declare_parameter<string>("subscribers.landing_pad_position", "");
    this->declare_parameter<string>("service_clients.obtain_joystick_authority", "");
    this->declare_parameter<string>("service_clients.set_joystick_mode", "");
    this->declare_parameter<string>("actions.land_on_pad", "");
    this->declare_parameter<float>("vertical_setpoint", 1.0);
    this->declare_parameter<float>("horizontal_deviation_threshold", 0.2);
    this->declare_parameter<float>("vertical_deviation_threshold", 0.2);
    
    this->get_parameter("publishers.velocity_command", velocity_command_topic);
    this->get_parameter("subscribers.landing_pad_position", landing_pad_position_topic);
    this->get_parameter("service_clients.obtain_joystick_authority", obtain_joystick_authority_server_name);
    this->get_parameter("service_clients.set_joystick_mode", set_joystick_mode_server_name);
    this->get_parameter("actions.land_on_pad", land_on_pad_action_name);
    this->get_parameter("vertical_setpoint", vertical_setpoint_);
    this->get_parameter("horizontal_deviation_threshold", horizontal_deviation_threshold_);
    this->get_parameter("vertical_deviation_threshold", vertical_deviation_threshold_);

    // Initialize subscribers
    landing_pad_position_sub_ = this->create_subscription<Point>(
        landing_pad_position_topic, 10, std::bind(&DroneLander::positionCallback, this, std::placeholders::_1));

    // Initialize publishers
    cmd_vel_pub_ = this->create_publisher<VelocityCommand>(velocity_command_topic, 10);

    // Initialize service clients
    obtain_joystick_authority_client_ = this->create_client<ObtainJoystickAuthority>(obtain_joystick_authority_server_name);
    set_joystick_mode_client_ = this->create_client<SetJoystickMode>(set_joystick_mode_server_name);

    // Wait for the services to be available
    while (!obtain_joystick_authority_client_->wait_for_service(3s)) {
        RCLCPP_INFO(this->get_logger(), "Waiting for ObtainJoystickAuthority service...");
        if (!rclcpp::ok()) return;
    }
    while (!set_joystick_mode_client_->wait_for_service(3s)) {
        RCLCPP_INFO(this->get_logger(), "Waiting for SetJoystickMode service...");
        if (!rclcpp::ok()) return;
    }

    // Retrieve and initialize PID parameters for all axes
    initPID("x", x_pid_);
    initPID("y", y_pid_);
    initPID("z", z_pid_);
    
    // Initialize action server
    land_on_pad_action_server_ = create_server<LandOnPad>(
        this, land_on_pad_action_name,
        [this](const GoalUUID & uuid, std::shared_ptr<const LandOnPad::Goal> goal) {
            RCLCPP_INFO(this->get_logger(), "Received LandOnPad goal request");
            (void)uuid;
            return GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [this](const std::shared_ptr<ServerGoalHandle<LandOnPad>> goal_handle) {
            RCLCPP_INFO(this->get_logger(), "Received request to cancel LandOnPad goal");
            return CancelResponse::ACCEPT;
        },
        [this](const std::shared_ptr<ServerGoalHandle<LandOnPad>> goal_handle) {
            // Execute the goal in a separate thread
            std::thread([this, goal_handle]() {
                execute(goal_handle);
            }).detach();
        }
    );

    RCLCPP_INFO(this->get_logger(), "Finished initialization!");
}

void DroneLander::execute(const std::shared_ptr<ServerGoalHandle<LandOnPad>> goal_handle)
{
    obtainJoystickAuthority();
    setJoystickMode();

    is_action_called_ = true;
    auto goal = goal_handle->get_goal();
    int timeout = goal->timeout;

    auto result = std::make_shared<LandOnPad::Result>();
    auto starting_time = std::chrono::steady_clock::now();
    rclcpp::Rate rate(10);

    while (rclcpp::ok())
    {
        rate.sleep();

        // Check if the goal is being cancelled
        if (goal_handle->is_canceling())
        {
            RCLCPP_INFO(this->get_logger(), "LandOnPad is being cancelled");
            result->error_code = 3;
            goal_handle->canceled(result);
            break;
        }

        if (is_completed_)
        {
            result->error_code = 0;
            goal_handle->succeed(result);           
            RCLCPP_INFO(this->get_logger(), "LandOnPad completed");
            break;
        }

        if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - starting_time).count() > timeout)
        {
            RCLCPP_INFO(this->get_logger(), "LandOnPad timed out");
            result->error_code = 1;
            goal_handle->abort(result);
            break;
        } 
    }
    is_action_called_ = false;
}

void DroneLander::positionCallback(const Point::SharedPtr msg)
{
    if (!is_action_called_)
        return;

    // x deviation is rightwards in the body frame (aka rightwards in the camera frame)
    float x_deviation_in_meter = (msg->x - 0.5) * msg->z;  
    // y axis is backwards in the body frame (aka downwards in the camera frame)
    float y_deviation_in_meter = (msg->y - 0.5) * msg->z;   

    if (abs(msg->z - vertical_setpoint_) < vertical_deviation_threshold_ 
        && abs(x_deviation_in_meter) < horizontal_deviation_threshold_ 
        && abs(y_deviation_in_meter) < horizontal_deviation_threshold_)
    {
        if (!prev_goal_reached_time_.has_value())
            prev_goal_reached_time_ = std::chrono::steady_clock::now();
        else
        {
            auto now = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - prev_goal_reached_time_.value()).count();
            if (duration >= 3)
            {
                is_completed_ = true;
                RCLCPP_INFO(this->get_logger(), "Goal reached for 3 seconds");
            }
        }
        std::cout << "Goal reached" << std::endl;
    }
    else
        // Reset the time if the goal is not reached
        prev_goal_reached_time_.reset();
        
    auto velocity_command = VelocityCommand();

    // Calculate pid from (setpoint, current)
    // For velocity command, the coordinate system in body frame is Forward-Right-Down (FRD), as per PSDK documentation
    // It differs from the camera frame which is Right-Backward-Down, so we have to be careful
    // Error is always setpoint - current, so you can check for the signs of the error to see if the drone is moving in the correct direction

    velocity_command.x = x_pid_.calculate(0, y_deviation_in_meter);
    velocity_command.y = -y_pid_.calculate(0, x_deviation_in_meter);
    velocity_command.z = z_pid_.calculate(vertical_setpoint_, msg->z);
    velocity_command.yaw = 0.0;

    cmd_vel_pub_->publish(velocity_command);
    // RCLCPP_INFO(this->get_logger(), "Foward: %f, Right: %f, Up: %f", velocity_command.x, velocity_command.y, velocity_command.z);
}

void DroneLander::obtainJoystickAuthority()
{
    auto obtain_authority_request = std::make_shared<ObtainJoystickAuthority::Request>();
    auto obtain_authority_result = obtain_joystick_authority_client_->async_send_request(obtain_authority_request,
        [this](rclcpp::Client<ObtainJoystickAuthority>::SharedFuture future) {
            if (!future.valid()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to call ObtainJoystickAuthority service");
                return;
            }

            auto result = future.get();
            if (!result->is_success) {
                RCLCPP_ERROR(this->get_logger(), "ObtainJoystickAuthority service call failed");
            } else {
                RCLCPP_INFO(this->get_logger(), "Joystick authority obtained successfully");
            }
        }
    );
}

void DroneLander::setJoystickMode()
{
    auto set_mode_request = std::make_shared<SetJoystickMode::Request>();
    set_mode_request->horizontal_control_mode = 1; // velocity control
    set_mode_request->vertical_control_mode = 0;   // velocity control
    set_mode_request->yaw_control_mode = 0;        // yaw angle control
    set_mode_request->horizontal_coordinate = 1;   // body frame
    set_mode_request->stable_control_mode = 1;     // stable mode enabled

    auto set_mode_result = set_joystick_mode_client_->async_send_request(set_mode_request,
        [this](rclcpp::Client<SetJoystickMode>::SharedFuture future) {
            if (!future.valid()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to call SetJoystickMode service");
                return;
            }

            auto result = future.get();
            if (!result->is_success) {
                RCLCPP_ERROR(this->get_logger(), "SetJoystickMode service call failed");
            } else {
                RCLCPP_INFO(this->get_logger(), "Joystick mode set successfully");
            }
        }
    );
}

void DroneLander::initPID(const string &axis, PID &pid)
{
    float kp, ki, kd, dt, max, min, conditional_integral_input_deviation;
    bool verbose;
    this->declare_parameter<float>(axis + "_pid.kp", 0.0);
    this->declare_parameter<float>(axis + "_pid.ki", 0.0);
    this->declare_parameter<float>(axis + "_pid.kd", 0.0);
    this->declare_parameter<float>(axis + "_pid.dt", 0.0);
    this->declare_parameter<float>(axis + "_pid.max", 0.0);
    this->declare_parameter<float>(axis + "_pid.min", 0.0);
    this->declare_parameter<float>(axis + "_pid.conditional_integral_input_deviation", 1e10);   // If not given, all input values will be considered for integral
    this->declare_parameter<bool>(axis + "_pid.verbose", false);

    this->get_parameter(axis + "_pid.kp", kp);
    this->get_parameter(axis + "_pid.ki", ki);
    this->get_parameter(axis + "_pid.kd", kd);
    this->get_parameter(axis + "_pid.dt", dt);
    this->get_parameter(axis + "_pid.max", max);
    this->get_parameter(axis + "_pid.min", min);
    this->get_parameter(axis + "_pid.conditional_integral_input_deviation", conditional_integral_input_deviation);
    this->get_parameter(axis + "_pid.verbose", verbose);

    pid = PID(kp, ki, kd, dt, max, min);
    pid.enable_verbose_mode(verbose, axis + "_pid");
    pid.set_conditional_integral_input_range(-conditional_integral_input_deviation, conditional_integral_input_deviation);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DroneLander>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}