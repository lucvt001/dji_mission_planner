#include "vehicle_following_planner/vehicle_following.hpp"

VehicleFollower::VehicleFollower() : Node("vehicle_following_node")
{
    // Declare and get parameters
    string car_gps_topic, drone_gps_topic, yaw_control_topic, height_control_topic, velocity_command_topic, follow_vehicle_action_name;
    string obtain_joystick_authority_server_name, set_joystick_mode_server_name;
    string car_ned;
    this->declare_parameter<string>("publishers.velocity_command", "");
    this->declare_parameter<string>("publishers.car_ned", "");
    this->declare_parameter<string>("subscribers.car_gps", "");
    this->declare_parameter<string>("subscribers.drone_gps", "");
    this->declare_parameter<string>("subscribers.yaw_control", "");
    this->declare_parameter<string>("subscribers.height_control", "");
    this->declare_parameter<string>("service_clients.obtain_joystick_authority", "");
    this->declare_parameter<string>("service_clients.set_joystick_mode", "");
    this->declare_parameter<string>("actions.follow_vehicle", "");
    this->get_parameter("publishers.velocity_command", velocity_command_topic);
    this->get_parameter("publishers.car_ned", car_ned);
    this->get_parameter("subscribers.car_gps", car_gps_topic);
    this->get_parameter("subscribers.drone_gps", drone_gps_topic);
    this->get_parameter("subscribers.yaw_control", yaw_control_topic);
    this->get_parameter("subscribers.height_control", height_control_topic);
    this->get_parameter("service_clients.obtain_joystick_authority", obtain_joystick_authority_server_name);
    this->get_parameter("service_clients.set_joystick_mode", set_joystick_mode_server_name);
    this->get_parameter("actions.follow_vehicle", follow_vehicle_action_name);

    // Initialize subscribers with their lambda callback functions
    car_gps_sub_ = this->create_subscription<NavSatFix>(car_gps_topic, 2,
        [this](const NavSatFix::SharedPtr msg) { car_gps_data_ = *msg; is_new_msg_received_ = true; });
    drone_gps_sub_ = this->create_subscription<NavSatFix>(drone_gps_topic, 2,
        [this](const NavSatFix::SharedPtr msg) { drone_gps_data_ = *msg; is_new_msg_received_ = true; });
    yaw_control_sub_ = this->create_subscription<Float32>(yaw_control_topic, 2,
        [this](const Float32::SharedPtr msg) { yaw_control_data_ = msg->data; is_new_msg_received_ = true; });
    height_control_sub_ = this->create_subscription<Float32>(height_control_topic, 2,
        [this](const Float32::SharedPtr msg) { height_control_data_ = msg->data; is_new_msg_received_ = true; });

    // Initialize publishers
    cmd_vel_pub_ = this->create_publisher<VelocityCommand>(velocity_command_topic, 2);
    car_ned_pub_ = this->create_publisher<Point>(car_ned, 2);

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
    follow_vehicle_action_server_ = create_server<FollowVehicle>(
        this, follow_vehicle_action_name,
        [this](const GoalUUID & uuid, std::shared_ptr<const FollowVehicle::Goal> goal) {
            RCLCPP_INFO(this->get_logger(), "Received FollowVehicle goal request");
            (void)uuid;
            return GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [this](const std::shared_ptr<ServerGoalHandle<FollowVehicle>> goal_handle) {
            RCLCPP_INFO(this->get_logger(), "Received request to cancel FollowVehicle goal");
            return CancelResponse::ACCEPT;
        },
        [this](const std::shared_ptr<ServerGoalHandle<FollowVehicle>> goal_handle) {
            std::thread([this, goal_handle]() {
                execute(goal_handle);
            }).detach();
        }
    );

    RCLCPP_INFO(this->get_logger(), "Finished initialization!");
}

void VehicleFollower::execute(const std::shared_ptr<ServerGoalHandle<FollowVehicle>> goal_handle)
{
    obtainJoystickAuthority();
    setJoystickMode();
    
    rclcpp::Rate rate(10);
    auto start_time = this->now();

    while (rclcpp::ok())
    {
        rate.sleep();

        // Check if the goal is being cancelled
        if (goal_handle->is_canceling())
        {
            RCLCPP_INFO(this->get_logger(), "FollowVehicle action is being cancelled");
            auto result = std::make_shared<FollowVehicle::Result>();
            goal_handle->canceled(result);
            return;
        }

        // Skip the first x seconds of the loop
        auto current_time = this->now();
        auto elapsed_time = current_time - start_time;
        if (elapsed_time.seconds() < 2.0) 
        {
            cmd_vel_pub_->publish(VelocityCommand());
            continue;
        }

        if (!is_new_msg_received_) continue;

        // Main calculation starts from here
        // Calculate NED coordinates of the car relative to the drone
        // And publish to a topic for logging purpose
        Point car_ned = gps_to_ned(car_gps_data_, drone_gps_data_);
        car_ned_pub_->publish(car_ned);

        float x_vel = x_pid_.calculate(0, -car_ned.x);
        float y_vel = y_pid_.calculate(0, -car_ned.y);
        float z_vel = (height_control_data_ >= 2) ? -z_pid_.calculate(height_control_data_, car_ned.z) : 0; // If target height is too low, leave vertical velocity to be 0

        // Publish velocity command to follow the car
        VelocityCommand velocity_command;
        velocity_command.x = x_vel;
        velocity_command.y = y_vel;
        velocity_command.z = z_vel;
        velocity_command.yaw = yaw_control_data_;   // Yaw angle (not rate)
        cmd_vel_pub_->publish(velocity_command);

        is_new_msg_received_ = false;
    }
}

void VehicleFollower::obtainJoystickAuthority()
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

void VehicleFollower::setJoystickMode()
{
    auto set_mode_request = std::make_shared<SetJoystickMode::Request>();
    set_mode_request->horizontal_control_mode = 1; // velocity control
    set_mode_request->vertical_control_mode = 0;   // velocity control
    set_mode_request->yaw_control_mode = 0;        // yaw angle control
    set_mode_request->horizontal_coordinate = 0;   // ground frame
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

void VehicleFollower::initPID(const string &axis, PID &pid)
{
    float kp, ki, kd, dt, max, min;
    bool verbose;
    this->declare_parameter<float>(axis + "_pid.kp", 0.0);
    this->declare_parameter<float>(axis + "_pid.ki", 0.0);
    this->declare_parameter<float>(axis + "_pid.kd", 0.0);
    this->declare_parameter<float>(axis + "_pid.dt", 0.0);
    this->declare_parameter<float>(axis + "_pid.max", 0.0);
    this->declare_parameter<float>(axis + "_pid.min", 0.0);
    this->declare_parameter<bool>(axis + "_pid.verbose", false);

    this->get_parameter(axis + "_pid.kp", kp);
    this->get_parameter(axis + "_pid.ki", ki);
    this->get_parameter(axis + "_pid.kd", kd);
    this->get_parameter(axis + "_pid.dt", dt);
    this->get_parameter(axis + "_pid.max", max);
    this->get_parameter(axis + "_pid.min", min);
    this->get_parameter(axis + "_pid.verbose", verbose);

    pid = PID(kp, ki, kd, dt, max, min);
    pid.enable_verbose_mode(verbose, axis + "_pid");
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VehicleFollower>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
