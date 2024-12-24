#include <drone_landing_planner/descend_and_center.hpp>

DescendAndCenter::DescendAndCenter() : Node("descend_and_center_node")
{
    this->declare_parameter<std::string>("landing_pad_position_topic", "/landing_pad_position");
    this->declare_parameter<std::string>("velocity_command_topic", "/drone_velocity_command");
    
    this->get_parameter("landing_pad_position_topic", position_topic_);
    this->get_parameter("velocity_command_topic", velocity_command_topic_);

    std::string node_name = this->get_name();
    
    descend_and_center_action_server_ = rclcpp_action::create_server<DescendAndCenterAction>
    (
        this, node_name + "/descend_and_center_action",
        [this](const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const DescendAndCenterAction::Goal> goal) {
            RCLCPP_INFO(this->get_logger(), "Received DescendAndCenter goal request");
            (void)uuid;
            // Accept the goal
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<DescendAndCenterAction>> goal_handle) {
            RCLCPP_INFO(this->get_logger(), "Received request to cancel DescendAndCenterAction goal");
            // Accept the cancel request
            return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<DescendAndCenterAction>> goal_handle) {
            // Execute the goal in a separate thread
            std::thread([this, goal_handle]() {
                execute(goal_handle);
            }).detach();
        }
    );

    landing_pad_position_sub_ = this->create_subscription<Point>(
        position_topic_, 10, std::bind(&DescendAndCenter::positionCallback, this, std::placeholders::_1));

    velocity_pub_ = this->create_publisher<VelocityCommand>(velocity_command_topic_, 10);

    retrievePidParameters();
    this->declare_parameter<float>("z_setpoint", 1.0);
    this->get_parameter("z_setpoint", z_setpoint_);
    std::cout << "Z setpoint: " << z_setpoint_ << std::endl;
}

void DescendAndCenter::execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<DescendAndCenterAction>> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing DescendAndCenter action");
    is_action_called_ = true;

    auto goal = goal_handle->get_goal();
    int timeout = goal->timeout;

    auto result = std::make_shared<DescendAndCenterAction::Result>();
    auto feedback = std::make_shared<DescendAndCenterAction::Feedback>();
    auto starting_time = std::chrono::steady_clock::now();
    rclcpp::Rate rate(100);

    while (true)
    {
        if (is_completed_)
        {
            result->error_code = 0;
            goal_handle->succeed(result);           
            RCLCPP_INFO(this->get_logger(), "DescendAndCenter action completed");
            break;
        }

        if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - starting_time).count() > timeout)
        {
            RCLCPP_INFO(this->get_logger(), "DescendAndCenter action timed out");
            result->error_code = 1;
            goal_handle->abort(result);
            break;
        }
        
        if (goal_handle->is_canceling())
        {
            RCLCPP_INFO(this->get_logger(), "DescendAndCenter action is being cancelled");
            result->error_code = 3;
            goal_handle->canceled(result);
            break;
        }

        feedback->current_height = z_current_;
        goal_handle->publish_feedback(feedback);
        rate.sleep();
    }
    is_action_called_ = false;
}

void DescendAndCenter::positionCallback(const Point::SharedPtr msg)
{
    if (!is_action_called_)
        return;

    z_current_ = msg->z;
    if (abs(z_current_ - z_setpoint_) < 0.2)
    {
        if (!prev_z_goal_reached_time_.has_value())
        {
            prev_z_goal_reached_time_ = std::chrono::steady_clock::now();
        }
        else
        {
            auto now = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - prev_z_goal_reached_time_.value()).count();
            if (duration >= 3)
            {
                is_completed_ = true;
                RCLCPP_INFO(this->get_logger(), "Goal reached for 3 seconds");
            }
        }
    }
    else
    {
        // Reset the time if the goal is not reached
        prev_z_goal_reached_time_.reset();
    }
        
    auto velocity_command = VelocityCommand();

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

void DescendAndCenter::retrievePidParameters()
{
    double x_kp, x_ki, x_kd, x_dt, x_max, x_min;
    double y_kp, y_ki, y_kd, y_dt, y_max, y_min;
    double z_kp, z_ki, z_kd, z_dt, z_max, z_min;
    double yaw_kp, yaw_ki, yaw_kd, yaw_dt, yaw_max, yaw_min;

    this->declare_parameter<double>("x_pid.kp", 0.);
    this->declare_parameter<double>("x_pid.ki", 0.);
    this->declare_parameter<double>("x_pid.kd", 0.);
    this->declare_parameter<double>("x_pid.dt", 0.);
    this->declare_parameter<double>("x_pid.max", 0.);
    this->declare_parameter<double>("x_pid.min", 0.);

    this->declare_parameter<double>("y_pid.kp", 0.);
    this->declare_parameter<double>("y_pid.ki", 0.);
    this->declare_parameter<double>("y_pid.kd", 0.);
    this->declare_parameter<double>("y_pid.dt", 0.);
    this->declare_parameter<double>("y_pid.max", 0.);
    this->declare_parameter<double>("y_pid.min", 0.);

    this->declare_parameter<double>("z_pid.kp", 0.);
    this->declare_parameter<double>("z_pid.ki", 0.);
    this->declare_parameter<double>("z_pid.kd", 0.);
    this->declare_parameter<double>("z_pid.dt", 0.);
    this->declare_parameter<double>("z_pid.max", 0.);
    this->declare_parameter<double>("z_pid.min", 0.);

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

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DescendAndCenter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}