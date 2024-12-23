#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <psdk_interfaces/action/take_off.hpp>
#include <psdk_interfaces/action/move_to_position.hpp>
#include <drone_landing_planner/init_flight_control.hpp>
#include <drone_landing_planner/descend_and_center.hpp>

using TakeOff = psdk_interfaces::action::TakeOff;
using MoveToPosition = psdk_interfaces::action::MoveToPosition;
using DescendAndCenterAction = dji_mission_interfaces::action::DescendAndCenter;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("drone_landing_testing");
    initialize_flight_control(node);

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
    std::this_thread::sleep_for(std::chrono::seconds(5));   
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Assume that TakeOff action completed");


    auto move_to_position_goal = MoveToPosition::Goal();
    move_to_position_goal.x = 0.0;
    move_to_position_goal.y = 0.0;
    move_to_position_goal.z = 4.0;
    move_to_position_goal.timeout = 10.0;   // 15 second
    auto move_to_position_send_goal_options = rclcpp_action::Client<MoveToPosition>::SendGoalOptions();
    auto future_result_move_to_position = move_to_position_action_client->async_send_goal(move_to_position_goal, move_to_position_send_goal_options);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "MoveToPosition action sent");
    std::this_thread::sleep_for(std::chrono::seconds(10));   

    // Descend and center
    auto descend_and_center_goal = DescendAndCenterAction::Goal();
    descend_and_center_goal.timeout = 20;
    auto descend_and_center_send_goal_options = rclcpp_action::Client<DescendAndCenterAction>::SendGoalOptions();
    auto future_result_descend_and_center = descend_and_center_action_client->async_send_goal(descend_and_center_goal, descend_and_center_send_goal_options);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "DescendAndCenter action sent");
    std::this_thread::sleep_for(std::chrono::seconds(20));

    rclcpp::shutdown();
    return 0;
}