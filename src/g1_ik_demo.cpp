#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    // MoveIt requires parameters to be automatically declared from the config files
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("g1_ik_demo_node", node_options);

    // MoveIt needs to continuously read /joint_states in the background to know where the robot currently is
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread spinner = std::thread([&executor]() { executor.spin(); });

    // We specify "left_arm" exactly as it is named in your g1_29dof.srdf file
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(move_group_node, "left_arm");

    geometry_msgs::msg::Pose target_pose;
    target_pose.orientation.w = 1.0; // Pointing straight forward
    target_pose.position.x = 0.30;   // 30 cm forward
    target_pose.position.y = 0.20;   // 20 cm to the left
    target_pose.position.z = 0.40;   // 40 cm up (relative to the planning frame)
    
    // Tell MoveIt we want the hand to go here
    move_group_interface.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    // Execute the Trajectory (This sends the Action Goal to your g1_moveit_bridge!)
    if (success) {
        RCLCPP_INFO(move_group_node->get_logger(), "IK Math Successful! Executing trajectory...");
        move_group_interface.execute(my_plan);
    } else {
        RCLCPP_ERROR(move_group_node->get_logger(), "IK Math Failed! The target might be out of reach or causing a collision.");
    }

    rclcpp::shutdown();
    spinner.join();
    return 0;
}