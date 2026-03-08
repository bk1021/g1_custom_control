#ifndef G1_CUSTOM_CONTROL__G1_MOVEIT_BRIDGE_HPP_
#define G1_CUSTOM_CONTROL__G1_MOVEIT_BRIDGE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "unitree_hg/msg/low_cmd.hpp"
#include "unitree_hg/msg/low_state.hpp"

#include <array>
#include <map>
#include <mutex>
#include <string>
#include <vector>

namespace g1_custom_control {

constexpr int G1_NUM_MOTOR = 29;

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFJT = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

class G1MoveItBridge : public rclcpp::Node {
public:
    G1MoveItBridge(bool keep_lowbody_home);
    ~G1MoveItBridge();

private:
    // ROS 2 Callbacks
    void lowstate_callback(const unitree_hg::msg::LowState::SharedPtr msg);
    void control_loop();

    // Action Server Callbacks
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& uuid, 
        std::shared_ptr<const FollowJointTrajectory::Goal> goal);
    
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleFJT> goal_handle);
    
    void handle_accepted(const std::shared_ptr<GoalHandleFJT> goal_handle);
    void execute_trajectory(const std::shared_ptr<GoalHandleFJT> goal_handle);

    // ROS 2 Interfaces
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
    rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr lowstate_subscriber_;
    rclcpp::Publisher<unitree_hg::msg::LowCmd>::SharedPtr lowcmd_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    rclcpp_action::Server<FollowJointTrajectory>::SharedPtr left_arm_action_server_;
    rclcpp_action::Server<FollowJointTrajectory>::SharedPtr right_arm_action_server_;

    // Internal State
    std::map<std::string, int> name_to_index_;
    std::array<float, G1_NUM_MOTOR> initial_q_{0};
    std::array<float, G1_NUM_MOTOR> current_q_{0};
    std::array<float, G1_NUM_MOTOR> current_dq_{0};
    std::array<float, G1_NUM_MOTOR> target_q_{0};
    std::array<float, G1_NUM_MOTOR> current_tau_est_{0};
    
    bool initial_q_captured_ = false;
    std::mutex state_mutex_;

    double hc_weight_ = 0.0;
    // Note: In Unitree's HG SDK, the weight gate is often passed in an unused motor index
    const int WEIGHT_GATE_INDEX = 29;

    bool keep_lowbody_home_;
};

} // namespace g1_custom_control

#endif // G1_CUSTOM_CONTROL__G1_MOVEIT_BRIDGE_HPP_