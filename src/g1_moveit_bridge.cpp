#include "g1_custom_control/g1_moveit_bridge.hpp"
#include "g1_custom_control/motor_crc_hg.h"
#include <algorithm>
#include <thread>

#define MAX_JOINT_SPEED 2.0 // radians per second, adjust as needed for safety

namespace g1_custom_control {

enum MotorType { GEARBOX_S = 0, GEARBOX_M = 1, GEARBOX_L = 2 };

const std::array<MotorType, G1_NUM_MOTOR> G1MotorType{
    GEARBOX_M, GEARBOX_M, GEARBOX_M, GEARBOX_L, GEARBOX_S, GEARBOX_S, // Left Leg
    GEARBOX_M, GEARBOX_M, GEARBOX_M, GEARBOX_L, GEARBOX_S, GEARBOX_S, // Right Leg
    GEARBOX_M, GEARBOX_S, GEARBOX_S,                                  // Waist
    GEARBOX_S, GEARBOX_S, GEARBOX_S, GEARBOX_S, GEARBOX_S, GEARBOX_S, GEARBOX_S, // Left Arm
    GEARBOX_S, GEARBOX_S, GEARBOX_S, GEARBOX_S, GEARBOX_S, GEARBOX_S, GEARBOX_S  // Right Arm
};

float GetMotorKp(MotorType type) {
    switch (type) { case GEARBOX_S: case GEARBOX_M: return 40.0; case GEARBOX_L: return 100.0; default: return 0.0; }
}

float GetMotorKd(MotorType type) {
    switch (type) { case GEARBOX_S: case GEARBOX_M: case GEARBOX_L: return 1.0; default: return 0.0; }
}

const std::vector<std::string> JOINT_NAMES = {
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint", 
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint", 
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint", 
    "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint", 
    "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint"
};

G1MoveItBridge::G1MoveItBridge(bool keep_lowbody_home) 
    : Node("g1_moveit_bridge"), keep_lowbody_home_(keep_lowbody_home) {

    for (int i = 0; i < G1_NUM_MOTOR; ++i) { name_to_index_[JOINT_NAMES[i]] = i; }

    joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    lowstate_subscriber_ = this->create_subscription<unitree_hg::msg::LowState>(
        "lowstate", 10, std::bind(&G1MoveItBridge::lowstate_callback, this, std::placeholders::_1));

    lowcmd_publisher_ = this->create_publisher<unitree_hg::msg::LowCmd>("lowcmd", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(2), std::bind(&G1MoveItBridge::control_loop, this));

    left_arm_action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
        this, "left_arm_controller/follow_joint_trajectory",
        std::bind(&G1MoveItBridge::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&G1MoveItBridge::handle_cancel, this, std::placeholders::_1),
        std::bind(&G1MoveItBridge::handle_accepted, this, std::placeholders::_1));

    right_arm_action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
        this, "right_arm_controller/follow_joint_trajectory",
        std::bind(&G1MoveItBridge::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&G1MoveItBridge::handle_cancel, this, std::placeholders::_1),
        std::bind(&G1MoveItBridge::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "MoveIt to Unitree LowCmd Bridge Started!");

    if (keep_lowbody_home_) {
        RCLCPP_WARN(this->get_logger(), "MuJoCo Mode: Legs will be rigidly locked (KLH active)!");
    } else {
        RCLCPP_INFO(this->get_logger(), "Hardware Mode: Legs yield to Unitree AI.");
    }
}

void G1MoveItBridge::lowstate_callback(const unitree_hg::msg::LowState::SharedPtr msg) {
    sensor_msgs::msg::JointState joint_state_msg;
    joint_state_msg.header.stamp = this->now();

    std::lock_guard<std::mutex> lock(state_mutex_);
    mode_machine_ = msg->mode_machine;
    for (int i = 0; i < G1_NUM_MOTOR; i++) {
        current_q_[i] = msg->motor_state[i].q;
        current_dq_[i] = msg->motor_state[i].dq;

        joint_state_msg.name.push_back(JOINT_NAMES[i]);
        joint_state_msg.position.push_back(current_q_[i]);
        joint_state_msg.velocity.push_back(current_dq_[i]);

        if (!initial_q_captured_) { 
            initial_q_[i] = current_q_[i];
            target_q_[i] = current_q_[i];
        }
    }
    initial_q_captured_ = true;
    joint_state_publisher_->publish(joint_state_msg);
}

void G1MoveItBridge::control_loop() {
    if (!initial_q_captured_) return;

    unitree_hg::msg::LowCmd low_cmd;
    low_cmd.mode_pr = mode_pr_;
    low_cmd.mode_machine = mode_machine_;

    // Ramp up from 0.0 to 1.0 over 2 seconds (2.0s * 500Hz = 1000 ticks)
    if (hc_weight_ < 1.0) {
        hc_weight_ += (1.0 / 1000.0);
        if (hc_weight_ > 1.0) hc_weight_ = 1.0;
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    for (int i = 0; i < G1_NUM_MOTOR; i++) {
        low_cmd.motor_cmd[i].mode = 0x01;
        if (i < 15) {
            if (keep_lowbody_home_) {
                // MUJOCO MODE: Smoothly transition from spawn pose (initial_q_) to 0.0
                double target_zero = 0.0;
                double current_leg_target = initial_q_[i] * (1.0 - hc_weight_) + target_zero * hc_weight_;
                
                low_cmd.motor_cmd[i].q = current_leg_target;
                low_cmd.motor_cmd[i].dq = 0.0;
                low_cmd.motor_cmd[i].tau = 0.0;
                low_cmd.motor_cmd[i].kp = GetMotorKp(G1MotorType[i]);
                low_cmd.motor_cmd[i].kd = GetMotorKd(G1MotorType[i]);
            } else {
                // HARDWARE MODE: Yield entirely to Unitree AI
                low_cmd.motor_cmd[i].q = 0.0;
                low_cmd.motor_cmd[i].dq = 0.0;
                low_cmd.motor_cmd[i].tau = 0.0;
                low_cmd.motor_cmd[i].kp = 0.0;
                low_cmd.motor_cmd[i].kd = 0.0;
            }
        } else {
            low_cmd.motor_cmd[i].q = target_q_[i];
            low_cmd.motor_cmd[i].dq = 0.0;
            low_cmd.motor_cmd[i].tau = 0.0;
            low_cmd.motor_cmd[i].kp = GetMotorKp(G1MotorType[i]);
            low_cmd.motor_cmd[i].kd = GetMotorKd(G1MotorType[i]);
        }
    }

    // Tell the High-Level AI to yield arm control based on our hc_weight_
    low_cmd.motor_cmd[WEIGHT_GATE_INDEX].q = hc_weight_;

    get_crc(low_cmd);
    lowcmd_publisher_->publish(low_cmd);
}

rclcpp_action::GoalResponse G1MoveItBridge::handle_goal(const rclcpp_action::GoalUUID&, std::shared_ptr<const FollowJointTrajectory::Goal>) {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse G1MoveItBridge::handle_cancel(const std::shared_ptr<GoalHandleFJT>) {
    return rclcpp_action::CancelResponse::ACCEPT;
}

void G1MoveItBridge::handle_accepted(const std::shared_ptr<GoalHandleFJT> goal_handle) {
    std::thread{std::bind(&G1MoveItBridge::execute_trajectory, this, goal_handle)}.detach();
}

void G1MoveItBridge::execute_trajectory(const std::shared_ptr<GoalHandleFJT> goal_handle) {
    const auto goal = goal_handle->get_goal();
    auto start_time = this->now();
    
    std::vector<int> mapped_indices;
    for (const auto& name : goal->trajectory.joint_names) {
        mapped_indices.push_back(name_to_index_[name]);
    }

    std::vector<double> actual_start_q(G1_NUM_MOTOR);
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        for(int i = 0; i < G1_NUM_MOTOR; i++) {
            actual_start_q[i] = current_q_[i];
        }
    }

    rclcpp::Rate loop_rate(500);
    for (size_t i = 1; i < goal->trajectory.points.size(); ++i) {
        auto& prev_point = goal->trajectory.points[i - 1];
        auto& target_point = goal->trajectory.points[i];
        
        double target_time = rclcpp::Duration(target_point.time_from_start).seconds();
        double prev_time = rclcpp::Duration(prev_point.time_from_start).seconds();
        
        while (rclcpp::ok() && (this->now() - start_time).seconds() < target_time) {
            if (goal_handle->is_canceling()) {
                auto result = std::make_shared<FollowJointTrajectory::Result>();
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Trajectory Canceled by MoveIt!");
                return; // Exit the thread immediately!
            }

            double current_time = (this->now() - start_time).seconds();
            double ratio = (current_time - prev_time) / (target_time - prev_time);
            ratio = std::clamp(ratio, 0.0, 1.0);

            std::lock_guard<std::mutex> lock(state_mutex_);
            for (size_t j = 0; j < mapped_indices.size(); ++j) {
                int motor_id = mapped_indices[j];

                // Interpolate from the ACTUAL position for the first segment
                double start_angle = (i == 1) ? actual_start_q[motor_id] : prev_point.positions[j];
                target_q_[motor_id] = start_angle + ratio * (target_point.positions[j] - start_angle);
            }
            loop_rate.sleep();
        }
    }
    
    // Give the physical MuJoCo motors 0.5 seconds to "settle" into their final positions
    rclcpp::sleep_for(std::chrono::milliseconds(500));

    auto result = std::make_shared<FollowJointTrajectory::Result>();
    bool reached_goal = true;
    double tolerance = 0.10; // 0.10 rad is ~5.7 degrees of allowable physical error

    // Grab the final target waypoint MoveIt wanted us to reach
    auto& final_point = goal->trajectory.points.back();
    
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        for (size_t j = 0; j < mapped_indices.size(); ++j) {
            int motor_id = mapped_indices[j];
            double target_angle = final_point.positions[j];
            double actual_angle = current_q_[motor_id]; 

            if (std::abs(target_angle - actual_angle) > tolerance) {
                RCLCPP_ERROR(this->get_logger(), 
                    "Physics check failed for %s! Target: %.3f rad, Actual: %.3f rad", 
                    goal->trajectory.joint_names[j].c_str(), target_angle, actual_angle);
                reached_goal = false;
            }
        }
    }

    if (reached_goal) {
        result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Trajectory Executed and Physically Verified!");
    } else {
        // Tell MoveIt the robot failed to physically reach the pose
        result->error_code = FollowJointTrajectory::Result::GOAL_TOLERANCE_VIOLATED;
        goal_handle->abort(result);
        RCLCPP_ERROR(this->get_logger(), "Action Aborted: The physical robot could not reach the IK target.");
    }
}

G1MoveItBridge::~G1MoveItBridge() {
    RCLCPP_INFO(this->get_logger(), "Shutting down... Handing arm control back to Unitree AI.");
    
    unitree_hg::msg::LowCmd low_cmd;
    low_cmd.mode_pr = mode_pr_;
    low_cmd.mode_machine = mode_machine_;

    // Ramp weight down from 1.0 to 0.0 over 1 second (500 ticks at 2ms)
    for (int step = 0; step <= 500; ++step) {
        hc_weight_ = 1.0 - (step / 500.0);
        
        std::lock_guard<std::mutex> lock(state_mutex_);
        for (int i = 0; i < G1_NUM_MOTOR; i++) {
            low_cmd.motor_cmd[i].mode = 0x01;
            if (i >= 15) { // Keep holding the arms steady while fading the weight
                low_cmd.motor_cmd[i].q = target_q_[i];
                low_cmd.motor_cmd[i].dq = 0.0;
                low_cmd.motor_cmd[i].tau = 0.0;
                low_cmd.motor_cmd[i].kp = GetMotorKp(G1MotorType[i]);
                low_cmd.motor_cmd[i].kd = GetMotorKd(G1MotorType[i]);
            }
        }
        
        low_cmd.motor_cmd[WEIGHT_GATE_INDEX].q = hc_weight_;
        get_crc(low_cmd);
        lowcmd_publisher_->publish(low_cmd);
        
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    RCLCPP_INFO(this->get_logger(), "Control successfully returned to AI. Safe to exit.");
}

} // namespace g1_custom_control

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    // Parse the klh argument
    bool keep_lowbody_home = false;
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "klh") {
            keep_lowbody_home = true;
            break;
        }
    }

    rclcpp::spin(std::make_shared<g1_custom_control::G1MoveItBridge>(keep_lowbody_home));
    rclcpp::shutdown();
    return 0;
}
