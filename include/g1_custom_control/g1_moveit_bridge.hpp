#ifndef G1_CUSTOM_CONTROL__G1_MOVEIT_BRIDGE_HPP_
#define G1_CUSTOM_CONTROL__G1_MOVEIT_BRIDGE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "unitree_hg/msg/low_cmd.hpp"
#include "unitree_hg/msg/low_state.hpp"
#include "g1/g1_motion_switch_client.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <vector>

namespace g1_custom_control {

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFJT = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

constexpr int G1_NUM_MOTOR = 29;

enum MotorType { GEARBOX_S = 0, GEARBOX_M = 1, GEARBOX_L = 2 };

const std::array<MotorType, G1_NUM_MOTOR> G1MotorType{
    GEARBOX_M, GEARBOX_M, GEARBOX_M, GEARBOX_L, GEARBOX_S, GEARBOX_S, // Left Leg
    GEARBOX_M, GEARBOX_M, GEARBOX_M, GEARBOX_L, GEARBOX_S, GEARBOX_S, // Right Leg
    GEARBOX_M, GEARBOX_S, GEARBOX_S,                                  // Waist
    GEARBOX_S, GEARBOX_S, GEARBOX_S, GEARBOX_S, GEARBOX_S, GEARBOX_S, GEARBOX_S, // Left Arm
    GEARBOX_S, GEARBOX_S, GEARBOX_S, GEARBOX_S, GEARBOX_S, GEARBOX_S, GEARBOX_S  // Right Arm
};

inline float GetMotorKp(MotorType type) {
    switch (type) { case GEARBOX_S: case GEARBOX_M: return 40.0; case GEARBOX_L: return 100.0; default: return 0.0; }
}

inline float GetMotorKd(MotorType type) {
    switch (type) { case GEARBOX_S: case GEARBOX_M: case GEARBOX_L: return 1.0; default: return 0.0; }
}

template <typename CommandT>
inline bool IsCommandChanged(
    const std::shared_ptr<const CommandT> &previous,
    const CommandT &next)
{
    return !previous ||
           next.q_target != previous->q_target ||
           next.dq_target != previous->dq_target ||
           next.kp != previous->kp ||
           next.kd != previous->kd ||
           next.tau_ff != previous->tau_ff;
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

inline const std::map<std::string, int> JOINT_NAME_TO_IDX = []() {
    std::map<std::string, int> mp;
    for (size_t i = 0; i < JOINT_NAMES.size(); ++i) {
        mp[JOINT_NAMES[i]] = i;
    }
    return mp;
}();

template <typename T>
class DataBuffer {
public:
    void SetData(const T &new_data)
    {
        std::unique_lock<std::shared_mutex> lock(mutex_);
        data_ = std::make_shared<T>(new_data);
    }

    std::shared_ptr<const T> GetData()
    {
        std::shared_lock<std::shared_mutex> lock(mutex_);
        return data_ ? data_ : nullptr;
    }

    void Clear()
    {
        std::unique_lock<std::shared_mutex> lock(mutex_);
        data_ = nullptr;
    }

private:
    std::shared_ptr<T> data_;
    std::shared_mutex mutex_;
};

class G1MoveItBridge : public rclcpp::Node {
public:
    G1MoveItBridge();
    ~G1MoveItBridge();
    void request_shutdown_homing();
    bool is_shutdown_homing_done() const;
    void perform_shutdown_homing();

private:
    struct TrajectoryRuntime;
    struct MotorState {
        std::array<float, G1_NUM_MOTOR> q{0};
        std::array<float, G1_NUM_MOTOR> dq{0};
    };

    struct MotorCommand {
        std::array<float, G1_NUM_MOTOR> q_target{0};
        std::array<float, G1_NUM_MOTOR> dq_target{0};
        std::array<float, G1_NUM_MOTOR> kp{0};
        std::array<float, G1_NUM_MOTOR> kd{0};
        std::array<float, G1_NUM_MOTOR> tau_ff{0};
    };

    // ROS 2 Callbacks
    void lowstate_callback(const unitree_hg::msg::LowState::SharedPtr msg);
    void command_writer_loop();
    void control_loop();

    // Action Server Callbacks
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& uuid, 
        std::shared_ptr<const FollowJointTrajectory::Goal> goal);
    
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleFJT> goal_handle);
    
    void handle_accepted(const std::shared_ptr<GoalHandleFJT> goal_handle);
    bool validate_goal_joints(
        const FollowJointTrajectory::Goal &goal,
        std::string &error_msg) const;
    bool snapshot_trajectory_runtime(TrajectoryRuntime &snapshot);
    void persist_trajectory_runtime(
        const std::shared_ptr<GoalHandleFJT> &goal_handle,
        bool settle_phase,
        size_t segment_index,
        const rclcpp::Time &settle_start_time);
    void finish_trajectory_goal(
        const std::shared_ptr<GoalHandleFJT> &goal_handle,
        int32_t error_code,
        bool canceled,
        const char *msg);
    void run_trajectory_state_machine(const rclcpp::Time &now, MotorCommand &command);

    // ROS 2 Interfaces
    rclcpp::CallbackGroup::SharedPtr lowstate_callback_group_;
    rclcpp::CallbackGroup::SharedPtr command_writer_callback_group_;
    rclcpp::CallbackGroup::SharedPtr control_callback_group_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
    rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr lowstate_subscriber_;
    rclcpp::Publisher<unitree_hg::msg::LowCmd>::SharedPtr lowcmd_publisher_;
    rclcpp::TimerBase::SharedPtr command_writer_timer_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    rclcpp_action::Server<FollowJointTrajectory>::SharedPtr left_arm_action_server_;
    rclcpp_action::Server<FollowJointTrajectory>::SharedPtr right_arm_action_server_;

    std::shared_ptr<unitree::robot::g1::MotionSwitchClient> msclient_;
    std::array<float, G1_NUM_MOTOR> initial_q_{0};
    std::array<float, G1_NUM_MOTOR> motor_kp_{0};
    std::array<float, G1_NUM_MOTOR> motor_kd_{0};
    double homing_ratio_ = 0.0;

    DataBuffer<MotorState> motor_state_buffer_;
    DataBuffer<MotorCommand> motor_command_buffer_;

    std::atomic<bool> initial_q_captured_{false};
    std::atomic<uint8_t> mode_machine_{0};
    uint8_t mode_pr_ = 0;

    std::mutex trajectory_mutex_;
    struct TrajectoryRuntime {
        bool active = false;
        bool cancel_requested = false;
        bool settle_phase = false;
        bool has_segment_vel = false;
        size_t segment_index = 1;
        double one_point_duration = 0.0;
        rclcpp::Time start_time{0, 0, RCL_ROS_TIME};
        rclcpp::Time settle_start_time{0, 0, RCL_ROS_TIME};
        std::shared_ptr<GoalHandleFJT> goal_handle;
        std::shared_ptr<const FollowJointTrajectory::Goal> goal;
        std::vector<int> mapped_indices;
        std::array<float, G1_NUM_MOTOR> actual_start_q{0};
    } trajectory_rt_;

    std::atomic<bool> shutdown_requested_{false};
    std::atomic<bool> shutdown_homing_done_{false};
    std::atomic<bool> shutdown_homing_active_{false};
    std::atomic<int> shutdown_homing_step_{0};
    std::mutex shutdown_homing_mutex_;
    std::array<float, G1_NUM_MOTOR> shutdown_homing_start_q_{0};

    // 2 ms loop period monitoring.
    std::chrono::steady_clock::time_point command_writer_last_tick_{};
    bool command_writer_timing_initialized_ = false;
    std::chrono::steady_clock::time_point control_last_tick_{};
    bool control_timing_initialized_ = false;

    int queryMotionStatus() 
    {
        std::string robotForm;
        std::string motionName;
        int motionStatus = 0;
        int32_t ret = msclient_->CheckMode(robotForm, motionName);
        if (ret == 0) {
            std::cout << "CheckMode succeeded." << std::endl;
        } else {
            std::cout << "CheckMode failed. Error code: " << ret << std::endl;
        }
        if (motionName.empty()) {
            std::cout << "The motion control-related service is deactivated." << std::endl;
            motionStatus = 0;
        } else {
            std::string serviceName = queryServiceName(robotForm, motionName);
            std::cout << "Service: " << serviceName << " is activate" << std::endl;
            motionStatus = 1;
        }
        return motionStatus;
    }

    static std::string queryServiceName(const std::string &form,
                                        const std::string &name) 
    {
        if (form == "0") {
            if (name == "normal") {
                return "sport_mode";
            }
            if (name == "ai") {
                return "ai_sport";
            }
            if (name == "advanced") {
                return "advanced_sport";
            }
        } else {
            if (name == "ai-w") {
                return "wheeled_sport(go2W)";
            }
            if (name == "normal-w") {
                return "wheeled_sport(b2W)";
            }
        }
        return "";
    }
};

} // namespace g1_custom_control

#endif // G1_CUSTOM_CONTROL__G1_MOVEIT_BRIDGE_HPP_