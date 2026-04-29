#ifndef G1_CUSTOM_CONTROL__G1_CONTROL_CONSTANTS_HPP_
#define G1_CUSTOM_CONTROL__G1_CONTROL_CONSTANTS_HPP_

#include <algorithm>
#include <array>
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <vector>

namespace g1_custom_control {

// ---------------------------------------------------------------------------
// Motor layout
// ---------------------------------------------------------------------------

inline constexpr int G1_NUM_MOTOR = 29;

enum MotorType { GEARBOX_S = 0, GEARBOX_M = 1, GEARBOX_L = 2 };

inline const std::array<MotorType, G1_NUM_MOTOR> G1MotorType{
    GEARBOX_M, GEARBOX_M, GEARBOX_M, GEARBOX_L, GEARBOX_S, GEARBOX_S, // Left Leg
    GEARBOX_M, GEARBOX_M, GEARBOX_M, GEARBOX_L, GEARBOX_S, GEARBOX_S, // Right Leg
    GEARBOX_M, GEARBOX_S, GEARBOX_S,                                   // Waist
    GEARBOX_S, GEARBOX_S, GEARBOX_S, GEARBOX_S, GEARBOX_S, GEARBOX_S, GEARBOX_S, // Left Arm
    GEARBOX_S, GEARBOX_S, GEARBOX_S, GEARBOX_S, GEARBOX_S, GEARBOX_S, GEARBOX_S  // Right Arm
};

inline float GetMotorKp(MotorType type) {
    switch (type) {
        case GEARBOX_S: case GEARBOX_M: return 40.0f;
        case GEARBOX_L: return 100.0f;
        default: return 0.0f;
    }
}

inline float GetMotorKd(MotorType type) {
    switch (type) {
        case GEARBOX_S: case GEARBOX_M: case GEARBOX_L: return 1.0f;
        default: return 0.0f;
    }
}

// ---------------------------------------------------------------------------
// Joint name ↔ motor index mapping
// ---------------------------------------------------------------------------

inline const std::vector<std::string> JOINT_NAMES = {
    "left_hip_pitch_joint",  "left_hip_roll_joint",  "left_hip_yaw_joint",
    "left_knee_joint",       "left_ankle_pitch_joint","left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint",      "right_ankle_pitch_joint","right_ankle_roll_joint",
    "waist_yaw_joint",       "waist_roll_joint",     "waist_pitch_joint",
    "left_shoulder_pitch_joint",  "left_shoulder_roll_joint",  "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",  "left_wrist_pitch_joint",  "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint"
};

inline const std::map<std::string, int> JOINT_NAME_TO_IDX = []() {
    std::map<std::string, int> mp;
    for (size_t i = 0; i < JOINT_NAMES.size(); ++i) mp[JOINT_NAMES[i]] = static_cast<int>(i);
    return mp;
}();

// ---------------------------------------------------------------------------
// Thread-safe data buffer
// ---------------------------------------------------------------------------

template <typename T>
class DataBuffer {
public:
    void SetData(const T &new_data)
    {
        std::unique_lock<std::shared_mutex> wlock(mutex_);
        data_ = std::make_shared<T>(new_data);
    }

    std::shared_ptr<const T> GetData()
    {
        std::shared_lock<std::shared_mutex> rlock(mutex_);
        return data_ ? data_ : nullptr;
    }

    void Clear()
    {
        std::unique_lock<std::shared_mutex> wlock(mutex_);
        data_ = nullptr;
    }

private:
    std::shared_ptr<T> data_;
    std::shared_mutex mutex_;
};

// ---------------------------------------------------------------------------
// Timing constants
// ---------------------------------------------------------------------------

inline constexpr std::chrono::milliseconds kLowCmdControlPeriod{2};
inline constexpr std::chrono::milliseconds kArmSdkControlPeriod{20};
inline constexpr int    kLowCmdHomingSteps               = 1000;
inline constexpr int    kArmSdkHomingSteps               = 100;
inline constexpr double kLowCmdLoopOverrunThresholdMs    = 2.2;
inline constexpr double kArmSdkLoopOverrunThresholdMs    = 21.0;

// ---------------------------------------------------------------------------
// arm_sdk joint limits and gains
// ---------------------------------------------------------------------------

inline constexpr float  kArmSdkJointKp          = 60.0f;
inline constexpr float  kArmSdkJointKd          = 1.5f;
inline constexpr float  kArmSdkWaistGainScale   = 4.0f;
inline constexpr int    kWaistFirstJoint         = 12;
inline constexpr int    kWaistLastJoint          = 14;
inline constexpr int    kArmSdkFirstJoint        = 12;
inline constexpr int    kArmSdkLastJoint         = 28;
inline constexpr int    kArmSdkWeightJoint       = 29;

// ---------------------------------------------------------------------------
// Helper functions
// ---------------------------------------------------------------------------

inline constexpr bool is_waist_joint(int idx)
{ return idx >= kWaistFirstJoint && idx <= kWaistLastJoint; }

inline constexpr bool is_arm_sdk_joint(int idx)
{ return idx >= kArmSdkFirstJoint && idx <= kArmSdkLastJoint; }

inline constexpr int homing_steps_for_mode(bool use_arm_sdk)
{ return use_arm_sdk ? kArmSdkHomingSteps : kLowCmdHomingSteps; }

inline constexpr auto control_period_for_mode(bool use_arm_sdk)
{ return use_arm_sdk ? kArmSdkControlPeriod : kLowCmdControlPeriod; }

inline constexpr double loop_overrun_threshold_for_mode(bool use_arm_sdk)
{ return use_arm_sdk ? kArmSdkLoopOverrunThresholdMs : kLowCmdLoopOverrunThresholdMs; }

} // namespace g1_custom_control

#endif // G1_CUSTOM_CONTROL__G1_CONTROL_CONSTANTS_HPP_
