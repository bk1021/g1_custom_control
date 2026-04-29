#include "g1_custom_control/g1_control_constants.hpp"
#include "g1/motor_crc_hg.h"
#include "g1/g1_motion_switch_client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "unitree_hg/msg/low_cmd.hpp"
#include "unitree_hg/msg/low_state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <thread>

using namespace g1_custom_control;
using namespace std::chrono_literals;

namespace {
std::atomic<bool> g_sigint_requested{false};
void sigint_handler(int) { g_sigint_requested.store(true, std::memory_order_relaxed); }
} // namespace

namespace g1_custom_control {

struct SBMotorCommand {
    std::array<float, G1_NUM_MOTOR> q_target{};
    std::array<float, G1_NUM_MOTOR> dq_target{};
    std::array<float, G1_NUM_MOTOR> kp{};
    std::array<float, G1_NUM_MOTOR> kd{};
    std::array<float, G1_NUM_MOTOR> tau_ff{};
};

struct SBMotorState {
    std::array<float, G1_NUM_MOTOR> q{};
    std::array<float, G1_NUM_MOTOR> dq{};
    uint8_t mode_machine{0};
};

class G1ServoBridge : public rclcpp::Node {
public:
    G1ServoBridge();
    ~G1ServoBridge();
    void request_shutdown();
    bool is_shutdown_done() const;

private:
    void traj_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
    void lowstate_callback(const unitree_hg::msg::LowState::SharedPtr msg);
    void control_loop();
    void perform_shutdown();
    void populate_arm_sdk_message(const SBMotorCommand &cmd, unitree_hg::msg::LowCmd &out) const;
    void populate_low_cmd_message(
        const SBMotorCommand &cmd,
        const SBMotorState &state,
        unitree_hg::msg::LowCmd &out) const;

    bool use_arm_sdk_;
    std::array<float, G1_NUM_MOTOR> motor_kp_{};
    std::array<float, G1_NUM_MOTOR> motor_kd_{};

    rclcpp::CallbackGroup::SharedPtr lowstate_cb_group_;
    rclcpp::CallbackGroup::SharedPtr traj_cb_group_;
    rclcpp::CallbackGroup::SharedPtr control_cb_group_;

    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr left_traj_sub_;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr right_traj_sub_;
    rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr lowstate_sub_;
    rclcpp::Publisher<unitree_hg::msg::LowCmd>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    DataBuffer<SBMotorCommand> cmd_buffer_;
    DataBuffer<SBMotorState>   state_buffer_;

    std::atomic<bool> initial_q_captured_{false};
    std::array<float, G1_NUM_MOTOR> initial_q_{};

    std::atomic<float> control_weight_{0.0f};
    int startup_step_{0};
    std::atomic<bool> startup_done_{false};

    std::atomic<bool> shutdown_requested_{false};
    std::atomic<bool> shutdown_done_{false};
    int shutdown_step_{0};
    std::array<float, G1_NUM_MOTOR> shutdown_start_q_{};
    float shutdown_start_weight_{0.0f};

    // set true by init_thread_ once motion-service handshake is complete
    std::atomic<bool> mode_switch_ready_{false};
    std::thread init_thread_;

    std::chrono::steady_clock::time_point last_tick_{};
    bool timing_initialized_ = false;
};

G1ServoBridge::G1ServoBridge() : Node("g1_servo_bridge")
{
    for (int i = 0; i < G1_NUM_MOTOR; ++i) {
        motor_kp_[i] = GetMotorKp(G1MotorType[i]);
        motor_kd_[i] = GetMotorKd(G1MotorType[i]);
    }

    use_arm_sdk_ = this->declare_parameter<bool>("use_arm_sdk", true);

    lowstate_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    traj_cb_group_     = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    control_cb_group_  = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::QoS qos_sub(rclcpp::KeepLast(1));
    qos_sub.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

    rclcpp::SubscriptionOptions lowstate_opts;
    lowstate_opts.callback_group = lowstate_cb_group_;
    lowstate_sub_ = this->create_subscription<unitree_hg::msg::LowState>(
        "lowstate", qos_sub,
        std::bind(&G1ServoBridge::lowstate_callback, this, std::placeholders::_1),
        lowstate_opts);

    rclcpp::SubscriptionOptions traj_opts;
    traj_opts.callback_group = traj_cb_group_;
    left_traj_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
        "/left_arm_controller/joint_trajectory", 10,
        std::bind(&G1ServoBridge::traj_callback, this, std::placeholders::_1),
        traj_opts);
    right_traj_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
        "/right_arm_controller/joint_trajectory", 10,
        std::bind(&G1ServoBridge::traj_callback, this, std::placeholders::_1),
        traj_opts);

    rclcpp::QoS qos_pub(rclcpp::KeepLast(1));
    qos_pub.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    const std::string pub_topic = use_arm_sdk_ ? "arm_sdk" : "lowcmd";
    publisher_ = this->create_publisher<unitree_hg::msg::LowCmd>(pub_topic, qos_pub);
    joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    control_timer_ = this->create_wall_timer(
        control_period_for_mode(use_arm_sdk_),
        std::bind(&G1ServoBridge::control_loop, this),
        control_cb_group_);

    auto msclient = std::make_shared<unitree::robot::g1::MotionSwitchClient>(this);
    init_thread_ = std::thread([this, msclient]() {
        std::this_thread::sleep_for(1s);
        if (!use_arm_sdk_) {
            auto query_status = [&]() -> int {
                std::string form, name;
                return (msclient->CheckMode(form, name) == 0 && !name.empty()) ? 1 : 0;
            };
            int retries = 0;
            while (query_status() != 0) {
                if (++retries > 10) {
                    RCLCPP_FATAL(this->get_logger(), "Failed to release motion mode after 10 attempts.");
                    return;
                }
                const int32_t ret = msclient->ReleaseMode();
                RCLCPP_INFO(this->get_logger(), "ReleaseMode returned %d", ret);
                std::this_thread::sleep_for(2s);
            }
            RCLCPP_INFO(this->get_logger(), "Motion service released. Servo bridge ready (lowcmd).");
        } else {
            std::string form, name;
            if (msclient->CheckMode(form, name) == 0 && name.empty()) {
                RCLCPP_WARN(this->get_logger(),
                    "arm_sdk mode selected but no active motion service detected. "
                    "Ensure robot motion mode is enabled.");
            } else {
                RCLCPP_INFO(this->get_logger(), "arm_sdk mode: motion service active.");
            }
        }
        mode_switch_ready_.store(true, std::memory_order_release);
    });

    if (use_arm_sdk_) {
        RCLCPP_INFO(this->get_logger(), "g1_servo_bridge started in arm_sdk mode.");
    } else {
        RCLCPP_INFO(this->get_logger(), "g1_servo_bridge started in lowcmd mode.");
    }
}

G1ServoBridge::~G1ServoBridge()
{
    if (init_thread_.joinable()) init_thread_.join();
    perform_shutdown();
}

void G1ServoBridge::request_shutdown()
{
    if (shutdown_done_.load(std::memory_order_acquire)) return;

    if (auto state = state_buffer_.GetData()) {
        shutdown_start_q_ = state->q;
    } else {
        shutdown_start_q_ = initial_q_;
    }
    shutdown_start_weight_ = control_weight_.load(std::memory_order_relaxed);
    shutdown_step_ = 0;

    if (use_arm_sdk_) {
        RCLCPP_INFO(this->get_logger(), "Shutting down... Releasing control weight over 2 seconds...");
    } else {
        RCLCPP_INFO(this->get_logger(), "Shutting down... Homing arm joints over 2 seconds...");
    }
    shutdown_requested_.store(true, std::memory_order_release);
}

bool G1ServoBridge::is_shutdown_done() const
{
    return shutdown_done_.load(std::memory_order_acquire);
}

// Blocking fallback called from the destructor if the spin loop in main() did not
// complete shutdown (e.g. executor already stopped).
void G1ServoBridge::perform_shutdown()
{
    if (shutdown_done_.load(std::memory_order_acquire)) return;

    request_shutdown();

    if (shutdown_done_.load(std::memory_order_acquire)) return;
    if (!publisher_) {
        shutdown_done_.store(true, std::memory_order_release);
        return;
    }

    const int homing_steps = homing_steps_for_mode(use_arm_sdk_);
    const auto control_period = control_period_for_mode(use_arm_sdk_);

    SBMotorCommand cmd{};
    auto state_ptr = state_buffer_.GetData();

    for (int step = 0; step <= homing_steps; ++step) {
        const double ratio = static_cast<double>(step) / static_cast<double>(homing_steps);

        if (use_arm_sdk_) {
            for (int i = kArmSdkFirstJoint; i <= kArmSdkLastJoint; ++i) {
                cmd.q_target[i]  = shutdown_start_q_[i];
                cmd.dq_target[i] = 0.0f;
                cmd.kp[i] = is_waist_joint(i)
                    ? kArmSdkJointKp * kArmSdkWaistGainScale : kArmSdkJointKp;
                cmd.kd[i] = is_waist_joint(i)
                    ? kArmSdkJointKd * kArmSdkWaistGainScale : kArmSdkJointKd;
            }
            const float w = std::clamp(
                shutdown_start_weight_ * static_cast<float>(1.0 - ratio), 0.0f, 1.0f);
            control_weight_.store(w, std::memory_order_relaxed);
            unitree_hg::msg::LowCmd out{};
            populate_arm_sdk_message(cmd, out);
            publisher_->publish(out);
        } else {
            if (!state_ptr) break;
            for (int i = kArmSdkFirstJoint; i <= kArmSdkLastJoint; ++i) {
                cmd.q_target[i]  = static_cast<float>(shutdown_start_q_[i] * (1.0 - ratio));
                cmd.dq_target[i] = 0.0f;
                cmd.kp[i]  = motor_kp_[i];
                cmd.kd[i]  = motor_kd_[i];
                cmd.tau_ff[i] = 0.0f;
            }
            unitree_hg::msg::LowCmd out{};
            populate_low_cmd_message(cmd, *state_ptr, out);
            get_crc(out);
            publisher_->publish(out);
        }
        std::this_thread::sleep_for(control_period);
    }

    shutdown_done_.store(true, std::memory_order_release);
    if (use_arm_sdk_) {
        RCLCPP_INFO(this->get_logger(), "Control weight released. Exiting...");
    } else {
        RCLCPP_INFO(this->get_logger(), "Arm joints homed. Exiting...");
    }
}

void G1ServoBridge::lowstate_callback(const unitree_hg::msg::LowState::SharedPtr msg)
{
    const bool capture = !initial_q_captured_.load(std::memory_order_acquire);
    SBMotorState state;
    state.mode_machine = msg->mode_machine;
    for (int i = 0; i < G1_NUM_MOTOR; ++i) {
        state.q[i]  = msg->motor_state[i].q;
        state.dq[i] = msg->motor_state[i].dq;
        if (capture) initial_q_[i] = state.q[i];
    }
    state_buffer_.SetData(state);

    sensor_msgs::msg::JointState js_msg;
    js_msg.header.stamp = this->now();
    for (int i = 0; i < G1_NUM_MOTOR; ++i) {
        js_msg.name.push_back(JOINT_NAMES[i]);
        js_msg.position.push_back(state.q[i]);
        js_msg.velocity.push_back(state.dq[i]);
    }
    joint_state_publisher_->publish(js_msg);

    if (capture) {
        // Seed cmd_buffer_ with initial positions so control_loop has something to
        // publish before the first Servo trajectory message arrives.
        SBMotorCommand init_cmd;
        if (use_arm_sdk_) {
            for (int i = kArmSdkFirstJoint; i <= kArmSdkLastJoint; ++i) {
                init_cmd.q_target[i] = state.q[i];
                init_cmd.kp[i] = is_waist_joint(i)
                    ? kArmSdkJointKp * kArmSdkWaistGainScale
                    : kArmSdkJointKp;
                init_cmd.kd[i] = is_waist_joint(i)
                    ? kArmSdkJointKd * kArmSdkWaistGainScale
                    : kArmSdkJointKd;
            }
        } else {
            for (int i = 0; i < G1_NUM_MOTOR; ++i) {
                init_cmd.q_target[i] = state.q[i];
                init_cmd.kp[i] = motor_kp_[i];
                init_cmd.kd[i] = motor_kd_[i];
            }
        }
        cmd_buffer_.SetData(init_cmd);
        initial_q_captured_.store(true, std::memory_order_release);
        RCLCPP_INFO(this->get_logger(), "Initial joint state captured.");
    }
}

void G1ServoBridge::traj_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
    if (msg->points.empty()) return;
    if (!startup_done_.load(std::memory_order_acquire)) return;

    auto prev = cmd_buffer_.GetData();
    SBMotorCommand cmd = prev ? *prev : SBMotorCommand{};

    const auto &pt = msg->points.back();
    const bool has_vel = !pt.velocities.empty() &&
                         pt.velocities.size() == msg->joint_names.size();

    for (size_t j = 0; j < msg->joint_names.size(); ++j) {
        auto it = JOINT_NAME_TO_IDX.find(msg->joint_names[j]);
        if (it == JOINT_NAME_TO_IDX.end()) continue;
        const int idx = it->second;
        if (idx < kArmSdkFirstJoint) continue;
        if (j >= pt.positions.size()) continue;

        cmd.q_target[idx]  = static_cast<float>(pt.positions[j]);
        cmd.dq_target[idx] = has_vel ? static_cast<float>(pt.velocities[j]) : 0.0f;
        if (use_arm_sdk_) {
            cmd.kp[idx] = is_waist_joint(idx)
                ? kArmSdkJointKp * kArmSdkWaistGainScale : kArmSdkJointKp;
            cmd.kd[idx] = is_waist_joint(idx)
                ? kArmSdkJointKd * kArmSdkWaistGainScale : kArmSdkJointKd;
        } else {
            cmd.kp[idx] = motor_kp_[idx];
            cmd.kd[idx] = motor_kd_[idx];
        }
        cmd.tau_ff[idx] = 0.0f;
    }
    cmd_buffer_.SetData(cmd);
}

void G1ServoBridge::control_loop()
{
    if (!initial_q_captured_.load(std::memory_order_acquire)) return;
    if (!mode_switch_ready_.load(std::memory_order_acquire)) return;

    const auto tick_now = std::chrono::steady_clock::now();
    if (timing_initialized_) {
        const double dt_ms = std::chrono::duration<double, std::milli>(tick_now - last_tick_).count();
        const double threshold = loop_overrun_threshold_for_mode(use_arm_sdk_);
        if (dt_ms > threshold) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "control_loop period exceeded: %.3f ms > %.3f ms", dt_ms, threshold);
        }
    } else {
        timing_initialized_ = true;
    }
    last_tick_ = tick_now;

    auto cmd_ptr = cmd_buffer_.GetData();
    SBMotorCommand cmd = cmd_ptr ? *cmd_ptr : SBMotorCommand{};
    const int homing_steps = homing_steps_for_mode(use_arm_sdk_);

    // ---- Startup: home arm joints to 0; arm_sdk also ramps control_weight_ 0→1 ----
    if (!startup_done_.load(std::memory_order_acquire)) {
        if (shutdown_requested_.load(std::memory_order_acquire)) {
            control_weight_.store(0.0f, std::memory_order_relaxed);
            startup_done_.store(true, std::memory_order_release);
        } else {
            const float ratio = std::clamp(
                static_cast<float>(startup_step_) / static_cast<float>(homing_steps),
                0.0f, 1.0f);
            if (use_arm_sdk_) {
                control_weight_.store(ratio, std::memory_order_relaxed);
                for (int i = kArmSdkFirstJoint; i <= kArmSdkLastJoint; ++i) {
                    cmd.q_target[i]  = initial_q_[i] * (1.0f - ratio);
                    cmd.dq_target[i] = 0.0f;
                    cmd.kp[i] = is_waist_joint(i)
                        ? kArmSdkJointKp * kArmSdkWaistGainScale : kArmSdkJointKp;
                    cmd.kd[i] = is_waist_joint(i)
                        ? kArmSdkJointKd * kArmSdkWaistGainScale : kArmSdkJointKd;
                }
            } else {
                for (int i = kArmSdkFirstJoint; i <= kArmSdkLastJoint; ++i) {
                    cmd.q_target[i]  = initial_q_[i] * (1.0f - ratio);
                    cmd.dq_target[i] = 0.0f;
                    cmd.kp[i]  = motor_kp_[i];
                    cmd.kd[i]  = motor_kd_[i];
                    cmd.tau_ff[i] = 0.0f;
                }
            }
            if (startup_step_ >= homing_steps) {
                if (use_arm_sdk_) control_weight_.store(1.0f, std::memory_order_relaxed);
                cmd_buffer_.SetData(cmd);
                startup_done_.store(true, std::memory_order_release);
                RCLCPP_INFO(this->get_logger(),
                    use_arm_sdk_
                        ? "Startup homing complete (arm_sdk). Accepting Servo commands."
                        : "Startup homing complete (lowcmd). Accepting Servo commands.");
            } else {
                ++startup_step_;
            }
        }
    }

    // ---- Shutdown: ramp weight to 0 (arm_sdk) or home arms (lowcmd) ----
    if (shutdown_requested_.load(std::memory_order_acquire) &&
        !shutdown_done_.load(std::memory_order_acquire))
    {
        const double ratio = std::clamp(
            static_cast<double>(shutdown_step_) / static_cast<double>(homing_steps),
            0.0, 1.0);

        if (use_arm_sdk_) {
            const float w = std::clamp(
                shutdown_start_weight_ * static_cast<float>(1.0 - ratio),
                0.0f, 1.0f);
            control_weight_.store(w, std::memory_order_relaxed);
            for (int i = kArmSdkFirstJoint; i <= kArmSdkLastJoint; ++i) {
                cmd.q_target[i]  = shutdown_start_q_[i];
                cmd.dq_target[i] = 0.0f;
                cmd.kp[i] = is_waist_joint(i)
                    ? kArmSdkJointKp * kArmSdkWaistGainScale : kArmSdkJointKp;
                cmd.kd[i] = is_waist_joint(i)
                    ? kArmSdkJointKd * kArmSdkWaistGainScale : kArmSdkJointKd;
            }
        } else {
            for (int i = kArmSdkFirstJoint; i <= kArmSdkLastJoint; ++i) {
                cmd.q_target[i]  = static_cast<float>(shutdown_start_q_[i] * (1.0 - ratio));
                cmd.dq_target[i] = 0.0f;
                cmd.kp[i]  = motor_kp_[i];
                cmd.kd[i]  = motor_kd_[i];
                cmd.tau_ff[i] = 0.0f;
            }
        }

        if (shutdown_step_ >= homing_steps) {
            shutdown_done_.store(true, std::memory_order_release);
            if (use_arm_sdk_) {
                RCLCPP_INFO(this->get_logger(), "Control weight released. Exiting...");
            } else {
                RCLCPP_INFO(this->get_logger(), "Arm joints homed. Exiting...");
            }
        } else {
            ++shutdown_step_;
        }
    }

    // ---- Publish ----
    if (use_arm_sdk_) {
        unitree_hg::msg::LowCmd out{};
        populate_arm_sdk_message(cmd, out);
        publisher_->publish(out);
    } else {
        auto state_ptr = state_buffer_.GetData();
        if (!state_ptr) return;
        unitree_hg::msg::LowCmd out{};
        populate_low_cmd_message(cmd, *state_ptr, out);
        get_crc(out);
        publisher_->publish(out);
    }
}

void G1ServoBridge::populate_arm_sdk_message(
    const SBMotorCommand &cmd,
    unitree_hg::msg::LowCmd &out) const
{
    for (int i = kArmSdkFirstJoint; i <= kArmSdkLastJoint; ++i) {
        out.motor_cmd[i].q   = cmd.q_target[i];
        out.motor_cmd[i].dq  = cmd.dq_target[i];
        out.motor_cmd[i].tau = cmd.tau_ff[i];
        out.motor_cmd[i].kp  = cmd.kp[i];
        out.motor_cmd[i].kd  = cmd.kd[i];
    }
    out.motor_cmd[kArmSdkWeightJoint].q =
        std::clamp(control_weight_.load(std::memory_order_relaxed), 0.0f, 1.0f);
}

void G1ServoBridge::populate_low_cmd_message(
    const SBMotorCommand &cmd,
    const SBMotorState &state,
    unitree_hg::msg::LowCmd &out) const
{
    out.mode_pr = 0;
    out.mode_machine = state.mode_machine;
    for (int i = 0; i < G1_NUM_MOTOR; ++i) {
        out.motor_cmd[i].mode = 0x01;
        if (i < kArmSdkFirstJoint) {
            // Leg joints: hold at last known lowstate position — never jump to zero.
            out.motor_cmd[i].q   = state.q[i];
            out.motor_cmd[i].dq  = 0.0f;
            out.motor_cmd[i].tau = 0.0f;
            out.motor_cmd[i].kp  = motor_kp_[i];
            out.motor_cmd[i].kd  = motor_kd_[i];
        } else {
            out.motor_cmd[i].q   = cmd.q_target[i];
            out.motor_cmd[i].dq  = cmd.dq_target[i];
            out.motor_cmd[i].tau = cmd.tau_ff[i];
            out.motor_cmd[i].kp  = cmd.kp[i];
            out.motor_cmd[i].kd  = cmd.kd[i];
        }
    }
}

} // namespace g1_custom_control

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    std::signal(SIGINT, sigint_handler);

    auto node = std::make_shared<g1_custom_control::G1ServoBridge>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
    executor.add_node(node);

    while (rclcpp::ok() && !g_sigint_requested.load(std::memory_order_relaxed)) {
        executor.spin_some(std::chrono::milliseconds(2));
    }

    node->request_shutdown();
    while (!node->is_shutdown_done() && rclcpp::ok()) {
        executor.spin_some(std::chrono::milliseconds(2));
    }

    executor.remove_node(node);
    node.reset();

    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
    return 0;
}
