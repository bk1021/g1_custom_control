#include "g1_custom_control/g1_moveit_bridge.hpp"
#include "g1/motor_crc_hg.h"
#include "g1/g1_motion_switch_client.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include <algorithm>
#include <csignal>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <thread>

using namespace std::chrono_literals;

namespace {
std::atomic<bool> g_sigint_requested{false};

constexpr auto kControlPeriod = 2ms;
constexpr int kHomingSteps = 1000;
constexpr double kLoopOverrunThresholdMs = 2.2;
constexpr double kOnePointFallbackDurationSec = 0.2;
constexpr double kSettleDurationSec = 0.5;
constexpr double kGoalToleranceRad = 0.10;

enum class EvalMode {
    kCubicHermite,
    kLinear,
    kLinearDq0,
};

struct EvalConfig {
    EvalMode mode = EvalMode::kCubicHermite;
    std::string mode_name{"cubic_hermite"};
    bool enabled = true;
    int decimation = 5;
    std::string csv_path;
    std::string run_label{"default"};
};

struct EvalLogger {
    std::mutex mutex;
    std::ofstream csv;
    bool enabled = false;
    int decimation = 5;
    int sample_counter = 0;
    std::string mode_name{"cubic_hermite"};
    std::string run_label{"default"};
};

enum class EvalPhase {
    kStartupHoming,
    kWaitAfterHoming,
    kRequestMoveIt,
    kWaitMoveItDone,
    kRequestReturnHome,
    kWaitReturnHomeDone,
    kRequestShutdown,
    kDone,
};

struct EvalRuntime {
    EvalPhase phase = EvalPhase::kStartupHoming;
    bool phase_started = false;
    rclcpp::Time phase_start{0, 0, RCL_ROS_TIME};

    double wait_after_homing_sec = 5.0;
    double settle_before_shutdown_sec = 0.5;

    std::string move_group_name{"left_arm"};
    std::string named_target{"left_arm_relax"};
    double planning_time_sec = 5.0;
    std::string eval_arm{"left"};
    int arm_start_idx = 15;

    std::atomic<bool> moveit_request_started{false};
    std::atomic<bool> moveit_request_done{false};
    std::atomic<bool> moveit_request_success{false};
    std::atomic<bool> trajectory_seen{false};

    int mode_index = 0;
    EvalMode active_mode = EvalMode::kCubicHermite;
    std::string active_mode_name{"cubic_hermite"};
    std::string pending_named_target;
    std::array<float, 7> return_home_start_q{0};
};

EvalConfig g_eval_config;
EvalLogger g_eval_logger;
EvalRuntime g_eval_runtime;

int arm_start_index_from_name(const std::string &eval_arm)
{
    if (eval_arm == "left") {
        return 15;
    }
    if (eval_arm == "right") {
        return 22;
    }
    throw std::runtime_error("Unsupported eval_arm: '" + eval_arm + "'. Use left | right");
}

std::string make_default_csv_name(const std::string &eval_arm)
{
    const auto now = std::time(nullptr);
    std::tm tm_snapshot{};
    if (const auto *tm_ptr = std::localtime(&now)) {
        tm_snapshot = *tm_ptr;
    }

    std::ostringstream oss;
    oss << "g1_evaluate_" << eval_arm << "_" << std::put_time(&tm_snapshot, "%Y%m%d_%H%M%S") << ".csv";
    return oss.str();
}

const char *eval_phase_name(EvalPhase phase)
{
    switch (phase) {
    case EvalPhase::kStartupHoming:
        return "startup_homing";
    case EvalPhase::kWaitAfterHoming:
        return "wait_after_homing";
    case EvalPhase::kRequestMoveIt:
        return "request_moveit_plan";
    case EvalPhase::kWaitMoveItDone:
        return "wait_moveit_done";
    case EvalPhase::kRequestReturnHome:
        return "request_return_home";
    case EvalPhase::kWaitReturnHomeDone:
        return "wait_return_home_done";
    case EvalPhase::kRequestShutdown:
        return "request_shutdown";
    case EvalPhase::kDone:
        return "done";
    default:
        return "unknown";
    }
}

EvalMode parse_eval_mode(const std::string &mode)
{
    if (mode == "cubic_hermite") {
        return EvalMode::kCubicHermite;
    }
    if (mode == "linear") {
        return EvalMode::kLinear;
    }
    if (mode == "linear_dq0") {
        return EvalMode::kLinearDq0;
    }
    throw std::runtime_error("Unsupported evaluate_mode: '" + mode + "'. Use cubic_hermite | linear | linear_dq0");
}

constexpr std::array<EvalMode, 3> kAutoEvalModes{
    EvalMode::kCubicHermite,
    EvalMode::kLinear,
    EvalMode::kLinearDq0,
};

const char *mode_to_string(EvalMode mode)
{
    switch (mode) {
    case EvalMode::kCubicHermite:
        return "cubic_hermite";
    case EvalMode::kLinear:
        return "linear";
    case EvalMode::kLinearDq0:
        return "linear_dq0";
    default:
        return "unknown";
    }
}

void sigint_handler(int)
{
    g_sigint_requested.store(true, std::memory_order_relaxed);
}
} // namespace

namespace g1_custom_control {

void G1MoveItBridge::request_shutdown_homing()
{
    if (shutdown_homing_done_.load(std::memory_order_acquire)) {
        return;
    }

    shutdown_requested_.store(true, std::memory_order_release);
    {
        std::lock_guard<std::mutex> lock(trajectory_mutex_);
        trajectory_rt_ = TrajectoryRuntime{};
    }

    {
        std::lock_guard<std::mutex> lock(shutdown_homing_mutex_);
        if (shutdown_homing_done_.load(std::memory_order_acquire) ||
            shutdown_homing_active_.load(std::memory_order_acquire)) {
            return;
        }

        if (auto state_snapshot = motor_state_buffer_.GetData()) {
            shutdown_homing_start_q_ = state_snapshot->q;
        } else {
            shutdown_homing_start_q_ = initial_q_;
        }
        shutdown_homing_step_.store(0, std::memory_order_relaxed);
        shutdown_homing_active_.store(true, std::memory_order_release);
        RCLCPP_INFO(this->get_logger(), "Shutting down... Homing all joints over 2 seconds...");
    }
}

bool G1MoveItBridge::is_shutdown_homing_done() const
{
    return shutdown_homing_done_.load(std::memory_order_acquire);
}

G1MoveItBridge::G1MoveItBridge() : Node("g1_evaluate")
{
    const auto mode_name = this->declare_parameter<std::string>("evaluate_mode", "cubic_hermite");
    g_eval_config.mode = parse_eval_mode(mode_name);
    g_eval_config.mode_name = mode_name;
    g_eval_config.enabled = this->declare_parameter<bool>("collect_data", true);
    g_eval_config.decimation = std::max(1, this->declare_parameter<int>("collect_every_n", 5));
    g_eval_config.run_label = this->declare_parameter<std::string>("run_label", "default");

    g_eval_runtime.wait_after_homing_sec = this->declare_parameter<double>("wait_after_homing_sec", 5.0);
    g_eval_runtime.settle_before_shutdown_sec = this->declare_parameter<double>("settle_before_shutdown_sec", 0.5);
    g_eval_runtime.eval_arm = this->declare_parameter<std::string>("eval_arm", "left");
    g_eval_runtime.arm_start_idx = arm_start_index_from_name(g_eval_runtime.eval_arm);
    g_eval_runtime.move_group_name = g_eval_runtime.eval_arm + "_arm";
    g_eval_runtime.named_target = g_eval_runtime.eval_arm + "_arm_relax";
    g_eval_runtime.planning_time_sec = this->declare_parameter<double>("planning_time_sec", 5.0);

    const std::string default_csv = make_default_csv_name(g_eval_runtime.eval_arm);
    g_eval_config.csv_path = this->declare_parameter<std::string>("csv_path", default_csv);

    g_eval_runtime.mode_index = 0;
    g_eval_runtime.active_mode = kAutoEvalModes[0];
    g_eval_runtime.active_mode_name = mode_to_string(g_eval_runtime.active_mode);
    g_eval_config.mode = g_eval_runtime.active_mode;
    g_eval_config.mode_name = g_eval_runtime.active_mode_name;

    if (g_eval_config.enabled) {
        std::filesystem::path out_path(g_eval_config.csv_path);
        if (out_path.has_parent_path()) {
            std::filesystem::create_directories(out_path.parent_path());
        }
        {
            std::lock_guard<std::mutex> lock(g_eval_logger.mutex);
            g_eval_logger.csv.open(g_eval_config.csv_path, std::ios::out | std::ios::trunc);
            if (!g_eval_logger.csv.is_open()) {
                throw std::runtime_error("Failed to open csv_path for evaluation logging: " + g_eval_config.csv_path);
            }
            g_eval_logger.enabled = true;
            g_eval_logger.decimation = g_eval_config.decimation;
            g_eval_logger.mode_name = g_eval_config.mode_name;
            g_eval_logger.run_label = g_eval_config.run_label;

            g_eval_logger.csv
                << "stamp_ns,run_label,mode,trajectory_active";
            for (int i = g_eval_runtime.arm_start_idx; i < g_eval_runtime.arm_start_idx + 7; ++i) {
                g_eval_logger.csv
                    << ",cmd_q_" << JOINT_NAMES[i]
                    << ",cmd_dq_" << JOINT_NAMES[i]
                    << ",act_q_" << JOINT_NAMES[i]
                    << ",act_dq_" << JOINT_NAMES[i];
            }
            g_eval_logger.csv << '\n';
        }
    }

    RCLCPP_INFO(
        this->get_logger(),
        "Auto-evaluate arm=%s modes=[cubic_hermite,linear,linear_dq0], data collection=%s, every_n=%d, csv='%s', label='%s', flow='startup_homing->wait(%.1fs)->(relax+return_home)*3->shutdown'",
        g_eval_runtime.eval_arm.c_str(),
        g_eval_config.enabled ? "on" : "off",
        g_eval_config.decimation,
        g_eval_config.csv_path.c_str(),
        g_eval_config.run_label.c_str(),
        g_eval_runtime.wait_after_homing_sec);

    for (int i = 0; i < G1_NUM_MOTOR; ++i) {
        motor_kp_[i] = GetMotorKp(G1MotorType[i]);
        motor_kd_[i] = GetMotorKd(G1MotorType[i]);
    }

    msclient_ = std::make_shared<unitree::robot::g1::MotionSwitchClient>(this);

    // Defer motion switcher check and ROS interface creation to a background
    // thread so that the executor is already spinning when BaseClient::Call()
    // publishes a request and waits for a subscription callback.
    init_thread_ = std::thread([this]() {
        std::this_thread::sleep_for(1s);

        int retries = 0;
        while (queryMotionStatus() != 0) 
        {
            if (++retries > 10) {
                RCLCPP_FATAL(this->get_logger(), "Failed to release motion mode after 10 attempts");
                return;
            }
            std::cout << "Try to deactivate the motion control-related service." << std::endl;
            int32_t ret = msclient_->ReleaseMode();
            if (ret == 0) {
                std::cout << "ReleaseMode succeeded." << std::endl;
            } 
            else {
                std::cout << "ReleaseMode failed. Error code: " << ret << std::endl;
            }
            std::this_thread::sleep_for(2s);
        }

        lowstate_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        command_writer_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        control_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

        rclcpp::QoS qos_sub(rclcpp::KeepLast(1));
        qos_sub.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        rclcpp::SubscriptionOptions lowstate_sub_options;
        lowstate_sub_options.callback_group = lowstate_callback_group_;
        lowstate_subscriber_ = this->create_subscription<unitree_hg::msg::LowState>(
            "lowstate", qos_sub, std::bind(&G1MoveItBridge::lowstate_callback, this, std::placeholders::_1), lowstate_sub_options);

        rclcpp::QoS qos_pub(rclcpp::KeepLast(1));
        qos_pub.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        lowcmd_publisher_ = this->create_publisher<unitree_hg::msg::LowCmd>("lowcmd", qos_pub);
        command_writer_timer_ = this->create_wall_timer(
            kControlPeriod,
            std::bind(&G1MoveItBridge::command_writer_loop, this),
            command_writer_callback_group_);
        control_timer_ = this->create_wall_timer(
            kControlPeriod,
            std::bind(&G1MoveItBridge::control_loop, this),
            control_callback_group_);

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
    });
}

void G1MoveItBridge::lowstate_callback(const unitree_hg::msg::LowState::SharedPtr msg)
{
    sensor_msgs::msg::JointState joint_state_msg;
    joint_state_msg.header.stamp = this->now();
    const bool capture_initial_state = !initial_q_captured_.load(std::memory_order_acquire);
    MotorState motor_state;

    mode_machine_.store(msg->mode_machine, std::memory_order_relaxed);
    for (int i = 0; i < G1_NUM_MOTOR; i++) {
        motor_state.q[i] = msg->motor_state[i].q;
        motor_state.dq[i] = msg->motor_state[i].dq;

        joint_state_msg.name.push_back(JOINT_NAMES[i]);
        joint_state_msg.position.push_back(motor_state.q[i]);
        joint_state_msg.velocity.push_back(motor_state.dq[i]);

        if (capture_initial_state) {
            initial_q_[i] = motor_state.q[i];
        }
    }

    motor_state_buffer_.SetData(motor_state);

    if (capture_initial_state) {
        initial_q_captured_.store(true, std::memory_order_release);
        RCLCPP_INFO(this->get_logger(), "Initial joint state captured. Start homing all joints to zero...");

        MotorCommand init_cmd;
        for (int i = 0; i < G1_NUM_MOTOR; ++i) {
            init_cmd.q_target[i] = motor_state.q[i];
            init_cmd.dq_target[i] = 0.0f;
            init_cmd.kp[i] = motor_kp_[i];
            init_cmd.kd[i] = motor_kd_[i];
            init_cmd.tau_ff[i] = 0.0f;
        }
        motor_command_buffer_.SetData(init_cmd);
    }

    joint_state_publisher_->publish(joint_state_msg);
}

void G1MoveItBridge::command_writer_loop()
{
    if (!initial_q_captured_.load(std::memory_order_acquire)) return;

    const auto tick_now = std::chrono::steady_clock::now();
    double loop_period_ms = -1.0;
    if (command_writer_timing_initialized_) {
        loop_period_ms = std::chrono::duration<double, std::milli>(tick_now - command_writer_last_tick_).count();
    } else {
        command_writer_timing_initialized_ = true;
    }
    command_writer_last_tick_ = tick_now;

    auto previous_command = motor_command_buffer_.GetData();

    MotorCommand command;
    if (previous_command) {
        command = *previous_command;
    } else if (auto state = motor_state_buffer_.GetData()) {
        command.q_target = state->q;
        command.dq_target.fill(0.0f);
    }

    bool trajectory_active_for_log = false;
    {
        std::lock_guard<std::mutex> lock(trajectory_mutex_);
        const bool is_relax_phase = (g_eval_runtime.phase == EvalPhase::kWaitMoveItDone);
        trajectory_active_for_log = trajectory_rt_.active && is_relax_phase;
    }

    if (shutdown_homing_active_.load(std::memory_order_acquire)) {
        const int step = shutdown_homing_step_.load(std::memory_order_acquire);
        const double ratio = std::clamp(
            static_cast<double>(step) / static_cast<double>(kHomingSteps),
            0.0,
            1.0);

        for (int i = 0; i < G1_NUM_MOTOR; ++i) {
            command.q_target[i] = static_cast<float>(shutdown_homing_start_q_[i] * (1.0 - ratio));
            command.dq_target[i] = 0.0f;
            command.kp[i] = motor_kp_[i];
            command.kd[i] = motor_kd_[i];
            command.tau_ff[i] = 0.0f;
        }

        if (IsCommandChanged(previous_command, command)) {
            motor_command_buffer_.SetData(command);
        }

        if (step >= kHomingSteps) {
            shutdown_homing_active_.store(false, std::memory_order_release);
            shutdown_homing_done_.store(true, std::memory_order_release);
            RCLCPP_INFO(this->get_logger(), "Returned to home. Exiting...");
        } else {
            shutdown_homing_step_.store(step + 1, std::memory_order_release);
        }
    } else {
        for (int i = 0; i < G1_NUM_MOTOR; ++i) {
            command.kp[i] = motor_kp_[i];
            command.kd[i] = motor_kd_[i];
            command.tau_ff[i] = 0.0f;
        }

        if (startup_homing_ratio_ < 1.0) {
            // Startup homing: blend ALL joints from initial position to zero.
            for (int i = 0; i < G1_NUM_MOTOR; ++i) {
                command.q_target[i] = static_cast<float>(initial_q_[i] * (1.0 - startup_homing_ratio_));
                command.dq_target[i] = 0.0f;
            }
            startup_homing_ratio_ += (1.0 / static_cast<double>(kHomingSteps));
            if (startup_homing_ratio_ > 1.0) startup_homing_ratio_ = 1.0;
        } else if (!startup_homing_done_.load(std::memory_order_acquire)) {
            // Final tick: clamp all joints to exact zero and mark done.
            for (int i = 0; i < G1_NUM_MOTOR; ++i) {
                command.q_target[i] = 0.0f;
                command.dq_target[i] = 0.0f;
            }
            startup_homing_done_.store(true, std::memory_order_release);
            g_eval_runtime.phase = EvalPhase::kWaitAfterHoming;
            g_eval_runtime.phase_started = false;
            RCLCPP_INFO(this->get_logger(), "Startup homing complete. Begin evaluation flow.");
        } else {
            // Autonomous evaluation flow:
            // startup homing -> wait -> for each mode: relax -> return home -> shutdown homing.
            const auto now = this->now();
            for (int i = 0; i < G1_NUM_MOTOR; ++i) {
                command.q_target[i] = 0.0f;
                command.dq_target[i] = 0.0f;
            }

            if (g_eval_runtime.phase == EvalPhase::kWaitAfterHoming) {
                if (!g_eval_runtime.phase_started) {
                    g_eval_runtime.phase_started = true;
                    g_eval_runtime.phase_start = now;
                    RCLCPP_INFO(this->get_logger(), "Evaluation phase: %s", eval_phase_name(g_eval_runtime.phase));
                }
                if ((now - g_eval_runtime.phase_start).seconds() >= g_eval_runtime.wait_after_homing_sec) {
                    g_eval_runtime.phase = EvalPhase::kRequestMoveIt;
                    g_eval_runtime.phase_started = false;
                }
            } else if (g_eval_runtime.phase == EvalPhase::kRequestMoveIt) {
                if (!g_eval_runtime.phase_started) {
                    g_eval_runtime.phase_started = true;
                    g_eval_runtime.phase_start = now;
                    RCLCPP_INFO(this->get_logger(), "Evaluation phase: %s", eval_phase_name(g_eval_runtime.phase));
                }

                if (!g_eval_runtime.moveit_request_started.exchange(true, std::memory_order_acq_rel)) {
                    g_eval_runtime.pending_named_target = g_eval_runtime.named_target;
                    g_eval_runtime.active_mode = kAutoEvalModes[g_eval_runtime.mode_index];
                    g_eval_runtime.active_mode_name = mode_to_string(g_eval_runtime.active_mode);
                    g_eval_config.mode = g_eval_runtime.active_mode;
                    g_eval_config.mode_name = g_eval_runtime.active_mode_name;

                    {
                        std::lock_guard<std::mutex> log_lock(g_eval_logger.mutex);
                        g_eval_logger.mode_name = g_eval_runtime.active_mode_name;
                    }

                    RCLCPP_INFO(
                        this->get_logger(),
                        "Running mode %d/3: %s -> named target '%s'",
                        g_eval_runtime.mode_index + 1,
                        g_eval_runtime.active_mode_name.c_str(),
                        g_eval_runtime.pending_named_target.c_str());

                    std::thread([this]() {
                        bool ok = false;
                        try {
                            auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                                shared_from_this(),
                                g_eval_runtime.move_group_name);
                            move_group->setPlanningTime(g_eval_runtime.planning_time_sec);
                            move_group->setStartStateToCurrentState();
                            move_group->setNamedTarget(g_eval_runtime.pending_named_target);

                            moveit::planning_interface::MoveGroupInterface::Plan plan;
                            const bool plan_ok =
                                (move_group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
                            if (!plan_ok) {
                                RCLCPP_ERROR(
                                    this->get_logger(),
                                    "MoveIt plan failed for named target '%s' in group '%s'.",
                                    g_eval_runtime.pending_named_target.c_str(),
                                    g_eval_runtime.move_group_name.c_str());
                            } else {
                                const bool exec_ok =
                                    (move_group->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
                                if (!exec_ok) {
                                    RCLCPP_ERROR(this->get_logger(), "MoveIt execute failed.");
                                }
                                ok = exec_ok;
                            }
                        } catch (const std::exception &e) {
                            RCLCPP_ERROR(this->get_logger(), "MoveIt request thread exception: %s", e.what());
                        }

                        g_eval_runtime.moveit_request_success.store(ok, std::memory_order_release);
                        g_eval_runtime.moveit_request_done.store(true, std::memory_order_release);
                    }).detach();
                }
                g_eval_runtime.phase = EvalPhase::kWaitMoveItDone;
                g_eval_runtime.phase_started = false;
            } else if (g_eval_runtime.phase == EvalPhase::kWaitMoveItDone) {
                if (!g_eval_runtime.phase_started) {
                    g_eval_runtime.phase_started = true;
                    g_eval_runtime.phase_start = now;
                    RCLCPP_INFO(this->get_logger(), "Evaluation phase: %s", eval_phase_name(g_eval_runtime.phase));
                }

                if (g_eval_runtime.moveit_request_done.load(std::memory_order_acquire)) {
                    if (g_eval_runtime.moveit_request_success.load(std::memory_order_acquire)) {
                        g_eval_runtime.moveit_request_started.store(false, std::memory_order_release);
                        g_eval_runtime.moveit_request_done.store(false, std::memory_order_release);
                        g_eval_runtime.moveit_request_success.store(false, std::memory_order_release);
                        g_eval_runtime.phase = EvalPhase::kRequestReturnHome;
                        g_eval_runtime.phase_started = false;
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "MoveIt planning/execution failed. Requesting shutdown homing.");
                        request_shutdown_homing();
                        g_eval_runtime.phase = EvalPhase::kDone;
                        g_eval_runtime.phase_started = false;
                    }
                }
            } else if (g_eval_runtime.phase == EvalPhase::kRequestReturnHome) {
                if (!g_eval_runtime.phase_started) {
                    g_eval_runtime.phase_started = true;
                    g_eval_runtime.phase_start = now;
                    RCLCPP_INFO(this->get_logger(), "Evaluation phase: %s", eval_phase_name(g_eval_runtime.phase));
                    if (auto state_snapshot = motor_state_buffer_.GetData()) {
                        for (int j = 0; j < 7; ++j) {
                            g_eval_runtime.return_home_start_q[j] = state_snapshot->q[g_eval_runtime.arm_start_idx + j];
                        }
                    } else {
                        for (int j = 0; j < 7; ++j) {
                            g_eval_runtime.return_home_start_q[j] = command.q_target[g_eval_runtime.arm_start_idx + j];
                        }
                    }
                }
                g_eval_runtime.phase = EvalPhase::kWaitReturnHomeDone;
                g_eval_runtime.phase_started = false;
            } else if (g_eval_runtime.phase == EvalPhase::kWaitReturnHomeDone) {
                if (!g_eval_runtime.phase_started) {
                    g_eval_runtime.phase_started = true;
                    g_eval_runtime.phase_start = now;
                    RCLCPP_INFO(this->get_logger(), "Evaluation phase: %s", eval_phase_name(g_eval_runtime.phase));
                }

                const double elapsed = (now - g_eval_runtime.phase_start).seconds();
                const double return_home_duration_sec =
                    static_cast<double>(kHomingSteps) *
                    std::chrono::duration<double>(kControlPeriod).count();
                const double ratio = std::clamp(elapsed / return_home_duration_sec, 0.0, 1.0);

                for (int j = 0; j < 7; ++j) {
                    const int motor_id = g_eval_runtime.arm_start_idx + j;
                    command.q_target[motor_id] = static_cast<float>(g_eval_runtime.return_home_start_q[j] * (1.0 - ratio));
                    command.dq_target[motor_id] = 0.0f;
                }

                if (elapsed >= return_home_duration_sec) {
                    g_eval_runtime.mode_index += 1;
                    if (g_eval_runtime.mode_index >= static_cast<int>(kAutoEvalModes.size())) {
                        g_eval_runtime.phase = EvalPhase::kRequestShutdown;
                    } else {
                        g_eval_runtime.phase = EvalPhase::kRequestMoveIt;
                    }
                    g_eval_runtime.phase_started = false;
                }
            } else if (g_eval_runtime.phase == EvalPhase::kRequestShutdown) {
                if (!g_eval_runtime.phase_started) {
                    g_eval_runtime.phase_started = true;
                    g_eval_runtime.phase_start = now;
                    RCLCPP_INFO(this->get_logger(), "Evaluation phase: %s", eval_phase_name(g_eval_runtime.phase));
                }

                if ((now - g_eval_runtime.phase_start).seconds() >= g_eval_runtime.settle_before_shutdown_sec) {
                    request_shutdown_homing();
                    g_eval_runtime.phase = EvalPhase::kDone;
                    g_eval_runtime.phase_started = false;
                    RCLCPP_INFO(this->get_logger(), "Evaluation flow complete. Triggered shutdown homing.");
                }
            }
        }

        if (IsCommandChanged(previous_command, command)) {
            motor_command_buffer_.SetData(command);
        }
    }

    if (g_eval_logger.enabled) {
        std::lock_guard<std::mutex> lock(g_eval_logger.mutex);
        if (g_eval_logger.csv.is_open()) {
            g_eval_logger.sample_counter++;
            if (g_eval_logger.decimation <= 1 ||
                (g_eval_logger.sample_counter % g_eval_logger.decimation) == 0) {
                const auto state = motor_state_buffer_.GetData();
                g_eval_logger.csv
                    << this->now().nanoseconds() << ','
                    << g_eval_logger.run_label << ','
                    << g_eval_logger.mode_name << ','
                    << (trajectory_active_for_log ? 1 : 0);

                for (int i = g_eval_runtime.arm_start_idx; i < g_eval_runtime.arm_start_idx + 7; ++i) {
                    g_eval_logger.csv
                        << ',' << command.q_target[i]
                        << ',' << command.dq_target[i]
                        << ',' << (state ? state->q[i] : 0.0f)
                        << ',' << (state ? state->dq[i] : 0.0f);
                }
                g_eval_logger.csv << '\n';
            }
        }
    }

    if (loop_period_ms >= 0.0 && loop_period_ms > kLoopOverrunThresholdMs) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "command_writer_loop period exceeded threshold: %.3f ms > %.3f ms",
            loop_period_ms,
            kLoopOverrunThresholdMs);
    }
}

void G1MoveItBridge::control_loop() 
{
    if (!initial_q_captured_.load(std::memory_order_acquire)) return;

    const auto tick_now = std::chrono::steady_clock::now();
    double loop_period_ms = -1.0;
    if (control_timing_initialized_) {
        loop_period_ms = std::chrono::duration<double, std::milli>(tick_now - control_last_tick_).count();
    } else {
        control_timing_initialized_ = true;
    }
    control_last_tick_ = tick_now;

    auto command = motor_command_buffer_.GetData();
    if (!command) return;

    unitree_hg::msg::LowCmd low_cmd;
    low_cmd.mode_pr = mode_pr_;
    low_cmd.mode_machine = mode_machine_.load(std::memory_order_relaxed);

    for (int i = 0; i < G1_NUM_MOTOR; i++) {
        low_cmd.motor_cmd[i].mode = 0x01;
        low_cmd.motor_cmd[i].q = command->q_target[i];
        low_cmd.motor_cmd[i].dq = command->dq_target[i];
        low_cmd.motor_cmd[i].tau = command->tau_ff[i];
        low_cmd.motor_cmd[i].kp = command->kp[i];
        low_cmd.motor_cmd[i].kd = command->kd[i];
    }

    get_crc(low_cmd);
    lowcmd_publisher_->publish(low_cmd);

    if (loop_period_ms >= 0.0 && loop_period_ms > kLoopOverrunThresholdMs) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "control_loop period exceeded threshold: %.3f ms > %.3f ms",
            loop_period_ms,
            kLoopOverrunThresholdMs);
    }
}

rclcpp_action::GoalResponse G1MoveItBridge::handle_goal(
    const rclcpp_action::GoalUUID&, 
    std::shared_ptr<const FollowJointTrajectory::Goal> goal) 
{
    if (!initial_q_captured_.load(std::memory_order_acquire)) {
        RCLCPP_WARN(this->get_logger(), "Rejecting goal: no robot state received yet.");
        return rclcpp_action::GoalResponse::REJECT;
    }
    if (!startup_homing_done_.load(std::memory_order_acquire)) {
        RCLCPP_WARN(this->get_logger(), "Rejecting goal: startup homing to zero posture still in progress.");
        return rclcpp_action::GoalResponse::REJECT;
    }
    if (shutdown_requested_.load()) {
        RCLCPP_WARN(this->get_logger(), "Rejecting trajectory goal: node is shutting down.");
        return rclcpp_action::GoalResponse::REJECT;
    }

    std::string error_msg;
    if (!validate_goal_joints(*goal, error_msg)) {
        RCLCPP_WARN(this->get_logger(), "Rejecting trajectory goal: %s", error_msg.c_str());
        return rclcpp_action::GoalResponse::REJECT;
    }

    bool trajectory_busy = false;
    {
        std::lock_guard<std::mutex> lock(trajectory_mutex_);
        if (trajectory_rt_.active) {
            trajectory_busy = true;
        } else {
            trajectory_rt_ = TrajectoryRuntime{};
            trajectory_rt_.active = true;
        }
    }

    if (trajectory_busy) {
        RCLCPP_WARN(this->get_logger(), "Rejecting trajectory goal: another arm trajectory is running.");
        return rclcpp_action::GoalResponse::REJECT;
    }

    g_eval_runtime.trajectory_seen.store(true, std::memory_order_release);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse G1MoveItBridge::handle_cancel(const std::shared_ptr<GoalHandleFJT> goal_handle) 
{
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    if (!trajectory_rt_.active || !trajectory_rt_.goal_handle) {
        return rclcpp_action::CancelResponse::REJECT;
    }

    if (trajectory_rt_.goal_handle->get_goal_id() != goal_handle->get_goal_id()) {
        return rclcpp_action::CancelResponse::REJECT;
    }

    trajectory_rt_.cancel_requested = true;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void G1MoveItBridge::handle_accepted(const std::shared_ptr<GoalHandleFJT> goal_handle) 
{
    try {
        const auto goal = goal_handle->get_goal();
        std::vector<int> mapped_indices;
        mapped_indices.reserve(goal->trajectory.joint_names.size());
        for (const auto &name : goal->trajectory.joint_names) {
            mapped_indices.push_back(JOINT_NAME_TO_IDX.at(name));
        }

        std::array<float, G1_NUM_MOTOR> start_q{0};
        auto state_snapshot = motor_state_buffer_.GetData();
        if (!state_snapshot) {
            throw std::runtime_error("No motor state snapshot available for accepted trajectory goal.");
        }
        start_q = state_snapshot->q;

        std::lock_guard<std::mutex> lock(trajectory_mutex_);
        if (!trajectory_rt_.active) {
            auto result = std::make_shared<FollowJointTrajectory::Result>();
            result->error_code = FollowJointTrajectory::Result::INVALID_GOAL;
            goal_handle->abort(result);
            return;
        }

        trajectory_rt_.goal_handle = goal_handle;
        trajectory_rt_.goal = goal;
        trajectory_rt_.mapped_indices = std::move(mapped_indices);
        trajectory_rt_.actual_start_q = start_q;
        trajectory_rt_.start_time = this->now();
        trajectory_rt_.settle_start_time = trajectory_rt_.start_time;
        trajectory_rt_.segment_index = 1;
        trajectory_rt_.cancel_requested = false;
        trajectory_rt_.settle_phase = false;
        trajectory_rt_.has_segment_vel = false;

        const auto &traj = goal->trajectory;
        if (traj.points.size() == 1) {
            double one_point_duration = rclcpp::Duration(traj.points.front().time_from_start).seconds();
            if (one_point_duration <= 0.0) {
                one_point_duration = kOnePointFallbackDurationSec;
            }
            trajectory_rt_.one_point_duration = one_point_duration;
        } else {
            trajectory_rt_.one_point_duration = 0.0;

            const auto &first_point = traj.points[0];
            const auto &second_point = traj.points[1];
            const size_t expected = trajectory_rt_.mapped_indices.size();

            const bool has_segment_vel =
                (first_point.velocities.size() == expected) &&
                (second_point.velocities.size() == expected);
            trajectory_rt_.has_segment_vel = has_segment_vel;

            if (has_segment_vel) {
                std::ostringstream prev_vel;
                std::ostringstream target_vel;
                prev_vel << std::fixed << std::setprecision(4) << "[";
                target_vel << std::fixed << std::setprecision(4) << "[";
                for (size_t j = 0; j < expected; ++j) {
                    if (j > 0) {
                        prev_vel << ", ";
                        target_vel << ", ";
                    }
                    prev_vel << first_point.velocities[j];
                    target_vel << second_point.velocities[j];
                }
                prev_vel << "]";
                target_vel << "]";

                RCLCPP_INFO(
                    this->get_logger(),
                    "Trajectory first-segment velocity arrays: prev=%s target=%s",
                    prev_vel.str().c_str(),
                    target_vel.str().c_str());
            } else {
                RCLCPP_WARN(
                    this->get_logger(),
                    "Velocity fallback active for this goal: first segment velocity size mismatch (prev=%zu, target=%zu, expected=%zu). Using linear position interpolation + inferred dq for all segments.",
                    first_point.velocities.size(),
                    second_point.velocities.size(),
                    expected);
            }
        }
    } catch (const std::exception &e) {
        {
            std::lock_guard<std::mutex> lock(trajectory_mutex_);
            trajectory_rt_ = TrajectoryRuntime{};
        }
        auto result = std::make_shared<FollowJointTrajectory::Result>();
        result->error_code = FollowJointTrajectory::Result::INVALID_GOAL;
        goal_handle->abort(result);
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize trajectory runtime: %s", e.what());
    } catch (...) {
        {
            std::lock_guard<std::mutex> lock(trajectory_mutex_);
            trajectory_rt_ = TrajectoryRuntime{};
        }
        auto result = std::make_shared<FollowJointTrajectory::Result>();
        result->error_code = FollowJointTrajectory::Result::INVALID_GOAL;
        goal_handle->abort(result);
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize trajectory runtime due to unknown error.");
    }
}

bool G1MoveItBridge::validate_goal_joints(
    const FollowJointTrajectory::Goal &goal,
    std::string &error_msg) const
{
    const auto &traj = goal.trajectory;
    const size_t num_joints = traj.joint_names.size();

    if (num_joints == 0) {
        error_msg = "trajectory has no joint names";
        return false;
    }

    for (const auto &name : traj.joint_names) {
        auto it = JOINT_NAME_TO_IDX.find(name);
        if (it == JOINT_NAME_TO_IDX.end()) {
            error_msg = "unknown joint name: " + name;
            return false;
        }

        if (it->second < 15) {
            error_msg = "lower-body joint command is not allowed: " + name;
            return false;
        }
    }

    if (traj.points.empty()) {
        error_msg = "trajectory has no points";
        return false;
    }

    double prev_time_from_start = -1.0;
    for (size_t i = 0; i < traj.points.size(); ++i) {
        const auto &pt = traj.points[i];
        if (pt.positions.size() != num_joints) {
            error_msg = "point[" + std::to_string(i) + "] positions size mismatch";
            return false;
        }

        const double point_time_from_start = rclcpp::Duration(pt.time_from_start).seconds();
        if (i > 0 && point_time_from_start <= prev_time_from_start) {
            error_msg = "time_from_start must be strictly increasing";
            return false;
        }
        prev_time_from_start = point_time_from_start;
    }

    return true;
}

bool G1MoveItBridge::snapshot_trajectory_runtime(TrajectoryRuntime &snapshot)
{
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    if (!trajectory_rt_.active) {
        return false;
    }

    if (!trajectory_rt_.goal_handle || !trajectory_rt_.goal) {
        trajectory_rt_ = TrajectoryRuntime{};
        return false;
    }

    snapshot = trajectory_rt_;
    return true;
}

void G1MoveItBridge::persist_trajectory_runtime(
    const std::shared_ptr<GoalHandleFJT> &goal_handle,
    bool settle_phase,
    size_t segment_index,
    const rclcpp::Time &settle_start_time)
{
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    if (!trajectory_rt_.active || trajectory_rt_.goal_handle != goal_handle) {
        return;
    }

    trajectory_rt_.settle_phase = settle_phase;
    trajectory_rt_.segment_index = segment_index;
    trajectory_rt_.settle_start_time = settle_start_time;
}

void G1MoveItBridge::finish_trajectory_goal(
    const std::shared_ptr<GoalHandleFJT> &goal_handle,
    int32_t error_code,
    bool canceled,
    const char *msg)
{
    {
        std::lock_guard<std::mutex> lock(trajectory_mutex_);
        if (trajectory_rt_.active && trajectory_rt_.goal_handle == goal_handle) {
            trajectory_rt_ = TrajectoryRuntime{};
        }
    }

    auto result = std::make_shared<FollowJointTrajectory::Result>();
    result->error_code = error_code;
    if (goal_handle && goal_handle->is_active()) {
        if (canceled) {
            goal_handle->canceled(result);
        } else if (error_code == FollowJointTrajectory::Result::SUCCESSFUL) {
            goal_handle->succeed(result);
        } else {
            goal_handle->abort(result);
        }
    }
    RCLCPP_INFO(this->get_logger(), "%s", msg);
}

void G1MoveItBridge::run_trajectory_state_machine(const rclcpp::Time &now, MotorCommand &command)
{
    TrajectoryRuntime rt;
    if (!snapshot_trajectory_runtime(rt)) {
        return;
    }

    auto goal_handle = rt.goal_handle;
    const auto goal = rt.goal;
    auto mapped_indices = rt.mapped_indices;
    const auto actual_start_q = rt.actual_start_q;
    bool cancel_requested = rt.cancel_requested;
    bool settle_phase = rt.settle_phase;
    const bool has_segment_vel = rt.has_segment_vel;
    size_t segment_index = rt.segment_index;
    const double one_point_duration = rt.one_point_duration;
    const rclcpp::Time start_time = rt.start_time;
    rclcpp::Time settle_start_time = rt.settle_start_time;

    if (shutdown_requested_.load()) {
        finish_trajectory_goal(goal_handle, FollowJointTrajectory::Result::INVALID_GOAL, false, "Trajectory aborted: shutdown requested.");
        return;
    }

    const auto &traj = goal->trajectory;
    if (traj.points.empty()) {
        finish_trajectory_goal(goal_handle, FollowJointTrajectory::Result::INVALID_GOAL, false, "Trajectory aborted: empty trajectory points.");
        return;
    }

    if (cancel_requested || goal_handle->is_canceling()) {
        auto state_snapshot = motor_state_buffer_.GetData();
        if (!state_snapshot) {
            finish_trajectory_goal(
                goal_handle,
                FollowJointTrajectory::Result::INVALID_GOAL,
                false,
                "Trajectory aborted: no motor state snapshot for cancel hold.");
            return;
        }
        for (int motor_id : mapped_indices) {
            command.q_target[motor_id] = state_snapshot->q[motor_id];
            command.dq_target[motor_id] = 0.0f;
        }
        finish_trajectory_goal(goal_handle, FollowJointTrajectory::Result::SUCCESSFUL, true, "Trajectory canceled by MoveIt.");
        return;
    }

    if (!settle_phase) {
        const double elapsed = (now - start_time).seconds();

        if (traj.points.size() == 1) {
            const auto &point = traj.points.front();
            const double ratio = std::clamp(elapsed / one_point_duration, 0.0, 1.0);
            const bool finished_one_point = (elapsed >= one_point_duration);

            for (size_t j = 0; j < mapped_indices.size(); ++j) {
                const int motor_id = mapped_indices[j];
                const double start_angle = actual_start_q[motor_id];
                if (finished_one_point) {
                    command.q_target[motor_id] = point.positions[j];
                    command.dq_target[motor_id] = 0.0f;
                } else {
                    command.q_target[motor_id] = static_cast<float>(start_angle + ratio * (point.positions[j] - start_angle));
                    command.dq_target[motor_id] = static_cast<float>((point.positions[j] - start_angle) / one_point_duration);
                }
            }

            if (finished_one_point) {
                settle_phase = true;
                settle_start_time = now;
            }
        } else {
            const double final_time = rclcpp::Duration(traj.points.back().time_from_start).seconds();
            if (elapsed >= final_time) {
                const auto &final_point = traj.points.back();
                for (size_t j = 0; j < mapped_indices.size(); ++j) {
                    const int motor_id = mapped_indices[j];
                    command.q_target[motor_id] = final_point.positions[j];
                    command.dq_target[motor_id] = (final_point.velocities.size() == mapped_indices.size())
                        ? static_cast<float>(final_point.velocities[j])
                        : 0.0f;
                }
                settle_phase = true;
                settle_start_time = now;
            } else {
                while (segment_index < traj.points.size() &&
                       elapsed > rclcpp::Duration(traj.points[segment_index].time_from_start).seconds()) {
                    ++segment_index;
                }

                if (segment_index >= traj.points.size()) {
                    segment_index = traj.points.size() - 1;
                }

                const auto &prev_point = traj.points[segment_index - 1];
                const auto &target_point = traj.points[segment_index];
                const double prev_time = rclcpp::Duration(prev_point.time_from_start).seconds();
                const double target_time = rclcpp::Duration(target_point.time_from_start).seconds();
                const double segment_dt = std::max(target_time - prev_time, 1e-6);
                const double ratio = std::clamp((elapsed - prev_time) / segment_dt, 0.0, 1.0);

                std::array<float, G1_NUM_MOTOR> state_dq{0};
                if (g_eval_config.mode == EvalMode::kCubicHermite && has_segment_vel && segment_index == 1) {
                    auto state_snapshot = motor_state_buffer_.GetData();
                    if (!state_snapshot) {
                        finish_trajectory_goal(
                            goal_handle,
                            FollowJointTrajectory::Result::INVALID_GOAL,
                            false,
                            "Trajectory aborted: no motor state snapshot for segment velocity interpolation.");
                        return;
                    }
                    state_dq = state_snapshot->dq;
                }

                for (size_t j = 0; j < mapped_indices.size(); ++j) {
                    const int motor_id = mapped_indices[j];
                    const double start_angle = (segment_index == 1)
                        ? actual_start_q[motor_id]
                        : prev_point.positions[j];

                    if (g_eval_config.mode == EvalMode::kCubicHermite && has_segment_vel) {
                        const double v0 = (segment_index == 1) ? state_dq[motor_id] : prev_point.velocities[j];
                        const double v1 = target_point.velocities[j];
                        const double t = ratio;
                        const double t2 = t * t;
                        const double t3 = t2 * t;
                        const double h00 = 2.0 * t3 - 3.0 * t2 + 1.0;
                        const double h10 = t3 - 2.0 * t2 + t;
                        const double h01 = -2.0 * t3 + 3.0 * t2;
                        const double h11 = t3 - t2;

                        const double dh00 = 6.0 * t2 - 6.0 * t;
                        const double dh10 = 3.0 * t2 - 4.0 * t + 1.0;
                        const double dh01 = -6.0 * t2 + 6.0 * t;
                        const double dh11 = 3.0 * t2 - 2.0 * t;

                        command.q_target[motor_id] = static_cast<float>(
                            h00 * start_angle + h10 * segment_dt * v0 +
                            h01 * target_point.positions[j] + h11 * segment_dt * v1);
                        command.dq_target[motor_id] = static_cast<float>(
                            (dh00 * start_angle + dh10 * segment_dt * v0 +
                             dh01 * target_point.positions[j] + dh11 * segment_dt * v1) /
                            segment_dt);
                    } else if (g_eval_config.mode == EvalMode::kLinearDq0) {
                        command.q_target[motor_id] = static_cast<float>(
                            start_angle + ratio * (target_point.positions[j] - start_angle));
                        command.dq_target[motor_id] = 0.0f;
                    } else {
                        command.q_target[motor_id] = static_cast<float>(
                            start_angle + ratio * (target_point.positions[j] - start_angle));
                        command.dq_target[motor_id] = static_cast<float>(
                            (target_point.positions[j] - start_angle) / segment_dt);
                    }
                }
            }
        }
    }

    const bool about_to_finish = settle_phase && 
        (now - settle_start_time).seconds() >= kSettleDurationSec;
    if (!about_to_finish) {
        persist_trajectory_runtime(
            goal_handle,
            settle_phase,
            segment_index,
            settle_start_time);
    }

    if (!settle_phase) {
        return;
    }

    if ((now - settle_start_time).seconds() < kSettleDurationSec) {
        return;
    }

    const auto &final_point = traj.points.back();
    bool reached_goal = true;
    struct GoalMiss {
        std::string name;
        double target_angle;
        double actual_angle;
    };
    std::vector<GoalMiss> misses;

    auto state_snapshot = motor_state_buffer_.GetData();
    if (!state_snapshot) {
        finish_trajectory_goal(
            goal_handle,
            FollowJointTrajectory::Result::INVALID_GOAL,
            false,
            "Trajectory aborted: no motor state snapshot for final tolerance check.");
        return;
    }

    for (size_t j = 0; j < mapped_indices.size(); ++j) {
        const int motor_id = mapped_indices[j];
        const double target_angle = final_point.positions[j];
        const double actual_angle = state_snapshot->q[motor_id];
        command.dq_target[motor_id] = 0.0f;

        if (std::abs(target_angle - actual_angle) > kGoalToleranceRad) {
            misses.push_back({goal->trajectory.joint_names[j], target_angle, actual_angle});
            reached_goal = false;
        }
    }

    for (const auto &miss : misses) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Physics check failed for %s! Target: %.3f rad, Actual: %.3f rad",
            miss.name.c_str(),
            miss.target_angle,
            miss.actual_angle);
    }

    if (reached_goal) {
        finish_trajectory_goal(goal_handle, FollowJointTrajectory::Result::SUCCESSFUL, false, "Trajectory Executed and Physically Verified!");
    } else {
        finish_trajectory_goal(
            goal_handle,
            FollowJointTrajectory::Result::GOAL_TOLERANCE_VIOLATED,
            false,
            "Action Aborted: The physical robot could not reach the IK target.");
    }
}

void G1MoveItBridge::perform_shutdown_homing()
{
    if (shutdown_homing_done_.load(std::memory_order_acquire)) {
        return;
    }

    request_shutdown_homing();

    // Fallback path used during teardown when executor callbacks are no longer spinning.
    if (!shutdown_homing_done_.load(std::memory_order_acquire)) {
        unitree_hg::msg::LowCmd low_cmd;
        low_cmd.mode_pr = mode_pr_;
        low_cmd.mode_machine = mode_machine_.load(std::memory_order_relaxed);

        for (int step = 0; step <= kHomingSteps; ++step) {
            const double ratio = static_cast<double>(step) / static_cast<double>(kHomingSteps);
            for (int i = 0; i < G1_NUM_MOTOR; ++i) {
                low_cmd.motor_cmd[i].mode = 0x01;
                low_cmd.motor_cmd[i].q = shutdown_homing_start_q_[i] * (1.0 - ratio);
                low_cmd.motor_cmd[i].dq = 0.0;
                low_cmd.motor_cmd[i].tau = 0.0;
                low_cmd.motor_cmd[i].kp = motor_kp_[i];
                low_cmd.motor_cmd[i].kd = motor_kd_[i];
            }
            get_crc(low_cmd);
            lowcmd_publisher_->publish(low_cmd);
            std::this_thread::sleep_for(kControlPeriod);
        }
        shutdown_homing_active_.store(false, std::memory_order_release);
        shutdown_homing_done_.store(true, std::memory_order_release);
        RCLCPP_INFO(this->get_logger(), "Returned to home. Exiting...");
    }
}

G1MoveItBridge::~G1MoveItBridge() 
{
    if (init_thread_.joinable()) {
        init_thread_.join();
    }

    {
        std::lock_guard<std::mutex> lock(g_eval_logger.mutex);
        if (g_eval_logger.csv.is_open()) {
            g_eval_logger.csv.flush();
            g_eval_logger.csv.close();
        }
    }

    perform_shutdown_homing();
}

} // namespace g1_custom_control

int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    std::signal(SIGINT, sigint_handler);

    auto node = std::make_shared<g1_custom_control::G1MoveItBridge>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
    executor.add_node(node);

    while (rclcpp::ok() && !g_sigint_requested.load(std::memory_order_relaxed)) {
        executor.spin_some(2ms);
    }

    if (g_sigint_requested.load(std::memory_order_relaxed) && rclcpp::ok()) {
        node->request_shutdown_homing();
        while (rclcpp::ok() && !node->is_shutdown_homing_done()) {
            executor.spin_some(2ms);
        }
    }

    executor.remove_node(node);
    node.reset();

    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
    return 0;
}
