#include <algorithm>
#include <atomic>
#include <chrono>
#include <g1/g1_loco_client.hpp>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>

#include <sys/select.h>
#include <unistd.h>

#include <unitree_hg/msg/low_cmd.hpp>

using namespace std::chrono_literals;

namespace {
constexpr float kArmSdkJointKp = 60.0F;
constexpr float kArmSdkJointKd = 1.5F;
constexpr float kArmSdkWaistGainScale = 4.0F;
constexpr int kWaistFirstJoint = 12;
constexpr int kWaistLastJoint = 14;
constexpr int kArmSdkFirstJoint = 12;
constexpr int kArmSdkLastJoint = 28;
constexpr int kArmSdkWeightJoint = 29;

constexpr bool is_waist_joint(const int idx)
{
  return idx >= kWaistFirstJoint && idx <= kWaistLastJoint;
}
}  // namespace

class CustomLocoMovementNode : public rclcpp::Node {
public:
  CustomLocoMovementNode()
  : Node("custom_loco_movement"), loco_client_(this)
  {
    forward_speed_mps_ = this->declare_parameter<double>("forward_speed_mps", 0.25);
    turn_speed_radps_ = this->declare_parameter<double>("turn_speed_radps", 0.70);
    forward_duration_s_ = this->declare_parameter<double>("forward_duration_s", 2.5);
    uturn_duration_s_ = this->declare_parameter<double>("uturn_duration_s", 4.5);
    settle_duration_s_ = this->declare_parameter<double>("settle_duration_s", 0.5);
    arm_home_startup_s_ = this->declare_parameter<double>("arm_home_startup_s", 2.0);
    arm_home_publish_period_s_ = this->declare_parameter<double>("arm_home_publish_period_s", 0.02);
    arm_home_control_weight_ = this->declare_parameter<double>("arm_home_control_weight", 1.0);
    arm_weight_ramp_down_s_ = this->declare_parameter<double>("arm_weight_ramp_down_s", 2.0);
    current_control_weight_.store(
      std::clamp(static_cast<float>(arm_home_control_weight_), 0.0F, 1.0F),
      std::memory_order_relaxed);

    rclcpp::QoS qos_pub(rclcpp::KeepLast(1));
    qos_pub.best_effort();
    arm_sdk_publisher_ = this->create_publisher<unitree_hg::msg::LowCmd>("arm_sdk", qos_pub);

    arm_home_thread_ = std::thread([this]() {
      arm_home_loop();
    });

    worker_thread_ = std::thread([this]() {
      std::this_thread::sleep_for(1s);

      RCLCPP_INFO(
        this->get_logger(),
        "Homing arms on /arm_sdk for %.2f s before sequence startup...",
        arm_home_startup_s_);
      sleep_seconds(arm_home_startup_s_);

      if (!wait_for_enter()) {
        return;
      }

      run_predefined_sequence();
    });
  }

  ~CustomLocoMovementNode() override
  {
    keep_arm_home_.store(false, std::memory_order_release);

    if (worker_thread_.joinable()) {
      worker_thread_.join();
    }

    if (arm_home_thread_.joinable()) {
      arm_home_thread_.join();
    }
  }

private:
  bool handle_action_result(const std::string &action_name, int32_t error_code)
  {
    if (error_code == 0) {
      return true;
    }

    RCLCPP_ERROR(this->get_logger(), "%s failed, error code: %d", action_name.c_str(), error_code);
    return false;
  }

  void sleep_seconds(double seconds) const
  {
    std::this_thread::sleep_for(std::chrono::duration<double>(seconds));
  }

  bool wait_for_enter()
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Arm home hold active. Press ENTER to start the locomotion sequence...");

    while (rclcpp::ok() && keep_arm_home_.load(std::memory_order_acquire)) {
      fd_set readfds;
      FD_ZERO(&readfds);
      FD_SET(STDIN_FILENO, &readfds);

      timeval timeout;
      timeout.tv_sec = 0;
      timeout.tv_usec = 100000;  // 100 ms polling for responsive shutdown.

      const int ret = select(STDIN_FILENO + 1, &readfds, nullptr, nullptr, &timeout);
      if (ret > 0 && FD_ISSET(STDIN_FILENO, &readfds)) {
        std::string line;
        std::getline(std::cin, line);
        return true;
      }
    }

    return false;
  }

  void publish_arm_home_command(const float control_weight)
  {
    if (!arm_sdk_publisher_) {
      return;
    }

    unitree_hg::msg::LowCmd arm_cmd;
    for (int i = kArmSdkFirstJoint; i <= kArmSdkLastJoint; ++i) {
      arm_cmd.motor_cmd[i].q = 0.0F;
      arm_cmd.motor_cmd[i].dq = 0.0F;
      arm_cmd.motor_cmd[i].tau = 0.0F;
      arm_cmd.motor_cmd[i].kp = is_waist_joint(i)
        ? (kArmSdkJointKp * kArmSdkWaistGainScale)
        : kArmSdkJointKp;
      arm_cmd.motor_cmd[i].kd = is_waist_joint(i)
        ? (kArmSdkJointKd * kArmSdkWaistGainScale)
        : kArmSdkJointKd;
    }

    arm_cmd.motor_cmd[kArmSdkWeightJoint].q = std::clamp(control_weight, 0.0F, 1.0F);

    arm_sdk_publisher_->publish(arm_cmd);
  }

  void arm_home_loop()
  {
    while (rclcpp::ok() && keep_arm_home_.load(std::memory_order_acquire)) {
      publish_arm_home_command(current_control_weight_.load(std::memory_order_relaxed));
      sleep_seconds(arm_home_publish_period_s_);
    }
  }

  void ramp_down_arm_control_weight()
  {
    if (!arm_sdk_publisher_) {
      return;
    }

    const double period_s = std::max(0.001, arm_home_publish_period_s_);
    const int ramp_steps = std::max(1, static_cast<int>(arm_weight_ramp_down_s_ / period_s));
    const float start_weight = std::clamp(
      current_control_weight_.load(std::memory_order_relaxed),
      0.0F,
      1.0F);

    for (int step = 0; step <= ramp_steps; ++step) {
      const double ratio = static_cast<double>(step) / static_cast<double>(ramp_steps);
      const float control_weight = std::clamp(
        static_cast<float>(start_weight * (1.0 - ratio)),
        0.0F,
        1.0F);

      current_control_weight_.store(control_weight, std::memory_order_relaxed);
      publish_arm_home_command(control_weight);

      if (step < ramp_steps) {
        sleep_seconds(period_s);
      }
    }
  }

  void request_shutdown()
  {
    bool expected = false;
    if (!shutdown_requested_.compare_exchange_strong(
          expected, true, std::memory_order_acq_rel, std::memory_order_acquire)) {
      return;
    }

    keep_arm_home_.store(false, std::memory_order_release);

    if (arm_home_thread_.joinable() && std::this_thread::get_id() != arm_home_thread_.get_id()) {
      arm_home_thread_.join();
    }

    ramp_down_arm_control_weight();
    rclcpp::shutdown();
  }

  void safe_stop()
  {
    const auto ret = loco_client_.StopMove();
    if (ret != 0) {
      RCLCPP_WARN(this->get_logger(), "StopMove returned non-zero during cleanup: %d", ret);
    }
  }

  void run_predefined_sequence()
  {
    RCLCPP_INFO(this->get_logger(), "Starting predefined movement: forward -> U-turn -> forward");

    const auto forward_1_ret = loco_client_.SetVelocity(
      static_cast<float>(forward_speed_mps_), 0.0F, 0.0F, static_cast<float>(forward_duration_s_));
    if (!handle_action_result("SetVelocity(forward 1)", forward_1_ret)) {
      safe_stop();
      request_shutdown();
      return;
    }
    sleep_seconds(forward_duration_s_ + settle_duration_s_);

    const auto turn_ret = loco_client_.SetVelocity(
      0.0F, 0.0F, static_cast<float>(turn_speed_radps_), static_cast<float>(uturn_duration_s_));
    if (!handle_action_result("SetVelocity(U-turn)", turn_ret)) {
      safe_stop();
      request_shutdown();
      return;
    }
    sleep_seconds(uturn_duration_s_ + settle_duration_s_);

    const auto forward_2_ret = loco_client_.SetVelocity(
      static_cast<float>(forward_speed_mps_), 0.0F, 0.0F, static_cast<float>(forward_duration_s_));
    if (!handle_action_result("SetVelocity(forward 2)", forward_2_ret)) {
      safe_stop();
      request_shutdown();
      return;
    }
    sleep_seconds(forward_duration_s_ + settle_duration_s_);

    safe_stop();
    RCLCPP_INFO(this->get_logger(), "Predefined movement sequence completed");
    request_shutdown();
  }

  unitree::robot::g1::LocoClient loco_client_;
  rclcpp::Publisher<unitree_hg::msg::LowCmd>::SharedPtr arm_sdk_publisher_;

  std::atomic<bool> keep_arm_home_{true};
  std::atomic<bool> shutdown_requested_{false};
  std::atomic<float> current_control_weight_{1.0F};
  std::thread arm_home_thread_;
  std::thread worker_thread_;

  double forward_speed_mps_ = 0.25;
  double turn_speed_radps_ = 0.70;
  double forward_duration_s_ = 2.5;
  double uturn_duration_s_ = 4.5;
  double settle_duration_s_ = 0.5;
  double arm_home_startup_s_ = 2.0;
  double arm_home_publish_period_s_ = 0.02;
  double arm_home_control_weight_ = 1.0;
  double arm_weight_ramp_down_s_ = 2.0;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CustomLocoMovementNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}