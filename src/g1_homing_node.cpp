/**
 * A ROS 2 Node to safely lock all 29 joints of the Unitree G1 at the Home (0.0) position.
 * Uses exact physical motor gain mapping and 3-second interpolation for safety.
 **/
#include "g1/motor_crc_hg.h"
#include "rclcpp/rclcpp.hpp"
#include "unitree_hg/msg/low_cmd.hpp"
#include "unitree_hg/msg/low_state.hpp"
#include <algorithm>
#include <array>
#include <sstream>
#include <iomanip>

constexpr int G1_NUM_MOTOR = 29;

// --- EXACT GAINS FROM g1_dual_arm_example.cpp ---
enum MotorType { GEARBOX_S = 0, GEARBOX_M = 1, GEARBOX_L = 2 };

const std::array<MotorType, G1_NUM_MOTOR> G1MotorType{
    // legs
    GEARBOX_M, GEARBOX_M, GEARBOX_M, GEARBOX_L, GEARBOX_S, GEARBOX_S,
    GEARBOX_M, GEARBOX_M, GEARBOX_M, GEARBOX_L, GEARBOX_S, GEARBOX_S,
    // waist
    GEARBOX_M, GEARBOX_S, GEARBOX_S,
    // arms
    GEARBOX_S, GEARBOX_S, GEARBOX_S, GEARBOX_S, GEARBOX_S, GEARBOX_S, GEARBOX_S,
    GEARBOX_S, GEARBOX_S, GEARBOX_S, GEARBOX_S, GEARBOX_S, GEARBOX_S, GEARBOX_S
};

float GetMotorKp(MotorType type) {
  switch (type) {
    case GEARBOX_S:
    case GEARBOX_M:
      return 40.0;
    case GEARBOX_L:
      return 100.0;
    default:
      return 0.0;
  }
}

float GetMotorKd(MotorType type) {
  switch (type) {
    case GEARBOX_S:
    case GEARBOX_M:
    case GEARBOX_L:
      return 1.0;
    default:
      return 0.0;
  }
}
// ------------------------------------------------

class G1HomingNode : public rclcpp::Node {
public:
  G1HomingNode() : Node("g1_homing_node") {
    lowstate_subscriber_ = this->create_subscription<unitree_hg::msg::LowState>(
        "lowstate", 10,
        [this](const unitree_hg::msg::LowState::SharedPtr msg) {
          mode_machine_ = msg->mode_machine;
          for (int i = 0; i < G1_NUM_MOTOR; i++) {
            current_q_[i] = msg->motor_state[i].q; // Store live position
            
            if (!initial_q_captured_) {
                initial_q_[i] = msg->motor_state[i].q; // Lock homing start position
            }
          }
          initial_q_captured_ = true;

          // Throttled Console Print (1Hz)
          print_counter_++;
          if (print_counter_ >= 500) { 
            print_counter_ = 0;
            
            std::stringstream ss;
            ss << "\n--- Current 29 Joint Angles (rad) ---\n";
            for (int i = 0; i < G1_NUM_MOTOR; i++) {
              // Format to 2 decimal places with a tab space
              ss << "J[" << std::setw(2) << i << "]: " 
                 << std::fixed << std::setprecision(2) << current_q_[i] << "\t";
              
              // Create a new line every 5 joints for readability
              if ((i + 1) % 5 == 0) ss << "\n";
            }
            RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
          }
        });

    lowcmd_publisher_ = this->create_publisher<unitree_hg::msg::LowCmd>("lowcmd", 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(2), 
        std::bind(&G1HomingNode::timer_callback, this)
    );

    RCLCPP_INFO(this->get_logger(), "Starting 3-second smooth homing sequence...");
  }

private:
  void timer_callback() {
    if (!initial_q_captured_) return;

    unitree_hg::msg::LowCmd low_cmd;
    low_cmd.mode_pr = mode_pr_;
    low_cmd.mode_machine = mode_machine_;

    time_ += 0.002; 
    double ratio = std::clamp(time_ / duration_, 0.0, 1.0);

    for (int i = 0; i < G1_NUM_MOTOR; i++) {
      low_cmd.motor_cmd[i].mode = 0x01;
      
      // Interpolate from start position to 0.0
      low_cmd.motor_cmd[i].q = initial_q_[i] * (1.0 - ratio) + (0.0 * ratio);
      
      low_cmd.motor_cmd[i].dq = 0.0;
      low_cmd.motor_cmd[i].tau = 0.0;
      
      // Apply exact gains based on the physical motor type!
      low_cmd.motor_cmd[i].kp = GetMotorKp(G1MotorType[i]); 
      low_cmd.motor_cmd[i].kd = GetMotorKd(G1MotorType[i]);
      
      // if (i == 14) {
      //     low_cmd.motor_cmd[i].kp = 120.0; // Increase from 40 to 120
      // }
    }

    get_crc(low_cmd);
    lowcmd_publisher_->publish(low_cmd);

    if (ratio >= 1.0 && !homing_complete_printed_) {
        RCLCPP_INFO(this->get_logger(), "Homing complete! Holding position at 0.0.");
        homing_complete_printed_ = true;
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<unitree_hg::msg::LowCmd>::SharedPtr lowcmd_publisher_;
  rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr lowstate_subscriber_;
  
  std::array<float, G1_NUM_MOTOR> initial_q_{0};
  std::array<float, G1_NUM_MOTOR> current_q_{0};
  uint8_t mode_pr_ = 0;
  uint8_t mode_machine_ = 0;
  int print_counter_ = 0;
  bool initial_q_captured_ = false;
  bool homing_complete_printed_ = false;
  
  double time_ = 0.0;
  double duration_ = 3.0; // 3 seconds
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<G1HomingNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}