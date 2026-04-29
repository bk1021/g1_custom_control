# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build

This is a ROS 2 (ament_cmake) package. The workspace root is `/home/aaa/g1_ws`.

```bash
# Build only this package
cd /home/aaa/g1_ws
colcon build --packages-select g1_custom_control

# Source before running nodes
source /home/aaa/g1_ws/install/setup.bash
```

There are no automated tests beyond ament linting. The linters are intentionally disabled (`ament_cmake_copyright_FOUND TRUE`, `ament_cmake_cpplint_FOUND TRUE`) in `CMakeLists.txt`.

## Running nodes

```bash
# Home all 29 joints to zero (safe starting point)
ros2 run g1_custom_control g1_homing_node

# MoveIt bridge — low-level mode (publishes on /lowcmd, requires motion mode released)
ros2 run g1_custom_control g1_moveit_bridge

# MoveIt bridge — high-level arm_sdk mode (requires motion mode active)
ros2 run g1_custom_control g1_moveit_bridge --ros-args -p use_arm_sdk:=true

# Servo bridge — lightweight hardware bridge for MoveIt Servo output (arm_sdk by default)
ros2 run g1_custom_control g1_servo_bridge
ros2 run g1_custom_control g1_servo_bridge --ros-args -p use_arm_sdk:=false

# Autonomous evaluation (runs cubic_hermite → linear → linear_dq0 in sequence)
ros2 run g1_custom_control g1_evaluate --ros-args -p eval_arm:=left

# Dual-arm joystick teleoperation via MoveIt Servo (Logitech F710)
# use_arm_sdk:=true (default) for standing robot; false for released/lowcmd mode
ros2 launch g1_custom_control dual_arm_servo_teleop.launch.py
ros2 launch g1_custom_control dual_arm_servo_teleop.launch.py use_arm_sdk:=false

# Locomotion sequence (forward → U-turn → forward → U-turn)
ros2 run g1_custom_control custom_loco_movement

# Plot evaluation CSVs
python3 src/g1_plot_evaluation.py --csv g1_evaluate_left_*.csv --out-dir eval_plots --plot-dq
```

## Architecture

### Overview

Custom ROS 2 control stack for the **Unitree G1 29-DOF humanoid robot**. The main purpose is to bridge MoveIt trajectory execution to the robot's low-level motor interface, with two independent control paths.

### Joint index layout (G1_NUM_MOTOR = 29)

| Indices | Body part |
|---------|-----------|
| 0–11   | Legs (left 0-5, right 6-11) |
| 12–14  | Waist (yaw, roll, pitch) |
| 15–21  | Left arm (shoulder pitch/roll/yaw, elbow, wrist roll/pitch/yaw) |
| 22–28  | Right arm (same order) |

`arm_sdk` mode controls only joints 12–28 (waist + arms); joint 29 carries the `control_weight` float (0.0–1.0). `lowcmd` mode controls all 29 joints.

### Shared constants header (`g1_control_constants.hpp`)

`include/g1_custom_control/g1_control_constants.hpp` is included by both `g1_moveit_bridge.hpp` and `g1_servo_bridge.cpp`. It lives in `namespace g1_custom_control` and defines:

- Motor layout: `G1_NUM_MOTOR`, `MotorType` enum, `G1MotorType` array, `GetMotorKp`/`GetMotorKd`
- Joint name ↔ index tables: `JOINT_NAMES`, `JOINT_NAME_TO_IDX`
- `DataBuffer<T>` — lock-promoted `shared_ptr` wrapper using `std::shared_mutex`
- Timing constants: `kLowCmdControlPeriod` (2 ms), `kArmSdkControlPeriod` (20 ms), homing steps, overrun thresholds
- arm_sdk gains and joint range constants: `kArmSdkJointKp/Kd`, `kArmSdkWaistGainScale`, `kWaistFirstJoint/LastJoint`, `kArmSdkFirstJoint/LastJoint`, `kArmSdkWeightJoint`
- Helper functions: `is_waist_joint`, `is_arm_sdk_joint`, `homing_steps_for_mode`, `control_period_for_mode`, `loop_overrun_threshold_for_mode`

### G1MoveItBridge (`g1_moveit_bridge.cpp` / `g1_evaluate.cpp`)

The central node implementing `rclcpp::Node`. The same header (`g1_moveit_bridge.hpp`) is shared between two separate executables that compile the same class with different `main()` and different constructor bodies. The header now includes `g1_control_constants.hpp` for all shared constants and utilities.

**Two control modes** selected by `use_arm_sdk` parameter:
- **Low-level (`lowcmd`, default)**: Publishes `unitree_hg/LowCmd` on `/lowcmd` at 2 ms. Requires releasing the robot's motion service (`MotionSwitchClient::ReleaseMode()`). Controls all 29 joints. On startup, all joints are homed to 0 rad over 1000 steps (2 s).
- **High-level (`arm_sdk`)**: Publishes on `/arm_sdk` at 20 ms. Requires the robot motion service to be active. Controls only arm/waist joints (12–28). On SIGINT, releases `control_weight` to 0 over 100 steps (2 s) instead of homing.

**Dual-loop design**: Two independent 2 ms (or 20 ms) wall-timers on separate callback groups in a 3-thread `MultiThreadedExecutor`:
1. `command_writer_loop` — trajectory state machine → writes `MotorCommand` to `DataBuffer`
2. `control_loop` — reads `DataBuffer` → serializes and publishes the hardware command

**Trajectory state machine** (`run_trajectory_state_machine`):
- Accepts `FollowJointTrajectory` actions on `left_arm_controller/follow_joint_trajectory` and `right_arm_controller/follow_joint_trajectory`
- Lower-body joints (indices < 15) are rejected in goals
- Two interpolation strategies: cubic Hermite splines (when trajectory contains velocities) and linear fallback
- After trajectory completion, waits `kSettleDurationSec` (0.5 s), then checks physical joint positions against `kGoalToleranceRad` (0.1 rad)

**Graceful shutdown**: SIGINT is caught manually. The node runs a "shutdown homing" sequence (returns joints to 0 or releases arm_sdk weight) before `rclcpp::shutdown()`.

### G1ServoBridge (`g1_servo_bridge.cpp`)

Lightweight hardware bridge designed specifically for MoveIt Servo's streaming output. Unlike `G1MoveItBridge`, it does not execute full `FollowJointTrajectory` actions or interpolate — it simply forwards the **last point** of each incoming `trajectory_msgs/JointTrajectory` message directly to the hardware.

**Subscriptions**: `trajectory_msgs/JointTrajectory` on `/left_arm_controller/joint_trajectory` and `/right_arm_controller/joint_trajectory` (topics, not action servers).

**Publishes**: `/arm_sdk` or `/lowcmd` (selected by `use_arm_sdk` parameter, default `true`). Also republishes `/joint_states` from lowstate feedback.

**Two control modes** (same parameter as `G1MoveItBridge`):
- **`arm_sdk` (default)**: On startup, ramps `control_weight` 0→1 over 100 steps (2 s) while holding initial joint positions. On shutdown, ramps weight back to 0.
- **`lowcmd`**: No weight ramp. Leg joints (indices < 12) are always held at their current `LowState` position (never commanded to zero).

**Graceful shutdown**: Same SIGINT pattern as `G1MoveItBridge`. Shutdown sequence runs inside the timer callback or falls back to a blocking loop in the destructor.

### G1HomingNode (`g1_homing_node.cpp`)

Standalone utility that homes all 29 joints to 0 rad over 3 s using per-motor-type PD gains (kp=40 for S/M gearboxes, kp=100 for L gearboxes; kd=1.0 for all). Publishes at 500 Hz (2 ms timer) on `/lowcmd` with CRC.

### CustomLocoMovementNode (`custom_loco_movement.cpp`)

Locomotion demo using `unitree::robot::g1::LocoClient`. Runs a predefined sequence (forward → U-turn × 2) after ENTER key press, while simultaneously holding arms at home via `/arm_sdk` on a background thread.

### g1_evaluate (`g1_evaluate.cpp`)

Evaluation harness built on the same `G1MoveItBridge` class. Automatically cycles through three interpolation modes (`cubic_hermite`, `linear`, `linear_dq0`) for a named MoveIt target and logs commanded/actual joint positions/velocities to a timestamped CSV for offline analysis.

### Dual-arm servo teleoperation

`dual_arm_servo_teleop.launch.py` starts MoveIt `move_group`, two `moveit_servo` nodes (one per arm), a `joy_node`, `joy_to_servo_mapper.py`, and `g1_servo_bridge`. The `use_arm_sdk` launch argument (default `true`) is forwarded to `g1_servo_bridge` to select the hardware interface. The mapper supports three modes cycled by Y button: TRANSLATION, ROTATION, JOINT (select joint with D-pad); in JOINT mode, a zero command is always sent to the idle arm to prevent drift. MoveIt config is loaded from the external `g1_dual_arm_moveit_config` package.

### Key external dependencies

- `unitree_hg` — Unitree HG message types (`LowCmd`, `LowState`)
- `unitree_api` — `MotionSwitchClient`, `LocoClient` (header-only, vendored under `include/g1/`)
- `trajectory_msgs` — `JointTrajectory` used by `g1_servo_bridge`
- `g1_dual_arm_moveit_config` — robot URDF/SRDF/kinematics for the G1 29-DOF configuration (separate package, not in this repo)
- `moveit_ros_planning_interface`, `moveit_servo`, `moveit_ros_move_group` — MoveIt 2
