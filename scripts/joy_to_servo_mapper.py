#!/usr/bin/env python3

import threading
from typing import List

import rclpy
from rclpy.node import Node

from control_msgs.msg import JointJog
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import ServoCommandType
from sensor_msgs.msg import Joy


MODE_TRANSLATION = 0
MODE_ROTATION = 1
MODE_JOINT = 2
MODE_NAMES = {
    MODE_TRANSLATION: "TRANSLATION",
    MODE_ROTATION: "ROTATION",
    MODE_JOINT: "JOINT",
}


class JoyToServoMapper(Node):
    def __init__(self) -> None:
        super().__init__("joy_to_servo_mapper")

        self._declare_parameters()
        self._load_parameters()

        self.latest_joy = None
        self.prev_buttons = []
        self.prev_dpad_up = False
        self.prev_dpad_down = False

        self.mode = MODE_TRANSLATION
        self.selected_joint = 0

        self.left_twist_pub = self.create_publisher(TwistStamped, self.left_twist_topic, 10)
        self.right_twist_pub = self.create_publisher(TwistStamped, self.right_twist_topic, 10)
        self.left_joint_pub = self.create_publisher(JointJog, self.left_joint_topic, 10)
        self.right_joint_pub = self.create_publisher(JointJog, self.right_joint_topic, 10)

        self.left_cmd_type_client = self.create_client(
            ServoCommandType, "/left_servo/switch_command_type")
        self.right_cmd_type_client = self.create_client(
            ServoCommandType, "/right_servo/switch_command_type")

        self.create_subscription(Joy, self.joy_topic, self._joy_callback, 10)
        self.create_timer(1.0 / self.publish_rate_hz, self._tick)

        threading.Thread(target=self._init_command_type, daemon=True).start()

        self.get_logger().info(
            "joy_to_servo_mapper started. mode=%s, publish_rate=%.1f Hz"
            % (MODE_NAMES[self.mode], self.publish_rate_hz)
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("publish_rate_hz", 100.0)
        self.declare_parameter("deadzone", 0.10)

        self.declare_parameter("deadman_button_index", -1)
        self.declare_parameter("mode_cycle_button_index", 3)

        self.declare_parameter("axis_left_x", 0)
        self.declare_parameter("axis_left_y", 1)
        self.declare_parameter("axis_right_x", 3)
        self.declare_parameter("axis_right_y", 4)
        self.declare_parameter("axis_dpad_x", 6)
        self.declare_parameter("axis_dpad_y", 7)

        self.declare_parameter("button_left_up", 4)
        self.declare_parameter("button_left_down", -1)
        self.declare_parameter("button_right_up", 5)
        self.declare_parameter("button_right_down", -1)

        self.declare_parameter("translation_speed", 0.20)
        self.declare_parameter("z_speed", 0.20)
        self.declare_parameter("rotation_speed", 0.80)
        self.declare_parameter("joint_speed", 0.60)

        self.declare_parameter("left_twist_topic", "/left_servo/delta_twist_cmds")
        self.declare_parameter("right_twist_topic", "/right_servo/delta_twist_cmds")
        self.declare_parameter("left_joint_topic", "/left_servo/delta_joint_cmds")
        self.declare_parameter("right_joint_topic", "/right_servo/delta_joint_cmds")

        self.declare_parameter("left_command_frame", "left_wrist_yaw_link")
        self.declare_parameter("right_command_frame", "right_wrist_yaw_link")
        self.declare_parameter("planning_frame", "pelvis")

        self.declare_parameter(
            "left_joint_names",
            [
                "left_shoulder_pitch_joint",
                "left_shoulder_roll_joint",
                "left_shoulder_yaw_joint",
                "left_elbow_joint",
                "left_wrist_roll_joint",
                "left_wrist_pitch_joint",
                "left_wrist_yaw_joint",
            ],
        )
        self.declare_parameter(
            "right_joint_names",
            [
                "right_shoulder_pitch_joint",
                "right_shoulder_roll_joint",
                "right_shoulder_yaw_joint",
                "right_elbow_joint",
                "right_wrist_roll_joint",
                "right_wrist_pitch_joint",
                "right_wrist_yaw_joint",
            ],
        )

    def _load_parameters(self) -> None:
        self.joy_topic = self.get_parameter("joy_topic").value
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.deadzone = float(self.get_parameter("deadzone").value)

        self.deadman_button_index = int(self.get_parameter("deadman_button_index").value)
        self.mode_cycle_button_index = int(self.get_parameter("mode_cycle_button_index").value)

        self.axis_left_x = int(self.get_parameter("axis_left_x").value)
        self.axis_left_y = int(self.get_parameter("axis_left_y").value)
        self.axis_right_x = int(self.get_parameter("axis_right_x").value)
        self.axis_right_y = int(self.get_parameter("axis_right_y").value)
        self.axis_dpad_x = int(self.get_parameter("axis_dpad_x").value)
        self.axis_dpad_y = int(self.get_parameter("axis_dpad_y").value)

        self.button_left_up = int(self.get_parameter("button_left_up").value)
        self.button_left_down = int(self.get_parameter("button_left_down").value)
        self.button_right_up = int(self.get_parameter("button_right_up").value)
        self.button_right_down = int(self.get_parameter("button_right_down").value)

        self.translation_speed = float(self.get_parameter("translation_speed").value)
        self.z_speed = float(self.get_parameter("z_speed").value)
        self.rotation_speed = float(self.get_parameter("rotation_speed").value)
        self.joint_speed = float(self.get_parameter("joint_speed").value)

        self.left_twist_topic = self.get_parameter("left_twist_topic").value
        self.right_twist_topic = self.get_parameter("right_twist_topic").value
        self.left_joint_topic = self.get_parameter("left_joint_topic").value
        self.right_joint_topic = self.get_parameter("right_joint_topic").value

        self.left_command_frame = self.get_parameter("left_command_frame").value
        self.right_command_frame = self.get_parameter("right_command_frame").value
        self.planning_frame = self.get_parameter("planning_frame").value

        self.left_joint_names: List[str] = list(self.get_parameter("left_joint_names").value)
        self.right_joint_names: List[str] = list(self.get_parameter("right_joint_names").value)
        self.all_joint_names = self.left_joint_names + self.right_joint_names

    def _joy_callback(self, msg: Joy) -> None:
        self.latest_joy = msg

        if not self.prev_buttons:
            self.prev_buttons = [0] * len(msg.buttons)

        if self._rising_button(msg, self.mode_cycle_button_index):
            self.mode = (self.mode + 1) % 3
            self.get_logger().info("Mode switched to %s" % MODE_NAMES[self.mode])
            self._switch_servo_command_type(
                ServoCommandType.Request.JOINT_JOG if self.mode == MODE_JOINT
                else ServoCommandType.Request.TWIST
            )

        dpad_y = self._get_axis(msg, self.axis_dpad_y)
        dpad_up = dpad_y > 0.5
        dpad_down = dpad_y < -0.5

        if self.mode == MODE_JOINT:
            if dpad_up and not self.prev_dpad_up:
                self.selected_joint = (self.selected_joint - 1) % len(self.all_joint_names)
                self.get_logger().info("Selected joint: %s" % self.all_joint_names[self.selected_joint])
            if dpad_down and not self.prev_dpad_down:
                self.selected_joint = (self.selected_joint + 1) % len(self.all_joint_names)
                self.get_logger().info("Selected joint: %s" % self.all_joint_names[self.selected_joint])

        self.prev_dpad_up = dpad_up
        self.prev_dpad_down = dpad_down
        self.prev_buttons = list(msg.buttons)

    def _tick(self) -> None:
        if self.latest_joy is None:
            return

        joy = self.latest_joy
        deadman_active = self._deadman_active(joy)

        if self.mode in (MODE_TRANSLATION, MODE_ROTATION):
            self._publish_twist_mode(joy, deadman_active)
        else:
            self._publish_joint_mode(joy, deadman_active)

    def _publish_twist_mode(self, joy: Joy, deadman_active: bool) -> None:
        left_x = self._apply_deadzone(self._get_axis(joy, self.axis_left_y))
        left_y = -self._apply_deadzone(self._get_axis(joy, self.axis_left_x))
        right_x = self._apply_deadzone(self._get_axis(joy, self.axis_right_y))
        right_y = -self._apply_deadzone(self._get_axis(joy, self.axis_right_x))

        left_up = 1.0 if self._get_button(joy, self.button_left_up) else 0.0
        left_down = 1.0 if self._get_button(joy, self.button_left_down) else 0.0
        right_up = 1.0 if self._get_button(joy, self.button_right_up) else 0.0
        right_down = 1.0 if self._get_button(joy, self.button_right_down) else 0.0

        left_z = left_up - left_down
        right_z = right_up - right_down

        if not deadman_active:
            left_x = left_y = left_z = 0.0
            right_x = right_y = right_z = 0.0

        if self.mode == MODE_TRANSLATION:
            self._publish_twist(self.left_twist_pub, self.left_command_frame, left_x * self.translation_speed, left_y * self.translation_speed, left_z * self.z_speed, 0.0, 0.0, 0.0)
            self._publish_twist(self.right_twist_pub, self.right_command_frame, right_x * self.translation_speed, right_y * self.translation_speed, right_z * self.z_speed, 0.0, 0.0, 0.0)
        else:
            self._publish_twist(self.left_twist_pub, self.left_command_frame, 0.0, 0.0, 0.0, left_y * self.rotation_speed, left_x * self.rotation_speed, left_z * self.rotation_speed)
            self._publish_twist(self.right_twist_pub, self.right_command_frame, 0.0, 0.0, 0.0, right_y * self.rotation_speed, right_x * self.rotation_speed, right_z * self.rotation_speed)

    def _publish_joint_mode(self, joy: Joy, deadman_active: bool) -> None:
        velocity = self._apply_deadzone(self._get_axis(joy, self.axis_left_y)) * self.joint_speed
        if not deadman_active:
            velocity = 0.0

        if self.selected_joint < len(self.left_joint_names):
            self._publish_joint_jog(self.left_joint_pub, self.left_joint_names[self.selected_joint], velocity)
            self._publish_joint_jog(self.right_joint_pub, self.right_joint_names[0], 0.0)
        else:
            idx = self.selected_joint - len(self.left_joint_names)
            self._publish_joint_jog(self.right_joint_pub, self.right_joint_names[idx], velocity)
            self._publish_joint_jog(self.left_joint_pub, self.left_joint_names[0], 0.0)

    def _publish_twist(
        self,
        publisher,
        frame_id: str,
        lx: float,
        ly: float,
        lz: float,
        ax: float,
        ay: float,
        az: float,
    ) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id

        msg.twist.linear.x = float(lx)
        msg.twist.linear.y = float(ly)
        msg.twist.linear.z = float(lz)
        msg.twist.angular.x = float(ax)
        msg.twist.angular.y = float(ay)
        msg.twist.angular.z = float(az)
        publisher.publish(msg)

    def _publish_joint_jog(self, publisher, joint_name: str, velocity: float) -> None:
        msg = JointJog()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.planning_frame
        msg.joint_names = [joint_name]
        msg.velocities = [float(velocity)]
        msg.duration = 1.0 / self.publish_rate_hz
        publisher.publish(msg)

    def _init_command_type(self) -> None:
        self.left_cmd_type_client.wait_for_service()
        self.right_cmd_type_client.wait_for_service()
        self._switch_servo_command_type(ServoCommandType.Request.TWIST)
        self.get_logger().info("Servo command type initialized to TWIST")

    def _switch_servo_command_type(self, command_type: int) -> None:
        req = ServoCommandType.Request()
        req.command_type = command_type
        self.left_cmd_type_client.call_async(req)
        self.right_cmd_type_client.call_async(req)

    def _deadman_active(self, msg: Joy) -> bool:
        if self.deadman_button_index < 0:
            return True
        return self._get_button(msg, self.deadman_button_index)

    def _rising_button(self, msg: Joy, index: int) -> bool:
        if index < 0 or index >= len(msg.buttons) or index >= len(self.prev_buttons):
            return False
        return msg.buttons[index] == 1 and self.prev_buttons[index] == 0

    def _get_axis(self, msg: Joy, index: int) -> float:
        if index < 0 or index >= len(msg.axes):
            return 0.0
        return float(msg.axes[index])

    def _get_button(self, msg: Joy, index: int) -> bool:
        if index < 0 or index >= len(msg.buttons):
            return False
        return msg.buttons[index] == 1

    def _apply_deadzone(self, value: float) -> float:
        return 0.0 if abs(value) < self.deadzone else value


def main() -> None:
    rclpy.init()
    node = JoyToServoMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
