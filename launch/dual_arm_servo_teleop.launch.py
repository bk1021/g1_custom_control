import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # MoveIt semantic/config package (your existing package).
    moveit_config = MoveItConfigsBuilder(
        "g1_29dof",
        package_name="g1_dual_arm_moveit_config",
    ).to_moveit_configs()

    # Temporary location for Servo and joy-mapper config files.
    this_pkg_share = get_package_share_directory("g1_custom_control")
    left_servo_yaml = os.path.join(this_pkg_share, "config", "servo_left_arm.yaml")
    right_servo_yaml = os.path.join(this_pkg_share, "config", "servo_right_arm.yaml")
    joy_mapper_yaml = os.path.join(this_pkg_share, "config", "joy_to_servo_f710.yaml")

    # Publish a static identity transform world→pelvis so MoveIt Servo can resolve
    # the virtual_joint and compute Jacobians for Cartesian IK.
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_pelvis",
        arguments=["0", "0", "0", "0", "0", "0", "world", "pelvis"],
    )

    left_servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        name="left_servo",
        output="screen",
        emulate_tty=True,
        parameters=[moveit_config.to_dict(), left_servo_yaml],
        remappings=[
            ("~/delta_twist_cmds", "/left_servo/delta_twist_cmds"),
            ("~/delta_joint_cmds", "/left_servo/delta_joint_cmds"),
        ],
    )

    right_servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        name="right_servo",
        output="screen",
        emulate_tty=True,
        parameters=[moveit_config.to_dict(), right_servo_yaml],
        remappings=[
            ("~/delta_twist_cmds", "/right_servo/delta_twist_cmds"),
            ("~/delta_joint_cmds", "/right_servo/delta_joint_cmds"),
        ],
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        output="screen",
        parameters=[
            {
                "autorepeat_rate": 150.0,
                "deadzone": 0.10,
            }
        ],
    )

    joy_mapper_node = Node(
        package="g1_custom_control",
        executable="joy_to_servo_mapper.py",
        name="joy_to_servo_mapper",
        output="screen",
        parameters=[joy_mapper_yaml],
    )

    return LaunchDescription([
        static_tf_node,
        left_servo_node,
        right_servo_node,
        joy_node,
        joy_mapper_node,
    ])
