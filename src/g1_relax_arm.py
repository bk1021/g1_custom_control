import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class RelaxArms(Node):
    def __init__(self):
        super().__init__('relax_arms_node')
        self.left_client = ActionClient(self, FollowJointTrajectory, 'left_arm_controller/follow_joint_trajectory')
        self.right_client = ActionClient(self, FollowJointTrajectory, 'right_arm_controller/follow_joint_trajectory')

    def send_relax_goal(self):
        self.get_logger().info('Waiting for arm controllers...')
        self.left_client.wait_for_server()
        self.right_client.wait_for_server()

        # Define the joint names (must match your C++ bridge exactly)
        left_joints = [
            "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint", 
            "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint"
        ]
        right_joints = [
            "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint", 
            "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint"
        ]

        # Target position (0.0 usually means hanging straight down on the G1)
        target_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # Create the trajectory point (Take 3.0 seconds to smoothly drop)
        point = JointTrajectoryPoint()
        point.positions = target_positions
        point.time_from_start = Duration(sec=3, nanosec=0)

        # Build and send Left Goal
        left_goal = FollowJointTrajectory.Goal()
        left_goal.trajectory.joint_names = left_joints
        left_goal.trajectory.points = [point]
        self.left_client.send_goal_async(left_goal)

        # Build and send Right Goal
        right_goal = FollowJointTrajectory.Goal()
        right_goal.trajectory.joint_names = right_joints
        right_goal.trajectory.points = [point]
        self.right_client.send_goal_async(right_goal)

        self.get_logger().info('Relax commands sent! Waiting 3 seconds for arms to drop...')

def main(args=None):
    rclpy.init(args=args)
    node = RelaxArms()
    node.send_relax_goal()
    
    # Wait long enough for the 3-second trajectory to finish
    import time
    time.sleep(3.5) 
    
    node.get_logger().info('Arms are safe. You may now Ctrl+C the MoveIt Bridge.')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()