import time
import numpy as np

import rclpy
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class TestPublisher(Node):

    def __init__(self):
        super().__init__('test_publisher')

        self.side = input("Select hand (l / r) : ")

        self.left_joint_names = [
            "finger_l_joint1", "finger_l_joint2", "finger_l_joint3", "finger_l_joint4",
            "finger_l_joint5", "finger_l_joint6", "finger_l_joint7", "finger_l_joint8",
            "finger_l_joint9", "finger_l_joint10", "finger_l_joint11", "finger_l_joint12",
            "finger_l_joint13", "finger_l_joint14", "finger_l_joint15", "finger_l_joint16",
            "finger_l_joint17", "finger_l_joint18", "finger_l_joint19", "finger_l_joint20"
        ]

        self.right_joint_names = [
            "finger_r_joint1", "finger_r_joint2", "finger_r_joint3", "finger_r_joint4",
            "finger_r_joint5", "finger_r_joint6", "finger_r_joint7", "finger_r_joint8",
            "finger_r_joint9", "finger_r_joint10", "finger_r_joint11", "finger_r_joint12",
            "finger_r_joint13", "finger_r_joint14", "finger_r_joint15", "finger_r_joint16",
            "finger_r_joint17", "finger_r_joint18", "finger_r_joint19", "finger_r_joint20"
        ]

        self.duration = 1

        self.init_pose = np.array([
            0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0
        ])
        # ------------------------------------------------------------

        self.left_grasp_preset1 = np.array([
            0.0, 1.571, -0.628, -0.501,
            0.0, 0.85, 0.799, 0.6,
            0.0, 1.197, 1.501, 1.197,
            0.0, 1.197, 1.501, 1.197,
            0.0, 1.197, 1.501, 1.197
        ])

        self.left_release_preset1 = np.array([
            0.0, 1.571, -0.175, -0.262,
            0.0, 0.0, 0.0, 0.0,
            0.0, 1.197, 1.501, 1.197,
            0.0, 1.197, 1.501, 1.197,
            0.0, 1.197, 1.501, 1.197
        ])

        self.right_grasp_preset1 = np.array([
            0.0, -1.571, 0.628, 0.501,
            0.0, 0.85, 0.799, 0.6,
            0.0, 1.197, 1.501, 1.197,
            0.0, 1.197, 1.501, 1.197,
            0.0, 1.197, 1.501, 1.197
        ])

        self.right_release_preset1 = np.array([
            0.0, -1.571, 0.175, 0.262,
            0.0, 0.0, 0.0, 0.0,
            0.0, 1.197, 1.501, 1.197,
            0.0, 1.197, 1.501, 1.197,
            0.0, 1.197, 1.501, 1.197
        ])

        # ------------------------------------------------------------

        self.left_grasp_preset2 = np.array([
            -0.199, 1.571, -0.602, -0.475,
            0.147, 0.901, 0.89, 0.45,
            -0.148, 1.047, 0.799, 0.45,
            -0.148, 1.197, 1.501, 1.197,
            -0.148, 1.197, 1.501, 1.197
        ])

        self.left_release_preset2 = np.array([
            -0.199, 1.571, 0.0, 0.0,
            0.147, 0.0, 0.0, 0.0,
            -0.148, 0.0, 0.0, 0.0,
            -0.148, 1.197, 1.501, 1.197,
            -0.148, 1.197, 1.501, 1.197
        ])

        self.right_grasp_preset2 = np.array([
            0.199, -1.571, 0.602, 0.475,
            -0.147, 0.901, 0.89, 0.45,
            0.148, 1.047, 0.799, 0.45,
            0.148, 1.197, 1.501, 1.197,
            0.148, 1.197, 1.501, 1.197
        ])

        self.right_release_preset2 = np.array([
            0.199, -1.571, 0.0, 0.0,
            -0.147, 0.0, 0.0, 0.0,
            0.148, 0.0, 0.0, 0.0,
            0.148, 1.197, 1.501, 1.197,
            0.148, 1.197, 1.501, 1.197
        ])

        # ------------------------------------------------------------

        self.left_grasp_preset3 = np.array([
            -1.489, 1.483, -1.311, -1.047,
            0.0, 1.745, 1.396, 1.134,
            0.0, 1.745, 1.483, 1.134,
            0.0, 1.571, 1.396, 1.309,
            0.0, 1.426, 1.571, 1.571
        ])

        self.left_release_preset3 = np.array([
            -1.047, 1.571, 0.436, 0.0,
            0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0
        ])

        self.right_grasp_preset3 = np.array([
            0.0, -1.571, 1.311, 1.047,
            0.0, 1.745, 1.396, 1.134,
            0.0, 1.745, 1.483, 1.134,
            0.0, 1.571, 1.396, 1.309,
            0.0, 1.426, 1.571, 1.571
        ])

        self.right_release_preset3 = np.array([
            0.0, -1.571, -1.571, 0.0,
            0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0
        ])

        # ------------------------------------------------------------

        if self.side == 'l':
            self.publisher_ = self.create_publisher(JointTrajectory, '/leader/joint_trajectory_command_broadcaster_left_hand/joint_trajectory', 10)
        elif self.side == 'r':
            self.publisher_ = self.create_publisher(JointTrajectory, '/leader/joint_trajectory_command_broadcaster_right_hand/joint_trajectory', 10)

        self.timer_period = 1
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        while True:
            if self.side == 'l':
                print("\nGoal Positions\n(i) Init\n(1g) Preset 1 Grasp\n(1r) Preset 1 Release\n(2g) Preset 2 Grasp\n(2r) Preset 2 Release\n(3g) Preset 3 Grasp\n(3r) Preset 3 Release\n")
                cmd = input("Command : ")

                if cmd == "i":
                    goal = self.init_pose
                    self.publish_trajectory(goal, self.duration)
                elif cmd == "1g":
                    goal = self.left_grasp_preset1
                    self.publish_trajectory(goal, self.duration)
                elif cmd == "1r":
                    goal = self.left_release_preset1
                    self.publish_trajectory(goal, self.duration)
                elif cmd == "2g":
                    goal = self.left_grasp_preset2
                    self.publish_trajectory(goal, self.duration)
                elif cmd == "2r":
                    goal = self.left_release_preset2
                    self.publish_trajectory(goal, self.duration)
                elif cmd == "3g":
                    goal = self.left_grasp_preset3
                    self.publish_trajectory(goal, self.duration)
                elif cmd == "3r":
                    goal = self.left_release_preset3
                    self.publish_trajectory(goal, self.duration)
                else:
                    print("Invalid Command\n")

            elif self.side == 'r':
                print("\nGoal Positions\n(i) Init\n(1g) Preset 1 Grasp\n(1r) Preset 1 Release\n(2g) Preset 2 Grasp\n(2r) Preset 2 Release\n(3g) Preset 3 Grasp\n(3r) Preset 3 Release\n")
                cmd = input("Command : ")

                if cmd == "i":
                    goal = self.init_pose
                    self.publish_trajectory(goal, self.duration)
                elif cmd == "1g":
                    goal = self.right_grasp_preset1
                    self.publish_trajectory(goal, self.duration)
                elif cmd == "1r":
                    goal = self.right_release_preset1
                    self.publish_trajectory(goal, self.duration)
                elif cmd == "2g":
                    goal = self.right_grasp_preset2
                    self.publish_trajectory(goal, self.duration)
                elif cmd == "2r":
                    goal = self.right_release_preset2
                    self.publish_trajectory(goal, self.duration)
                elif cmd == "3g":
                    goal = self.right_grasp_preset3
                    self.publish_trajectory(goal, self.duration)
                elif cmd == "3r":
                    goal = self.right_release_preset3
                    self.publish_trajectory(goal, self.duration)
                else:
                    print("Invalid Command\n")

    def publish_trajectory(self, goal, duration=1):
        msg = JointTrajectory()
        if self.side == 'l':
            msg.joint_names = self.left_joint_names
        elif self.side == 'r':
            msg.joint_names = self.right_joint_names
        goal_point = JointTrajectoryPoint()
        goal_point.positions = goal.tolist()
        goal_point.time_from_start.sec = int(duration)
        msg.points.append(goal_point)
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    test_publisher = TestPublisher()
    rclpy.spin(test_publisher)
    test_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
