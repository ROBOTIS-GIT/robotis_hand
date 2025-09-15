import time
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class TestPublisher(Node):

    def __init__(self):
        super().__init__('test_publisher')
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

        self.present_left_thumb_joints = np.zeros(4)
        self.present_right_thumb_joints = np.zeros(4)

        self.present_left_finger_joints = np.zeros(20)
        self.present_right_finger_joints = np.zeros(20)

        self.left_thumb_subscriber_ = self.create_subscription(
            JointState,
            '/left_thumb/joint_states',
            self.left_thumb_callback,
            10
        )

        self.right_thumb_subscriber_ = self.create_subscription(
            JointState,
            '/right_thumb/joint_states',
            self.right_thumb_callback,
            10
        )

        self.left_finger_subscriber_ = self.create_subscription(
            JointTrajectory,
            '/left_hand/joint_trajectory',
            self.left_finger_callback,
            10
        )

        self.right_finger_subscriber_ = self.create_subscription(
            JointTrajectory,
            '/right_hand/joint_trajectory',
            self.right_finger_callback,
            10
        )

        # self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        self.publisher_ = self.create_publisher(JointTrajectory, '/hand_controller/joint_trajectory', 10)
        # self.publisher_ = self.create_publisher(JointTrajectory, '/leader/joint_trajectory_command_broadcaster_left_hand/joint_trajectory', 10)
        self.timer_period = 0.005
        self.timer = self.create_timer(self.timer_period, self.timer_callback) # JointTrajectory
        # self.timer = self.create_timer(self.timer_period, self.timer_callback2) # JointState

    def timer_callback(self):
        # TODO: combine left and right together?
        msg = JointTrajectory()
        msg.joint_names = self.left_joint_names
        traj_point = JointTrajectoryPoint()
        temp_hand_joints = np.hstack((self.present_left_thumb_joints, self.present_left_finger_joints[4:]))
        traj_point.positions = temp_hand_joints.tolist()
        traj_point.time_from_start.sec = 0
        traj_point.time_from_start.nanosec = 0
        msg.points.append(traj_point)
        self.publisher_.publish(msg)

    def timer_callback2(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.left_joint_names
        temp_hand_joints = np.hstack((self.present_left_thumb_joints, self.present_left_finger_joints[4:]))
        msg.position = temp_hand_joints.tolist()
        self.publisher_.publish(msg)

    def left_thumb_callback(self, msg):
        for i in range(len(self.present_left_thumb_joints)):
            idx = self.left_joint_names.index(msg.name[i])
            self.present_left_thumb_joints[idx] = msg.position[i]

    def right_thumb_callback(self, msg):
        for i in range(len(self.present_right_thumb_joints)):
            idx = self.right_joint_names.index(msg.name[i])
            self.present_right_thumb_joints[idx] = msg.position[i]

    def left_finger_callback(self, msg):
        for i in range(len(self.present_left_finger_joints)):
            idx = self.left_joint_names.index(msg.joint_names[i])
            self.present_left_finger_joints[idx] = msg.points[0].positions[i]

    def right_finger_callback(self, msg):
        for i in range(len(self.present_right_finger_joints)):
            idx = self.right_joint_names.index(msg.joint_names[i])
            self.present_right_finger_joints[idx] = msg.points[0].positions[i]

def main(args=None):
    rclpy.init(args=args)

    test_publisher = TestPublisher()

    rclpy.spin(test_publisher)

    test_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
