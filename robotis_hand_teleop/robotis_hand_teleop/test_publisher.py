import time
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class TestPublisher(Node):

    def __init__(self):
        super().__init__('test_publisher')
        # self.joint_names = [
        #     "finger_joint_1", "finger_joint_2", "finger_joint_3", "finger_joint_4",
        #     "finger_joint_5", "finger_joint_6", "finger_joint_7", "finger_joint_8",
        #     "finger_joint_9", "finger_joint_10", "finger_joint_11", "finger_joint_12",
        #     "finger_joint_13", "finger_joint_14", "finger_joint_15", "finger_joint_16",
        #     "finger_joint_17", "finger_joint_18", "finger_joint_19", "finger_joint_20"
        # ]

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

        self.present_joint_positions = np.zeros(20)
        self.temp_present_joint_positions = np.zeros(20)

        self.goal_threshold = 0.1
        self.duration = 1
        self.print_delay = 0.1

        self.goal_home = np.array([
            0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0
        ])

        # self.goal_v = np.array([
        #     0.0, 0.0, 0.0, 1.5,
        #     0.0, 0.2, 0.0, 0.0,
        #     0.0, 0.0, 0.0, 0.0,
        #     0.0, 0.2, 0.0, 0.0,
        #     0.0, 0.05, 0.0, 0.0
        # ])

        # std::vector<float> min_joint_positions_ = {
        #     0.0, -0.3, 0.0, 0.0
        # };

        # std::vector<float> max_joint_positions_ = {
        #     2.2, 2.0, 1.57, 1.57
        # };

        self.goal_v = np.array([
            0.0, 0.0, 1.5, 1.5,
            0.0, 1.0, 1.5, 1.5,
            0.0, 1.0, 1.5, 1.5,
            0.0, 1.0, 1.5, 1.5,
            0.0, 1.0, 1.5, 1.5
        ])

        self.goal_v2 = np.array([
            0.0, 0.0, 1.5, 1.5,
            0.0, 1.0, 1.5, 1.5,
            0.0, 1.0, 1.5, 1.5,
            0.0, 1.0, 1.5, 1.5,
            0.0, 1.0, 1.5, 1.5
        ])

        self.last_goal = np.array([
            0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0
        ])

        self.subscriber_ = self.create_subscription(
            JointState,
            '/joint_states',
            self.subscriber_callback,
            10
        )

        # self.publisher_ = self.create_publisher(JointTrajectory, '/hand_l_controller/joint_trajectory', 10)
        self.publisher_ = self.create_publisher(JointTrajectory, '/leader/joint_trajectory_command_broadcaster_left_hand/joint_trajectory', 10)
        # self.publisher2_ = self.create_publisher(JointTrajectory, '/leader/joint_trajectory_command_broadcaster_right_hand/joint_trajectory', 10)
        self.timer_period = 0.01
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.initializing = False
        self.start_initialize = True
        self.start_subscribe = False

    def timer_callback(self):
        if self.start_subscribe:
            if np.all(np.abs(self.last_goal - self.present_joint_positions) < self.goal_threshold) and (self.start_initialize==False):
                self.initializing = not self.initializing
                if self.initializing:
                    goal = self.goal_home
                    # goal2 = self.goal_home
                    goal_print = "Home"
                else:
                    goal = self.goal_v
                    # goal2 = self.goal_v2
                    goal_print = "V"
                self.publish_trajectory(goal, goal_print, self.duration)
                # self.publish_trajectory2(goal)
                self.last_goal = goal
                time.sleep(self.duration)
            elif self.start_initialize:
                goal = self.goal_home
                goal_print = "Home"
                self.publish_trajectory(goal, goal_print, self.duration)
                # self.publish_trajectory2(goal)
                self.last_goal = goal
                self.initializing = True
                self.start_initialize = False
                time.sleep(self.duration)
            # else:
                # print(np.max(np.abs(self.last_goal - self.present_joint_positions)))
                # print(self.present_joint_positions)
                # self.publish_trajectory(self.last_goal)
            # else:
            #     print(np.abs(self.last_goal - self.present_joint_positions))
                # self.get_logger().info(f'Not there yet...') #\n{np.abs(self.last_goal - self.present_joint_positions)}')
                # time.sleep(self.print_delay)
        # else:
        #     self.get_logger().info('Not subscribing yet...')

    def publish_trajectory(self, goal, goal_print='retrying..', duration=1):
        msg = JointTrajectory()
        msg.joint_names = self.left_joint_names
        goal_point = JointTrajectoryPoint()
        goal_point.positions = goal.tolist()
        goal_point.time_from_start.sec = int(duration)
        msg.points.append(goal_point)

        self.publisher_.publish(msg)
        self.get_logger().warn(f'Going {goal_print}')

    def publish_trajectory2(self, goal):
        msg = JointTrajectory()
        msg.joint_names = self.right_joint_names
        goal_point = JointTrajectoryPoint()
        goal_point.positions = goal.tolist()
        goal_point.time_from_start.sec = int(self.duration)
        msg.points.append(goal_point)
        self.publisher2_.publish(msg)

    def subscriber_callback(self, msg):
        # self.get_logger().info('Received joint_states')
        for i in range(len(self.present_joint_positions)):
            # idx = self.left_joint_names.index(msg.name[i])
            # self.present_joint_positions[idx] = msg.position[i]
            idx = msg.name.index(self.left_joint_names[i])
            self.temp_present_joint_positions[i] = msg.position[idx]
        self.present_joint_positions = self.temp_present_joint_positions.copy()
        self.start_subscribe = True

def main(args=None):
    rclpy.init(args=args)

    test_publisher = TestPublisher()

    rclpy.spin(test_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    test_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()