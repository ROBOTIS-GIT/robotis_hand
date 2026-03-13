#!/usr/bin/env python3

import select
import sys
import termios
import tty
import math
import threading
import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


HELP_MSG = """
================ HX5-D20 Right Hand Teleop ================

v / c : finger0 close / open
q / a : finger1 close / open
w / s : finger2 close / open
e / d : finger3 close / open
r / f : finger4 close / open

1 : finger1 left / right 
2 : finger2 left / right
3 : finger3 left / right
4 : finger4 left / right

TAB : toggle left/right (+ / -)

z : all open (release)
x : all close (grip)
b : print current target
ESC : quit

===========================================================
"""

class KeyboardController(Node):
    def __init__(self):
        super().__init__('KeyboardController')

        self.fixed_indices = {0, 4, 8, 12, 16}

        # Publisher for Hand joint control
        self.hand_publisher = self.create_publisher(
            JointTrajectory,
            '/right_hand_controller/joint_trajectory',
            10
        )

        # Subscriber for joint states
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.joint_names = [
            'finger_r_joint1', 'finger_r_joint2', 'finger_r_joint3', 'finger_r_joint4',
            'finger_r_joint5', 'finger_r_joint6', 'finger_r_joint7', 'finger_r_joint8',
            'finger_r_joint9', 'finger_r_joint10', 'finger_r_joint11', 'finger_r_joint12',
            'finger_r_joint13', 'finger_r_joint14', 'finger_r_joint15', 'finger_r_joint16',
            'finger_r_joint17', 'finger_r_joint18', 'finger_r_joint19', 'finger_r_joint20',
        ]

        # For Release
        self.initial_positions = [
            1.57, -1.57, 0.0, 0.0,# 1~4
            0.0, 0.0, 0.0, 0.0,   # 5~8
            0.0, 0.0, 0.0, 0.0,   # 9~12
            0.0, 0.0, 0.0, 0.0,   # 13~16
            0.0, 0.0, 0.0, 0.0,   # 17~20
        ]

        # For 0 joint each finger
        self.number_joint_map = {
            '1': 4,
            '2': 8,
            '3': 12,
            '4': 16,
        }
        self.single_joint_mode = +1.0

        # 0, 4, 8, 12, 16 except for collison
        self.finger_groups = {
            'v': ([2, 3], +1.0),  
            'c': ([2, 3], -1.0),  

            'q': ([5, 6, 7], +1.0),
            'a': ([5, 6, 7], -1.0),

            'w': ([9, 10, 11], +1.0),
            's': ([9, 10, 11], -1.0),

            'e': ([13, 14, 15], +1.0),
            'd': ([13, 14, 15], -1.0),

            'r': ([17, 18, 19], +1.0),
            'f': ([17, 18, 19], -1.0),
        }

        self.current_positions = [0.0] * len(self.joint_names)
        self.target_positions = [0.0] * len(self.joint_names)

        self.step = 0.08
        self.min_limit = -1.5
        self.max_limit = 1.5
        self.duration = 0.3

        self.running = False

        self.get_logger().info('Waiting for /joint_states ...')
        self.rate = self.create_rate(10)

    def enforce_fixed_joints(self):
        # Force a specific joint angle when starting
        idx1 = self.joint_names.index('finger_r_joint1')
        idx2 = self.joint_names.index('finger_r_joint2')

        self.target_positions[idx1] = math.pi / 2.0
        self.target_positions[idx2] = -1 * math.pi / 2.0
    
    def toggle_joint_direction(self):
        self.single_joint_mode *= -1.0
        mode_str = '+' if self.single_joint_mode > 0 else '-'
        self.get_logger().info(f'Single joint mode changed to: {mode_str}')

    def update_single_joint(self, joint_idx):
        self.target_positions[joint_idx] = self.clamp(
            self.target_positions[joint_idx] + self.single_joint_mode * self.step
        )

        self.publish_trajectory()
        # self.print_targets()

        mode_str = '+' if self.single_joint_mode > 0 else '-'
        self.get_logger().info(
            f'Joint [{joint_idx}] {self.joint_names[joint_idx]} moved in {mode_str} mode'
        )

    def joint_state_callback(self, msg: JointState):
        name_to_idx = {name: i for i, name in enumerate(msg.name)}

        if not all(j in name_to_idx for j in self.joint_names):
            return

        for i, name in enumerate(self.joint_names):
            self.current_positions[i] = msg.position[name_to_idx[name]]

        if not self.running:
            self.target_positions = self.current_positions.copy()

            # Fix joint1 and joint2 at 90 degrees
            self.enforce_fixed_joints()

            self.running = True
            self.get_logger().info('Initial joint states received.')
            self.print_targets()

    def clamp(self, x):
        return max(self.min_limit, min(self.max_limit, x))

    def publish_trajectory(self):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names

        pt = JointTrajectoryPoint()
        pt.positions = self.target_positions.copy()
        pt.time_from_start.sec = int(self.duration)
        pt.time_from_start.nanosec = int((self.duration - int(self.duration)) * 1e9)

        msg.points.append(pt)
        self.hand_publisher.publish(msg)

    def update_finger(self, joint_indices, directions):

        if isinstance(directions, (int, float)):
            directions = [float(directions)] * len(joint_indices)

        if len(joint_indices) != len(directions):
            self.get_logger().error(
                f'Length mismatch: joint_indices={len(joint_indices)}, directions={len(directions)}'
            )
            return

        for idx, direction in zip(joint_indices, directions):
            self.target_positions[idx] = self.clamp(
                self.target_positions[idx] + direction * self.step
            )

        self.publish_trajectory()
        # self.print_targets()

    def open_all(self):
        self.target_positions = self.initial_positions.copy()
        self.publish_trajectory()
        self.get_logger().info('Return to fixed initial pose')


    def close_all(self):
        for i in range(len(self.joint_names)):
            if i in self.fixed_indices:
                continue
            else:
                self.target_positions[i] = self.max_limit

        self.enforce_fixed_joints()
        self.publish_trajectory()
        self.get_logger().info('Close all')

    def print_targets(self):
        print("\nCurrent target positions")
        for i, (name, val) in enumerate(zip(self.joint_names, self.target_positions)):
            print(f"[{i:02d}] {name:18s}: {val:+.3f}")
        print("")

    def get_key(self, timeout=0.01):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            rlist, _, _ = select.select([sys.stdin], [], [], timeout)
            if rlist:
                return sys.stdin.read(1)
            return None
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def run(self):
        while rclpy.ok() and not self.running:
            rclpy.spin_once(self, timeout_sec=0.1)

        print(HELP_MSG)

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            key = self.get_key()

            if key is None:
                continue

            if key == '\x1b':  # ESC
                break
            elif key == '\t':  # TAB
                self.toggle_joint_direction()
            elif key == 'z':
                self.open_all()
            elif key == 'x':
                self.close_all()
            elif key == 'b':
                self.print_targets()
            elif key in self.number_joint_map:
                self.update_single_joint(self.number_joint_map[key])
            elif key in self.finger_groups:
                joints, direction = self.finger_groups[key]
                self.update_finger(joints, direction)


def main(args=None):
    rclpy.init()
    node = KeyboardController()

    thread = threading.Thread(target=node.run)
    thread.start()

    try:
        while thread.is_alive():
            time.sleep(0.1)
        # node.run()
    except KeyboardInterrupt:
        print('\nCtrl+C detected. Shutting down...')
        node.running = False
        thread.join()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()