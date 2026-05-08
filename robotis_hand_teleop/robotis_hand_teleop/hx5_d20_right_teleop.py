#!/usr/bin/env python3
#
# Copyright 2026 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Howon Kim

import select
import sys
import termios
import threading
import time
import tty

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
5 : thumb joint1 + / -
6 : thumb joint2 + / -

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
        self.reverse_indices = set()

        # Publisher for Hand joint control
        self.hand_publisher = self.create_publisher(
            JointTrajectory,
            '/leader/joint_trajectory_command_broadcaster_right_hand/joint_trajectory',
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

        self.joint_limits = [
            (-1.57, 2.57), (-2.57, 0.0), (-0.5, 1.57), (-0.5, 1.57),
            (-0.6, 0.6), (0.0, 2.0), (0.0, 1.57), (0.0, 1.57),
            (-0.6, 0.6), (0.0, 2.0), (0.0, 1.57), (0.0, 1.57),
            (-0.6, 0.6), (0.0, 2.0), (0.0, 1.57), (0.0, 1.57),
            (-0.6, 0.6), (0.0, 2.0), (0.0, 1.57), (0.0, 1.57),
        ]

        self.initial_positions = [0.0] * len(self.joint_names)
        self.initial_positions[0] = 1.57
        self.initial_positions[1] = -1.57

        self.shift_joint_map = {
            '1': 4,
            '2': 8,
            '3': 12,
            '4': 16,
            '5': 0,     # thumb joint 1
            '6': 1,     # thumb joint 2
        }
        self.single_joint_direction = +1.0

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
        self.duration = 0.3

        self.running = False

        self.get_logger().info('Waiting for /joint_states ...')
        self.rate = self.create_rate(10)

    def joint_state_callback(self, msg: JointState):
        name_to_idx = {name: i for i, name in enumerate(msg.name)}

        if not all(j in name_to_idx for j in self.joint_names):
            return

        for i, name in enumerate(self.joint_names):
            self.current_positions[i] = msg.position[name_to_idx[name]]

        if not self.running:
            self.target_positions = self.current_positions.copy()

            self.reset_thumb_shift_joints()
            self.clamp_targets()

            self.running = True
            self.get_logger().info('Initial joint states received.')
            self.print_targets()

    # Reset thumb shift joints to their home pose
    def reset_thumb_shift_joints(self):
        idx1 = self.joint_names.index('finger_r_joint1')
        idx2 = self.joint_names.index('finger_r_joint2')

        self.target_positions[idx1] = self.clamp(1.57, idx1)
        self.target_positions[idx2] = self.clamp(-1.57, idx2)

    def clamp(self, x, joint_idx):
        lower, upper = self.joint_limits[joint_idx]
        return max(lower, min(upper, x))

    def clamp_targets(self):
        for i, target in enumerate(self.target_positions):
            self.target_positions[i] = self.clamp(target, i)

    def publish_trajectory(self):
        self.clamp_targets()

        msg = JointTrajectory()
        msg.joint_names = self.joint_names

        pt = JointTrajectoryPoint()
        pt.positions = self.target_positions.copy()
        pt.time_from_start.sec = int(self.duration)
        pt.time_from_start.nanosec = int((self.duration - int(self.duration)) * 1e9)

        msg.points.append(pt)
        self.hand_publisher.publish(msg)

    def toggle_shift_joint_direction(self):
        self.single_joint_direction *= -1.0
        direction_str = '+' if self.single_joint_direction > 0 else '-'
        self.get_logger().info(f'Shift joint direction changed to: {direction_str}')

    def update_shift_joint(self, joint_idx):
        self.target_positions[joint_idx] = self.clamp(
            self.target_positions[joint_idx] + self.single_joint_direction * self.step,
            joint_idx
        )

        self.publish_trajectory()

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
                self.target_positions[idx] + direction * self.step,
                idx
            )

        self.publish_trajectory()

    def open_all(self):
        self.target_positions = self.initial_positions.copy()
        self.publish_trajectory()
        self.get_logger().info('Return to fixed initial pose')

    def close_all(self):
        for i in range(len(self.joint_names)):
            if i in self.fixed_indices:
                continue
            elif i in self.reverse_indices:
                self.target_positions[i] = self.joint_limits[i][0]
            else:
                self.target_positions[i] = self.joint_limits[i][1]

        self.reset_thumb_shift_joints()
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

            if key == '\x1b':   # ESC
                break
            elif key == '\t':   # TAB
                self.toggle_shift_joint_direction()
            elif key == 'z':    # Release Hand
                self.open_all()
            elif key == 'x':    # Close Hand
                self.close_all()
            elif key == 'b':    # Print current joint_states
                self.print_targets()
            elif key in self.shift_joint_map:
                self.update_shift_joint(self.shift_joint_map[key])
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
    except KeyboardInterrupt:
        print('\nCtrl+C detected. Shutting down...')
        node.running = False
        thread.join()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
