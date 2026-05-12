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

import argparse

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# Thumb joints and each finger's first joint.
DIFF_JOINT_NUMS = {1, 2, 3, 4, 5, 9, 13, 17}


class Hx5D20ExampleMotion(Node):

    def __init__(self, default_hand_side='right'):
        super().__init__('hx5_d20_example_motion')

        self.declare_parameter('hand_side', default_hand_side)
        self.hand_side = self.get_parameter('hand_side').value.lower()
        if self.hand_side not in ('right', 'left'):
            raise ValueError('hand_side must be right or left')

        self.joint_prefix = f'finger_{self.hand_side[0]}_joint'

        self.joint_names = [
            f'{self.joint_prefix}{joint_num}' for joint_num in range(1, 21)
        ]

        self.cmd_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.cmd_pub = self.create_publisher(
            JointTrajectory,
            f'/leader/joint_trajectory_command_broadcaster_{self.hand_side}_hand/joint_trajectory',
            self.cmd_qos
        )

        # Cycle pause
        self.cycle_end = 0.8

        # Timer
        self.check_period = 0.1

        self.fast_steps, self.slow_steps, self.fast_durations, self.slow_durations = (
            self.build_traj_steps()
        )

        self.current_cycle_is_fast = False
        self.current_pose = self.slow_steps[0].copy()

        self.next_cycle_time = self.get_clock().now()

        self.timer = None

        # Avoid ignored startup commands.
        self.publish_dummy_trajectory()
        self.start_timer = self.create_timer(0.5, self.start_after_dummy)

    def base_positions_dict(self, default_rad=0.0):
        base = {}
        for name in self.joint_names:
            base[name] = default_rad
        return base

    def joint_name(self, joint_num):
        return f'{self.joint_prefix}{joint_num}'

    def joint_value(self, joint_num, value):
        if self.hand_side == 'left' and joint_num in DIFF_JOINT_NUMS:
            return -value
        return value

    def finger_pose(self, start_joint_num, values_rad):
        pose = {}
        for i, val in enumerate(values_rad):
            joint_num = start_joint_num + i
            pose[self.joint_name(joint_num)] = self.joint_value(joint_num, val)
        return pose

    def build_traj_steps(self):
        home_pose = self.base_positions_dict(0.0)
        home_pose.update(self.finger_pose(1,  [0.71, -1.03, -0.51, 0.84]))
        home_pose.update(self.finger_pose(5,  [-0.42, 0.91, 0.15, 0.72]))
        home_pose.update(self.finger_pose(9,  [-0.05, 0.59, 0.30, 0.68]))
        home_pose.update(self.finger_pose(13, [0.22, 0.77, 0.14, 0.68]))
        home_pose.update(self.finger_pose(17, [0.36, 0.62, 0.46, 0.67]))

        home_pose_setup = self.base_positions_dict(0.0)
        home_pose_setup[self.joint_name(5)] = self.joint_value(5, -0.42)
        home_pose_setup[self.joint_name(9)] = self.joint_value(9, -0.05)
        home_pose_setup[self.joint_name(13)] = self.joint_value(13, 0.22)
        home_pose_setup[self.joint_name(17)] = self.joint_value(17, 0.36)

        release_pose = self.base_positions_dict(0.0)
        release_pose.update(self.finger_pose(1,  [0.11, 0.04, 0.72, 0.72]))

        grisp = self.base_positions_dict(1.57)
        grisp.update(self.finger_pose(1,  [0.11, 0.04, 0.72, 0.72]))
        grisp[self.joint_name(5)] = 0.0
        grisp[self.joint_name(9)] = 0.0
        grisp[self.joint_name(13)] = 0.0
        grisp[self.joint_name(17)] = 0.0

        pinch_index = home_pose.copy()
        pinch_index.update(self.finger_pose(1,  [0.58, -1.03, 0.45, 0.32]))
        pinch_index.update(self.finger_pose(5,  [-0.42, 1.19, 0.30, 0.71]))

        pinch_middle = home_pose.copy()
        pinch_middle.update(self.finger_pose(1,  [0.32, -1.88, 0.20, 1.55]))
        pinch_middle.update(self.finger_pose(9,  [-0.05, 0.98, 1.46, 0.49]))

        pinch_ring = home_pose.copy()
        pinch_ring.update(self.finger_pose(1,  [0.45, -2.03, 0.20, 1.55]))
        pinch_ring.update(self.finger_pose(13, [0.25, 1.06, 1.24, 0.51]))

        pinch_little = home_pose.copy()
        pinch_little.update(self.finger_pose(1,  [0.90, -2.29, 0.58, 1.00]))
        pinch_little.update(self.finger_pose(17, [0.31, 1.20, 1.25, 0.94]))

        # Step sequence
        fast_steps = [
            home_pose_setup, home_pose, pinch_index, pinch_middle, pinch_ring, pinch_little,
            pinch_ring, pinch_middle, pinch_index, home_pose, release_pose,
        ]
        slow_steps = [
            release_pose, home_pose_setup, home_pose, pinch_index, pinch_middle, pinch_ring,
            pinch_little, pinch_ring, pinch_middle, pinch_index, release_pose, grisp, grisp,
            release_pose,
        ]

        # Step duration
        fast_durations = [0.2] * len(fast_steps)
        slow_durations = [0.5] * len(slow_steps)

        return fast_steps, slow_steps, fast_durations, slow_durations

    def interpolate_pose(self, start_pose, goal_pose, alpha):
        interp_pose = {}
        for joint in self.joint_names:
            s = start_pose[joint]
            g = goal_pose[joint]
            interp_pose[joint] = s + alpha * (g - s)
        return interp_pose

    def make_cycle_trajectory(self, start_pose, steps, durations):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names

        prev_pose = start_pose.copy()
        cumulative_t = 0.0

        for target_pose, duration in zip(steps, durations):
            # Interpolation points
            if duration <= 0.2:
                num_points = 2
            elif duration <= 0.5:
                num_points = 4
            else:
                num_points = 6

            for i in range(1, num_points + 1):
                alpha = i / num_points
                interp_pose = self.interpolate_pose(prev_pose, target_pose, alpha)

                point = JointTrajectoryPoint()
                point.positions = [interp_pose[j] for j in self.joint_names]

                point_t = cumulative_t + alpha * duration
                sec = int(point_t)
                nanosec = int((point_t - sec) * 1e9)
                point.time_from_start.sec = sec
                point.time_from_start.nanosec = nanosec

                msg.points.append(point)

            cumulative_t += duration
            prev_pose = target_pose.copy()

        return msg, cumulative_t, prev_pose

    def publish_dummy_trajectory(self):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = [self.current_pose[j] for j in self.joint_names]
        point.time_from_start.nanosec = 500000000
        msg.points.append(point)

        self.cmd_pub.publish(msg)
        self.get_logger().info('Published dummy trajectory.')

    def start_trajectory_loop(self):
        self.next_cycle_time = self.get_clock().now()
        self.timer = self.create_timer(self.check_period, self.timer_callback)
        self.get_logger().info('Trajectory repeat mode started.')

    def start_after_dummy(self):
        self.start_timer.cancel()
        self.start_trajectory_loop()

    def timer_callback(self):
        now = self.get_clock().now()

        # Wait for the current cycle to finish
        if now < self.next_cycle_time:
            return

        if self.current_cycle_is_fast:
            steps = self.fast_steps
            durations = self.fast_durations
            cycle_name = 'fast'
        else:
            steps = self.slow_steps
            durations = self.slow_durations
            cycle_name = 'slow'

        msg, cycle_duration, final_pose = self.make_cycle_trajectory(
            self.current_pose,
            steps,
            durations
        )
        self.cmd_pub.publish(msg)

        self.get_logger().info(
            f'Published {cycle_name} cycle: steps={len(steps)}, '
            f'duration={cycle_duration:.2f}s, points={len(msg.points)}'
        )

        self.current_pose = final_pose
        self.next_cycle_time = now + Duration(seconds=cycle_duration + self.cycle_end)

        # Toggle the next cycle
        self.current_cycle_is_fast = not self.current_cycle_is_fast


def parse_args(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--hand-side', choices=['right', 'left'], default='right')
    parsed_args, remaining_args = parser.parse_known_args(args)
    return parsed_args, remaining_args


def main(args=None):
    parsed_args, remaining_args = parse_args(args)

    rclpy.init(args=remaining_args)
    node = Hx5D20ExampleMotion(default_hand_side=parsed_args.hand_side)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down by Ctrl+C')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
