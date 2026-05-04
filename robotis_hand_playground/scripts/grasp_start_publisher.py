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
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class GraspStartKeyboard(Node):

    def __init__(self):
        super().__init__('grasp_start_keyboard')
        self.publisher_ = self.create_publisher(Bool, '/grasp_start', 10)

    def publish_grasp_start(self, value: bool):
        msg = Bool()
        msg.data = value
        self.publisher_.publish(msg)
        self.get_logger().info(f'/grasp_start: {value}')


def read_key(timeout_sec: float = 0.1) -> str:
    readable, _, _ = select.select([sys.stdin], [], [], timeout_sec)
    if readable:
        return sys.stdin.read(1)
    return ''


def main():
    rclpy.init()
    node = GraspStartKeyboard()
    settings = termios.tcgetattr(sys.stdin)

    print('Press z: true, x: false, q: quit')
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            key = read_key()

            if key == 'z':
                node.publish_grasp_start(True)
            elif key == 'x':
                node.publish_grasp_start(False)
            elif key in ('q', '\x03'):
                break
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
