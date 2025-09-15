import math
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion

class HandJointStatePublisher(Node):

    def __init__(self):
        super().__init__('hand_joint_publisher')
        self.joint_names = [
            "finger_joint_1", "finger_joint_2", "finger_joint_3", "finger_joint_4",
            "finger_joint_5", "finger_joint_6", "finger_joint_7", "finger_joint_8",
            "finger_joint_9", "finger_joint_10", "finger_joint_11", "finger_joint_12",
            "finger_joint_13", "finger_joint_14", "finger_joint_15", "finger_joint_16",
            "finger_joint_17", "finger_joint_18", "finger_joint_19", "finger_joint_20"
        ]

        self.joint_positions = [0.0] * 20

        self.min_joint_limits = [
            -2.2, -2.0, 0.0, 0.0,
            -0.6, 0.0, 0.0, 0.0,
            -0.6, 0.0, 0.0, 0.0,
            -0.6, 0.0, 0.0, 0.0,
            -0.6, 0.0, 0.0, 0.0
        ]

        self.max_joint_limits = [
            0.0, 0.3, 1.57, 1.57,
            0.6, 2.0, 1.57, 1.57,
            0.6, 2.0, 1.57, 1.57,
            0.6, 2.0, 1.57, 1.57,
            0.6, 2.0, 1.57, 1.57,
        ]

        self.subscriber_ = self.create_subscription(
            PoseArray,
            '/vr_hand/right_poses',
            self.subscriber_callback,
            10
        )
        self.subscriber_ # prevent unused variable warning

        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        self.timer_period = 0.0333  # seconds (30 Hz)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.i = 0
        self.delay1 = 0.0
        self.delay2 = 0.5
        self.delay3 = 2*self.delay2
        self.delay4 = 3*self.delay2
        self.scale_joint_limit = 0.8

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.joint_positions
        # msg.position = [
        #     0.0, 0.0, 0.0, 0.0, 
        #     0.0, self.scale_joint_limit*self.max_joint_limits[5]*(0.5*math.sin(2*math.pi*self.i+self.delay1)+0.5), self.scale_joint_limit*self.max_joint_limits[6]*(0.5*math.sin(2*math.pi*self.i+self.delay1)+0.5), self.scale_joint_limit*self.max_joint_limits[7]*(0.5*math.sin(2*math.pi*self.i+self.delay1)+0.5),
        #     0.0, self.scale_joint_limit*self.max_joint_limits[9]*(0.5*math.sin(2*math.pi*self.i+self.delay2)+0.5), self.scale_joint_limit*self.max_joint_limits[10]*(0.5*math.sin(2*math.pi*self.i+self.delay2)+0.5), self.scale_joint_limit*self.max_joint_limits[11]*(0.5*math.sin(2*math.pi*self.i+self.delay2)+0.5),
        #     0.0, self.scale_joint_limit*self.max_joint_limits[13]*(0.5*math.sin(2*math.pi*self.i+self.delay3)+0.5), self.scale_joint_limit*self.max_joint_limits[14]*(0.5*math.sin(2*math.pi*self.i+self.delay3)+0.5), self.scale_joint_limit*self.max_joint_limits[15]*(0.5*math.sin(2*math.pi*self.i+self.delay3)+0.5),
        #     0.0, self.scale_joint_limit*self.max_joint_limits[17]*(0.5*math.sin(2*math.pi*self.i+self.delay4)+0.5), self.scale_joint_limit*self.max_joint_limits[18]*(0.5*math.sin(2*math.pi*self.i+self.delay4)+0.5), self.scale_joint_limit*self.max_joint_limits[19]*(0.5*math.sin(2*math.pi*self.i+self.delay4)+0.5)
        # ]
        msg.velocity = []
        msg.effort = []
        self.publisher_.publish(msg)
        # self.i += 0.02 # self.timer_period
        # self.get_logger().info('Publishing: "%s"' % msg.data)

    def subscriber_callback(self, msg):
        quat0 = msg.poses[0].orientation # wrist
        quat1 = msg.poses[1].orientation
        quat2 = msg.poses[2].orientation
        quat3 = msg.poses[3].orientation
        quat4 = msg.poses[4].orientation

        # quat5 = msg.poses[5].orientation
        quat6 = msg.poses[6].orientation
        quat7 = msg.poses[7].orientation
        quat8 = msg.poses[8].orientation

        # quat10 = msg.poses[10].orientation
        quat11 = msg.poses[11].orientation
        quat12 = msg.poses[12].orientation
        quat13 = msg.poses[13].orientation

        # quat15 = msg.poses[15].orientation
        quat16 = msg.poses[16].orientation
        quat17 = msg.poses[17].orientation
        quat18 = msg.poses[18].orientation
 
        # quat20 = msg.poses[20].orientation
        quat21 = msg.poses[21].orientation
        quat22 = msg.poses[22].orientation
        quat23 = msg.poses[23].orientation

        self.joint_positions[0] = -self.get_roll_pitch_yaw(self.quat_inverse(quat1), quat2, 'r') # + math.pi/2
        self.joint_positions[1] = -self.get_roll_pitch_yaw(self.quat_inverse(quat1), quat2, 'r')
        self.joint_positions[2] = -self.get_roll_pitch_yaw(self.quat_inverse(quat2), quat3, 'r')
        self.joint_positions[3] = -self.get_roll_pitch_yaw(self.quat_inverse(quat3), quat4, 'r')

        self.joint_positions[4] = -self.get_roll_pitch_yaw(self.quat_inverse(quat0), quat6, 'p')
        self.joint_positions[5] = -self.get_roll_pitch_yaw(self.quat_inverse(quat0), quat6, 'r')
        self.joint_positions[6] = -self.get_roll_pitch_yaw(self.quat_inverse(quat6), quat7, 'r')
        self.joint_positions[7] = -self.get_roll_pitch_yaw(self.quat_inverse(quat7), quat8, 'r')

        self.joint_positions[8] = -self.get_roll_pitch_yaw(self.quat_inverse(quat0), quat11, 'p')
        self.joint_positions[9] = -self.get_roll_pitch_yaw(self.quat_inverse(quat0), quat11, 'r')
        self.joint_positions[10] = -self.get_roll_pitch_yaw(self.quat_inverse(quat11), quat12, 'r')
        self.joint_positions[11] = -self.get_roll_pitch_yaw(self.quat_inverse(quat12), quat13, 'r')

        self.joint_positions[12] = -self.get_roll_pitch_yaw(self.quat_inverse(quat0), quat16, 'p')
        self.joint_positions[13] = -self.get_roll_pitch_yaw(self.quat_inverse(quat0), quat16, 'r')
        self.joint_positions[14] = -self.get_roll_pitch_yaw(self.quat_inverse(quat16), quat17, 'r')
        self.joint_positions[15] = -self.get_roll_pitch_yaw(self.quat_inverse(quat17), quat18, 'r')

        self.joint_positions[16] = -self.get_roll_pitch_yaw(self.quat_inverse(quat0), quat21, 'p')
        self.joint_positions[17] = -self.get_roll_pitch_yaw(self.quat_inverse(quat0), quat21, 'r')
        self.joint_positions[18] = -self.get_roll_pitch_yaw(self.quat_inverse(quat21), quat22, 'r')
        self.joint_positions[19] = -self.get_roll_pitch_yaw(self.quat_inverse(quat22), quat23, 'r')

    def quat_inverse(self, q):
        """Returns the inverse of a quaternion."""
        norm = q.x**2 + q.y**2 + q.z**2 + q.w**2
        if norm == 0:
            return Quaternion()
        inv_norm = 1.0 / norm
        msg = Quaternion()
        msg.x = -q.x * inv_norm
        msg.y = -q.y * inv_norm
        msg.z = -q.z * inv_norm
        msg.w = q.w * inv_norm
        return msg

    def quat_multiply(self, q1, q2):
        w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z
        x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y
        y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x
        z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w
        msg = Quaternion()
        msg.x = x
        msg.y = y
        msg.z = z
        msg.w = w
        return msg

    def get_roll_pitch_yaw(self, q1, q2, cmd=''):
        q_combined = self.quat_multiply(q1, q2)
        w, x, y, z = q_combined.w, q_combined.x, q_combined.y, q_combined.z

        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x**2 + y**2)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        sinp = np.clip(sinp, -1, 1) # Clamp value to avoid domain error for asin
        pitch = np.arcsin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y**2 + z**2)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        if cmd == 'r':
            return roll
        elif cmd == 'p':
            return pitch
        elif cmd == 'y':
            return yaw
        else:
            return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args)

    hand_joint_state_publisher = HandJointStatePublisher()

    rclpy.spin(hand_joint_state_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    hand_joint_state_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()