#!/usr/bin/env python3
#
# Copyright (c) 2023 MangDang
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

# ROS 2 related packages
import rclpy
from rclpy.node import Node

# Message type packages
from geometry_msgs.msg import Pose

# Display related packages
from enum import Enum
from MangDang.mini_pupper.display import Display

# Other packages
import numpy as np


class DisplayState(Enum):
    # For LCD_display
    # REST = 0 # SHUTDOWN
    # CHASING = 1 # TROT
    # FOCUS = 2 # HOP
    # spin = 3 # FINISHHOP
    DEACTIVATED = -1
    REST = 0
    TROT = 1
    HOP = 2
    FINISHHOP = 3
    SHUTDOWN = 96
    IP = 97
    TEST = 98
    LOWBATTERY = 99


class PupperHeadPose(Node):
    def __init__(self):
        super().__init__('pupper_head_pose')
        # Display
        self.display = Display()
        self.display.show_state(DisplayState.TROT)
        # Constrain
        self.max_yaw = np.radians(10)
        self.max_pitch = np.radians(20)
        self.max_roll = np.radians(15)

        # YPR
        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        # Coefficients
        self.back_to_zero_coefficient = 0.8  # back to zero when target lost

        # Back to zero
        self.back_to_zero_counter = 0
        self.back_to_zero_threshold = 10
        self.back_to_zero_flag = 0
        # Topics
        self.subscription_topic = '/head_pose'
        self.publisher_topic_pose = '/body_pose'

        # Subscriber
        self.subscription = self.create_subscription(
            Pose,
            self.subscription_topic,
            self.head_pose_callback,
            10
        )

        # Publisher
        self.pose_publisher = self.create_publisher(
            Pose,
            self.publisher_topic_pose,
            10
        )
        # Timer for backing to zero
        self.timer = self.create_timer(0.1, self.back_to_zero_callback)

    def head_pose_callback(self, msg):
        target_yaw, target_pitch, target_roll = self.quaternion_to_ypr(
            msg.orientation.x, msg.orientation.y,
            msg.orientation.z, msg.orientation.w)
        # Smoothing
        raw_pose = np.array([target_yaw, target_pitch, target_roll])
        smoothed_pose = self.pose_smoothing(raw_pose)
        # Constrain
        target_yaw, target_pitch, target_roll = self.constain_target_pose(
            smoothed_pose[0], smoothed_pose[1], smoothed_pose[2])
        self.yaw = target_yaw
        self.pitch = target_pitch
        self.roll = target_roll
        # Back to zero counter reset
        self.back_to_zero_counter = 0

        self.pose_publish(target_yaw, target_pitch, target_roll)

    def constain_target_pose(self, target_yaw, target_pitch, target_roll):
        # constain target_position
        target_yaw = np.clip(target_yaw, -self.max_yaw, self.max_yaw)
        target_pitch = np.clip(target_pitch, -self.max_pitch, self.max_pitch)
        target_roll = np.clip(target_roll, -self.max_roll, self.max_roll)
        return target_yaw, target_pitch, target_roll

    def pose_publish(self, head_yaw, head_pitch, head_roll):

        pose_msg = Pose()
        self.target_quaternion = self.ypr_to_quaternion(
            head_yaw, head_pitch, head_roll)
        pose_msg.orientation.x = self.target_quaternion[0]
        pose_msg.orientation.y = self.target_quaternion[1]
        pose_msg.orientation.z = self.target_quaternion[2]
        pose_msg.orientation.w = self.target_quaternion[3]
        self.pose_publisher.publish(pose_msg)
        self.get_logger().info("Publishing head pose:{}".format(pose_msg))

    def ypr_to_quaternion(self, yaw, pitch, roll):
        qx = np.sin(roll/2.0) * np.cos(pitch/2.0) * np.cos(yaw/2.0) - \
            np.cos(roll/2.0) * np.sin(pitch/2.0) * np.sin(yaw/2.0)
        qy = np.cos(roll/2.0) * np.sin(pitch/2.0) * np.cos(yaw/2.0) + \
            np.sin(roll/2.0) * np.cos(pitch/2.0) * np.sin(yaw/2.0)
        qz = np.cos(roll/2.0) * np.cos(pitch/2.0) * np.sin(yaw/2.0) - \
            np.sin(roll/2.0) * np.sin(pitch/2.0) * np.cos(yaw/2.0)
        qw = np.cos(roll/2.0) * np.cos(pitch/2.0) * np.cos(yaw/2.0) + \
            np.sin(roll/2.0) * np.sin(pitch/2.0) * np.sin(yaw/2.0)
        self.get_logger().debug(
            "qx:{:.2f}, qy:{:.2f}, qz:{:.2f}, qw:{:.2f}"
            .format(qx, qy, qz, qw))
        return [qx, qy, qz, qw]

    def quaternion_to_ypr(self, qx, qy, qz, qw):
        yaw = np.arctan2(2*(qw*qz+qx*qy), 1-2*(qy**2+qz**2))
        pitch = np.arcsin(2*(qw*qy-qz*qx))
        roll = np.arctan2(2*(qw*qx+qy*qz), 1-2*(qx**2+qy**2))
        return yaw, pitch, roll

    def pose_smoothing(self, pose, window_size=5):
        if not hasattr(self, 'pose_buffer'):
            self.pose_buffer = []

        self.pose_buffer.append(pose)
        if len(self.pose_buffer) > window_size:
            self.pose_buffer.pop(0)

        smoothed_pose = np.mean(self.pose_buffer, axis=0)
        return smoothed_pose

    def back_to_zero_callback(self):
        # Back to zero flag
        if self.back_to_zero_flag != self.yaw:
            self.back_to_zero_flag = self.yaw
        else:
            self.back_to_zero_counter += 1

        if self.back_to_zero_counter > self.back_to_zero_threshold:
            self.yaw = self.yaw * self.back_to_zero_coefficient
            self.pitch = self.pitch * self.back_to_zero_coefficient
            self.roll = self.roll * self.back_to_zero_coefficient
            self.pose_publish(self.yaw, self.pitch, self.roll)


def main(args=None):
    rclpy.init(args=args)
    pupper_head_pose_node = PupperHeadPose()
    rclpy.spin(pupper_head_pose_node)

    pupper_head_pose_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
