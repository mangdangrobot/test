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
from vision_msgs.msg import Detection2DArray

# Display
from enum import Enum
from MangDang.mini_pupper.display import Display

# Other packages
import numpy as np


# For display
class BehaviorState(Enum):
    DEACTIVATED = -1
    REST = 0
    TROT = 1
    HOP = 2
    FINISHHOP = 3
    SHUTDOWN = 96
    IP = 97
    TEST = 98
    LOWBATTERY = 99


class ObjectTracker(Node):
    def __init__(self):
        super().__init__('object_tracker')
        # Camera info
        # Check classes in mini_pupper_examples/object_class_list.txt
        self.object_class = 15  # Default objcet class: 15 person
        self.img_height = 300  # PreviewSize in mobilenet_node
        self.img_width = 300  # PreviewSize in mobilenet_node

        # Current pose
        self.current_yaw = 0
        self.current_pitch = 0
        self.current_roll = 0
        self.yaw_vals = []
        self.pitch_vals = []
        self.roll_vals = []

        # Constrain
        self.max_yaw = np.radians(25)
        self.max_pitch = np.radians(25)
        self.max_roll = np.radians(25)

        # Display
        self.display = Display()

        # miss count
        self.target_miss_count = 0

        # Target pose
        self.target_yaw = 0
        self.target_pitch = 0
        self.target_roll = 0
        self.target_quaternion = np.array([1, 0, 0, 0])

        # Others
        self.refresh_timer_period = 1/35  # in seconds (35 FPS for mobilenet)
        self.interp_samples = 35  # interpolation samples
        self.i = 0  # movement index in movement_timer_callback
        # set to True if the target tracking has been started
        self.is_target_position_started = False
        # set to True if the target position has been updated
        self.is_target_position_updated = False
        self.target_found = False

        self.reaction_time = 0.01  # pupper will react in reaction_time seconds
        self.movement_timer_period = self.reaction_time/self.interp_samples
        self.detections_list = []

        # Coefficients
        self.back_to_zero_coefficient = 0.94  # back to zero when target lost
        self.smooth_coefficient = 0.0004  # avoid violent reactions of pupper

        # Topics
        self.subscription_topic = '/color/mobilenet_detections'
        self.publisher_topic = '/body_pose'

        # Subscriber
        self.detections_subscription = self.create_subscription(
            Detection2DArray,
            self.subscription_topic,
            self.detections_listener_callback,
            10
        )

        # Publisher
        self.pose_publisher = self.create_publisher(
            Pose,
            self.publisher_topic,
            10
        )

        # Timer for refreshing target pose
        self.refresh_target_pose_timer = self.create_timer(
            self.refresh_timer_period,
            self.refresh_timer_callback
        )

        # Timer for movement
        self.movement_timer = self.create_timer(
            self.movement_timer_period,
            self.movement_timer_callback
        )

    def detections_listener_callback(self, Detection2DArray_from_camera):
        # Get detections from camera and store in self.detections_list
        self.detections_list = Detection2DArray_from_camera.detections

        # Track object and update target pose
        self.get_logger().debug("Detection2DArray from camera received.")

    def refresh_timer_callback(self):
        if self.detections_list:
            # Track target object and update target pose
            self.track_object(self.object_class, self.detections_list)

            # Interpolate for smooth movement and store in self.yaw_vals, etc.
            self.yaw_vals, self.pitch_vals, self.roll_vals =\
                self.linear_interpolate(
                    self.current_yaw, self.current_pitch, self.current_roll,
                    self.target_yaw, self.target_pitch, self.target_roll,
                    self.interp_samples
                )
            # Update flag to indicate target tracking has been started
            self.is_target_position_started = True
            # Update flag to indicate target position has been updated
            self.is_target_position_updated = True

    def track_object(self, object_class, detections_list):
        for detection in detections_list:
            position_x = detection.bbox.center.position.x
            position_y = detection.bbox.center.position.y
            theta = detection.bbox.center.theta
            id = detection.id
            self.get_logger().debug('Object found: {}' .format(id))
            if int(id) == object_class:
                self.get_logger().info(
                    'Target object found, class: {}' .format(
                        object_class)
                )
                self.target_found = True
                self.target_miss_count = 0
                # Calculate increments
                pitch_increment = (position_y - self.img_height/2) * \
                    self.smooth_coefficient
                yaw_increment = (self.img_width/2-position_x) * \
                    self.smooth_coefficient
                roll_increment = theta

                # Update target camera pose
                self.target_yaw += yaw_increment
                self.target_pitch += pitch_increment
                self.target_roll += roll_increment

                # Display
                self.display.show_state(BehaviorState.TROT)
            else:
                self.get_logger().info(
                    'Target object lost, class: {}, returning to zero' .format(
                        object_class)
                )
                self.target_found = False
                self.target_miss_count += 1
                if self.target_miss_count >= 5:
                    # Update target camera pose
                    self.target_yaw = float(
                        self.target_yaw*self.back_to_zero_coefficient)
                    self.target_pitch = float(
                        self.target_pitch*self.back_to_zero_coefficient)
                    self.target_roll = float(
                        self.target_roll*self.back_to_zero_coefficient)

                    # Display
                    self.display.show_state(BehaviorState.SHUTDOWN)
            self.get_logger().debug('target_yaw:{}\
                 target_pitch:{}\
                     target_roll:{}' .format(
                self.target_yaw, self.target_pitch, self.target_roll))

    def movement_timer_callback(self):
        if self.is_target_position_started:
            # Constrain
            self.yaw_vals[self.i], \
                self.pitch_vals[self.i],\
                self.roll_vals[self.i] =\
                self.constain_target_pose(
                self.yaw_vals[self.i],
                self.pitch_vals[self.i],
                self.roll_vals[self.i]
            )
            # Euler angles to quaternion for current step interpolation
            self.target_quaternion = self.get_quaternion_from_euler(
                self.yaw_vals[self.i],
                self.pitch_vals[self.i],
                self.roll_vals[self.i]
            )

            # Update current pose
            self.current_yaw = self.yaw_vals[self.i]
            self.current_pitch = self.pitch_vals[self.i]
            self.current_roll = self.roll_vals[self.i]

            # Publish interpolated pose
            self.publish_pose()
            self.i = self.i+1

            # Reset index
            if self.i == self.interp_samples:
                self.i = 0
            if self.is_target_position_updated:
                self.is_target_position_updated = False
                self.i = 0

    def constain_target_pose(self, target_yaw, target_pitch, target_roll):
        # constain target_position
        target_yaw = np.clip(target_yaw, -self.max_yaw, self.max_yaw)
        target_pitch = np.clip(target_pitch, -self.max_pitch, self.max_pitch)
        target_roll = np.clip(target_roll, -self.max_roll, self.max_roll)
        return target_yaw, target_pitch, target_roll

    def publish_pose(self):
        # Create message
        msg = Pose()
        msg.orientation.x = self.target_quaternion[0]
        msg.orientation.y = self.target_quaternion[1]
        msg.orientation.z = self.target_quaternion[2]
        msg.orientation.w = self.target_quaternion[3]

        # Publish
        self.pose_publisher.publish(msg)
        self.get_logger().debug('Publishing: "%s"' % msg.orientation)

    def linear_interpolate(self, yaw, pitch, roll,
                           target_yaw, target_pitch, target_roll,
                           num_samples=10):
        """
        Linear interpolation
        Makes the movement of the robot dog smoother
        """
        # Linearly interpolate each angle
        yaw_vals = np.linspace(yaw, target_yaw, num_samples)
        pitch_vals = np.linspace(pitch, target_pitch, num_samples)
        roll_vals = np.linspace(roll, target_roll, num_samples)
        return yaw_vals, pitch_vals, roll_vals

    def get_quaternion_from_euler(self, yaw, pitch, roll):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - \
            np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + \
            np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - \
            np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + \
            np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return [qx, qy, qz, qw]


def main(args=None):
    rclpy.init(args=args)
    object_tracker = ObjectTracker()
    rclpy.spin(object_tracker)
    object_tracker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
