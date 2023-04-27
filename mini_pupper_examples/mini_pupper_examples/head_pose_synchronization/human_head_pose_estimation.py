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

from depthai_sdk import OakCamera, TwoStagePacket, TextPosition
import numpy as np
import cv2

# ROS2 imports
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose


class HeadPoseEstimationNode(Node):
    # Threshold for head movement
    MIN_THRESHOLD = 0.5
    # Degrees in yaw/pitch/roll to be considered as head movement

    def __init__(self):
        super().__init__('head_pose_estimation_node')
        # Create publishers
        self.publisher_position = self.create_publisher(
            Point, '/head_center_position', 10)
        self.publisher_pose = self.create_publisher(
            Pose, '/head_pose', 10)

        # Create target quaternion
        self.target_quaternion = [0, 0, 0, 1]

        # Create camera and NN
        self.oak = OakCamera()

        self.frame_name = 'head_pose_estimation'
        # set resolution to 300x300 for faster inference
        self.color = self.oak.create_camera(
            'color')
        self.det = self.oak.create_nn('face-detection-retail-0004', self.color)
        # Passthrough is enabled for debugging purposes
        # AspectRatioResizeMode has to be CROP
        # for 2-stage pipelines at the moment
        self.det.config_nn(resize_mode='crop')

        self.emotion_nn = self.oak.create_nn(
            'head-pose-estimation-adas-0001', input=self.det)
        # self.emotion_nn.config_multistage_nn(show_cropped_frames=True)
        # which is for debugging

    def cb(self, packet: TwoStagePacket, visualizer):
        for det, rec in zip(packet.detections, packet.nnData):
            head_yaw = rec.getLayerFp16('angle_y_fc')[0]
            head_pitch = rec.getLayerFp16('angle_p_fc')[0]
            head_roll = rec.getLayerFp16('angle_r_fc')[0]

            vals = np.array([abs(head_pitch), abs(head_yaw), abs(head_roll)])
            max_index = np.argmax(vals)

            if vals[max_index] < self.MIN_THRESHOLD:
                continue

            # Calculate the center of the bounding box
            center_x = (det.top_left[0] + det.bottom_right[0]) / 2
            center_y = (det.top_left[1] + det.bottom_right[1]) / 2

            # Publish the center position of the detected face
            position_msg = Point()
            position_msg.x = center_x
            position_msg.y = center_y
            position_msg.z = 0.0
            self.publisher_position.publish(position_msg)
            self._logger.info(f"Center position: {position_msg}")

            # Publish the head pose
            pose_msg = Pose()
            head_yaw_rad = np.radians(head_yaw)
            head_pitch_rad = np.radians(head_pitch)
            head_roll_rad = np.radians(head_roll)
            self.target_quaternion = self.ypr_to_quaternion(
                head_yaw_rad, head_pitch_rad, head_roll_rad)
            pose_msg.orientation.x = self.target_quaternion[0]
            pose_msg.orientation.y = self.target_quaternion[1]
            pose_msg.orientation.z = self.target_quaternion[2]
            pose_msg.orientation.w = self.target_quaternion[3]
            self.publisher_pose.publish(pose_msg)
            self._logger.info(f"Head pose: {pose_msg}")

            # Process text and visualization outside the loop
            txt, values_txt = self.get_head_movement_text(
                head_yaw, head_pitch, head_roll)
            visualizer.add_text(values_txt,
                                bbox=(*det.top_left, *det.bottom_right),
                                position=TextPosition.TOP_LEFT)
            visualizer.add_text(txt,
                                bbox=(*det.top_left, *det.bottom_right),
                                position=TextPosition.BOTTOM_LEFT)
        visualizer.draw(packet.frame)
        # IF you want to see the frames on PC, uncomment the following line
        # cv2.imshow(packet.name, packet.frame)

    def get_head_movement_text(self, head_yaw, head_pitch, head_roll):
        max_index = np.argmax(
            np.array([abs(head_pitch), abs(head_yaw), abs(head_roll)]))

        action = ""
        if max_index == 0:
            action = "Look down" if head_pitch > 0 else "Look up"
        elif max_index == 1:
            action = "Turn right" if head_yaw > 0 else "Turn left"
        elif max_index == 2:
            action = "Tilt right" if head_roll > 0 else "Tilt left"

        values_txt = f"\nYaw: {head_yaw:.2f}\
            \nPitch: {head_pitch:.2f}\
            \nRoll: {head_roll:.2f}"
        return action, values_txt

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

    def oak_run(self):
        # Visualize detections on the frame. Also display FPS on the frame.
        # Don't show the frame but send the packet
        # to the callback function (where it will be displayed)
        self.oak.visualize(self.emotion_nn, callback=self.cb,
                           scale=1/3, fps=True)
        # self.oak.visualize(self.det.out.passthrough)
        self.oak.start(blocking=True)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.oak.__exit__(exc_type, exc_value, traceback)
        cv2.destroyAllWindows()
        pass


def main(args=None):
    rclpy.init(args=args)
    with HeadPoseEstimationNode() as head_pose_estimation_node:
        try:
            head_pose_estimation_node.oak_run()
            rclpy.spin(head_pose_estimation_node)
        finally:
            head_pose_estimation_node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
