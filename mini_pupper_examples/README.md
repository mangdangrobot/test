[![Ubuntu VERSION](https://img.shields.io/badge/Ubuntu-22.04-green)](https://ubuntu.com/)
&nbsp;
[![ROS2 VERSION](https://img.shields.io/badge/ROS-ROS%202%20Humble-brightgreen)](http://docs.ros.org/en/humble/index.html)
&nbsp;
[![LICENSE](https://img.shields.io/badge/license-Apache--2.0-informational)](https://github.com/mangdangroboticsclub/mini_pupper_ros/blob/ros2/LICENSE)
&nbsp;
[![Twitter URL](https://img.shields.io/twitter/url?style=social&url=https%3A%2F%2Ftwitter.com%2FLeggedRobot)](https://twitter.com/LeggedRobot)

# Mini Pupper ROS2 Examples

This package contains ROS2 examples for the Mini Pupper robot, a small quadruped robot designed for research and educational purposes. By running the demos below, you can test the features of the Mini Pupper.

## Prerequisites

### Software
- ROS 2 Humble
- Ubuntu 22.04
- mini_pupper_bsp package
- mini_pupper_ros package

### Hardware
- Mini Pupper robot
- Required hardware for the demo (e.g., OAK-D-Lite)
- USB3 cable
- PC

Note: The mini_pupper_bsp package should only be installed on the Mini Pupper, not on the PC. All other software should be installed on both devices.

## Installation

Run `<demo_name>_install.sh` on your PC and Mini Pupper, which is in the `mini_pupper_examples/install_scripts` folder. 

Replace `<demo_name>` with the name of the demo you want to try. 

For example, if you want to run the `object_tracking` demo, run `object_tracking_install.sh`.

```bash
# Navigate to the mini_pupper_ros package
cd mini_pupper_ros
```

```bash
# Grant permissions to programs and scripts
chmod -R 777 mini_pupper_examples
```
```bash
# Navigate to the install_scripts folder
cd mini_pupper_examples/mini_pupper_examples/install_scripts
```
```bash
# Run install scripts
./<demo_name>_install.sh
```

## Quick Start

<details>
<summary>Demo1: Object Tracking</summary>

### Usage:

#### 0. Install dependencies before the demo
```bash
# To mini_pupper_examples/install_scripts folder
./object_tracking_install.sh
```
#### 1. Bring up Mini Pupper

First, bring up the Mini Pupper by running the following commands in Terminal 1:

```bash
# On Mini Pupper
. ~/ros2_ws/install/setup.bash
ros2 launch mini_pupper_bringup bringup.launch.py
```

#### 2. Bring up OAK mobile publisher

Next, bring up the OAK mobile publisher by running the following command in Terminal 2:

```bash
# On Mini Pupper
ros2 launch depthai_examples mobile_publisher.launch.py camera_model:=OAK-D-LITE
```

#### 3. Run OpenCV object tracking demo

Finally, run the OpenCV object tracking demo by running the following commands in Terminal 3:

```bash
# On Mini Pupper or PC
. ~/ros2_ws/install/setup.bash
ros2 launch mini_pupper_examples object_tracking.launch.py 
```

### Notes:
- You can run Terminal 3 either on your PC or on the Mini Pupper.
- Ubuntu server does'nt have GUI for Video display,please `ssh -X` to your pupper. 

### Details for the demo:
`object_tracking.py` imports ROS 2 packages, message types, and other necessary packages for operation. It defines an `ObjectTracker` class, which inherits from the ROS 2 `Node` class. The `ObjectTracker` class contains instance variables for tracking the object, maintaining the current and target poses, interpolating movements for smooth operation, and handling camera information.

The `ObjectTracker` class initializes subscriptions to detection data from the camera and publishes updated pose information for the robot. It also creates timers to refresh the target pose and control the robot's movement.

The script includes several methods for detecting, tracking, and moving towards the target object, such as `detections_listener_callback`, `refresh_timer_callback`, `track_object`, `movement_timer_callback`, and `publish_pose`. Additionally, it provides utility functions for linear interpolation of movements (`linear_interpolate`) and converting Euler angles to quaternions (`get_quaternion_from_euler`).

The main function initializes the ROS 2 system, creates an instance of the `ObjectTracker` class, and begins the ROS 2 spinning loop. The loop continues until the program is terminated, after which the node is destroyed, and the ROS 2 system is shut down.
</details>

This demo showcases the object tracking functionality of the Mini Pupper ROS2. By using the OAK camera, the quadruped robot tracks a target object (such as a bottle, person, or chair) and adjusts its posture to face the target object.

##
<details>
<summary>Demo2: Head Pose Synchronization</summary>

### Usage:

#### 0. Install dependencies before the demo
```bash
# To mini_pupper_examples/install_scripts folder
./head_pose_synchronization_install.sh
```
#### 1. Bring up Mini Pupper

First, bring up the Mini Pupper by running the following commands in Terminal 1:

```bash
# On Mini Pupper
. ~/ros2_ws/install/setup.bash
ros2 launch mini_pupper_bringup bringup.launch.py
```
#### 2. Run the human_head_pose_estimation.py on PC or Pupper
Next, bring up human_head_pose_estimation by running the following command in Terminal 2:
```bash
# To mini_pupper_examples/
cd ~/ros2_ws/src/mini_pupper_ros/mini_pupper_examples/mini_pupper_examples/head_pose_synchronization
python3 human_head_pose_estimation.py 
```

#### 3. Run the demo on pupper
Finally, run the OpenCV head pose synchronization demo by running the following commands in Terminal 3:
```bash
# On Mini Pupper
. ~/ros2_ws/install/setup.bash
ros2 launch mini_pupper_examples head_pose_synchronization.launch.py
```

### Notes:

- Ensure that the OAK Camera is properly connected and recognized by your system.
- The demo may require some tweaking of parameters and thresholds, depending on your specific hardware and environment.
- Ubuntu server does'nt have GUI for Video display,please `ssh -X` to your pupper. It is worth noting that ssh can only transmit low-quality images, if you want to see high-quality images, please run `human_head_pose_estimation.py` on a PC and uncomment `# cv2.imshow(packet.name, packet.frame)` in `human_head_pose_estimation.py`
- For better performance, choose to run the `human_head_pose_estimation.py` on your PC with a USB3 cable connected to the OAK Camera.

### Details for the demo:
**`head_pose_estimation_node.py`**: This script sets up a ROS2 node to estimate the head pose using an OAK Camera with a two-stage neural network. It receives the camera feed, detects the face, and estimates the head pose (yaw, pitch, roll). It publishes the head pose as a `geometry_msgs/Pose` message.

**`pupper_head_pose.py`**: This script sets up another ROS2 node to control the Mini Pupper robot based on the estimated head pose. It subscribes to the `geometry_msgs/Pose` message published by the head pose estimation node and converts the head pose to the robot's movement and publishes the `/body_pose` for the Mini Pupper.

This demo showcases the head posture tracking functionality of the Mini Pupper ROS2. By using the OAK camera, the quadruped robot dog tracks and imitates your head posture changes, shaking its head along with you while enjoying rock music.

</details>

This demo showcases the head posture tracking functionality of the Mini Pupper ROS2. By using the OAK camera, the quadruped robot dog tracks and imitates your head posture changes, shaking its head along with you while enjoying rock music.

## Contributing

Contributions to the Mini Pupper examples are welcome! Please read the contributing guidelines before submitting a pull request.

We appreciate your excellent demos for the Mini Pupper and will share your demo with other users!