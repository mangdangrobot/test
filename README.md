# Mini Pupper2 with ROS 2 Humble temp repo

Attention: this is ONLY temp repo for internal test, will delete soon.

Supported versions

* Ubuntu 22.04 + ROS 2 Humble

## 1. Installation

### 1.1 PC Setup

Ubuntu 22.04 + ROS 2 Humble is required.  
Please follow the [installation document for ROS Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) or use the [unofficial ROS 2 installation script](https://github.com/Tiryoh/ros2_setup_scripts_ubuntu).

After ROS 2 installation, download the Mini Pupper ROS package in the workspace.

```sh
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/mangdangrobot/test.git -b ros2
mv test mini_pupper_ros
vcs import < mini_pupper_ros/.minipupper.repos --recursive
```

Build and install all ROS packages.

```sh
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
sudo apt-get install ros-humble-teleop-twist-keyboard
colcon build --symlink-install
```

### 1.2 Mini Pupper Setup

Mini Pupper Setup corresponds to the Raspberry Pi on your Mini Pupper.  
Ubuntu 22.04 is required.

Use prepare_sd.py to install ros2 on Mini Pupper 2 following [mini_pupper](https://github.com/mangdangroboticsclub/mini_pupper) repo 

After ROS 2 installation, do the below steps

```sh
cd ~/ros2_ws/
sudo rm -rf src
mkdir src
cd src
git clone https://github.com/mangdangrobot/test.git -b ros2
mv test mini_pupper_ros
vcs import --recursive < mini_pupper_ros/.minipupper.repos
touch champ/champ/champ_gazebo/AMENT_IGNORE
touch champ/champ/champ_navigation/AMENT_IGNORE
touch mini_pupper_ros/mini_pupper_gazebo/AMENT_IGNORE
touch mini_pupper_ros/mini_pupper_navigation/AMENT_IGNORE
```

Build and install all ROS packages.

If the Raspberry Pi has less than 4GB memory, try `MAKEFLAGS=-j1 colcon build --executor sequential --symlink-install` instead of `colcon build --symlink-install`

```sh
# install dependencies without unused heavy packages
cd ~/ros2_ws
colcon build --symlink-install
```

Uset the below steps to test Lidar
```sh
sudo vi /home/ubuntu/ros2_ws/src/mini_pupper_ros/mini_pupper_description/urdf/mini_pupper_description.urdf.xacro
# change rpy="0 0 1.57"  to rpy="0 0 -1.57" if the Lidar direction is 180 deg difference with Mini Pupper

sudo vi /home/ubuntu/ros2_ws/src/mini_pupper_ros/mini_pupper_bringup/launch/lidar.launch.py
# check the right name for Lidar, maybe serial0 or serial1
# change {'port_name': '/dev/ttyUSB0'} to {'port_name': '/dev/serial0'} 
```


## 2. Quick Start for SLAM and Nav.

## 2.1 on Mini Pupper 2

```sh
# Terminal 1
. ~/ros2_ws/install/setup.bash
ros2 launch mini_pupper_bringup bringup.launch.py
```

```sh
# Terminal 2
. ~/ros2_ws/install/setup.bash
ros2 launch mini_pupper_driver imu_interface.launch.py
```

## 2.2 PC

```sh
ros2 launch mini_pupper_navigation bringup.launch.py slam:=true
```

