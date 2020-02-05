#!/usr/bin/env bash

# Creates and initializes catkin workspace called "ws"
mkdir -p ~/ws/src
cd ~/ws
catkin_make

# Updates the local environment variables to ROS
echo ". $(pwd)/devel/setup.bash" >> ~/.bashrc
. ~/.bashrc 

# Creates a package called "pkg"
cd src
catkin_create_pkg pkg std_msg rospy roscpp
cd ..
catkin_make

. ~/.bashrc

# Inserts a simple Python subscriber class file to the source folder of the package
cp ~/me780/submod.py src/pkg/src/
sudo cp ~/me780/core.py /opt/ros/melodic/lib/python2.7/dist-packages/cv_bridge/core.py

# Installs the Astra camera driver
sudo apt install ros-$ROS_DISTRO-rgbd-launch ros-$ROS_DISTRO-libuvc ros-$ROS_DISTRO-libuvc-camera ros-$ROS_DISTRO-libuvc-ros
cd src
git clone https://github.com/orbbec/ros_astra_camera.git
roscd astra_camera
./scripts/create_udev_rules
cd ../..
catkin_make --pkg astra_camera

