#!/usr/bin/env bash

# Adds the apt repositories
sudo add-apt-repository "deb http://archive.ubuntu.com/ubuntu $(lsb_release -sc) main universe restricted multiverse"

# Adds the ROS repository for apt
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Adds a key for authentication
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Updates the apt repositories
sudo apt update

# Installs the ROS collection
yes | sudo apt install ros-melodic-desktop catkin

# Initializes rosdep which handles dependencies used in ROS projects
sudo rosdep init 
rosdep update 

# Sets the environment variables at the start of each bash session
echo ". /opt/ros/melodic/setup.bash" >> ~/.bashrc
. ~/.bashrc 

# Configures the correct Python packages
yes | sudo apt upgrade python python3 python3-pip
yes | sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential python-catkin-tools
sudo pip3 install pip -U
sudo pip3 install numpy opencv-python cv_bridge
