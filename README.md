# The Incomplete Guide to ROS and Orbbec Astra Cameras
This guide contains instructions for installing and using the Rototic Operating System (ROS) and the Astra camera driver on Ubuntu 18.04 LTS. Similar packages may be available on other package manager repositories for different Linux distributions.

## Installation and Setup

### Cloning This Guide
Please enter the following on the command line:
```bash
sudo apt update && sudo apt install git
git clone https://github.com/sodiumbased/me780.git
cd me780
```
Explanation:

These commands install git on your machine and download this guide

### ROS Installation
Please run the following on the command line:
```bash
./install.sh
```
Explanation:

This script contains instructions for the package manager `apt-get` which updates the repository and installs the necessary ROS packages.

### Catkin Workspace and Package Setup
Please run the following on the command line:
```bash
source setup.sh
```
Explanation:

This scripts initializes and builds a catkin workspace, which is used to build custom ROS messages and services as well as compiling source code. Note that if you write Python scripts after the initial setup the workspace does not need to be rebuilt every time a modification is made because the scripts get directly interpreted by the CPython interpreter. 

The script also creates a package and it will change the directory for the current user to the source folder of the package where Python/C++ source files can be added. Note that if you create C++ source files you must also modify the `CMakeLists.txt` so that `catkin_build` can find them.

Additionally, this script installs the driver written for the Astra cameras.

## Using ROS/Camera