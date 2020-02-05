# The Incomplete Guide to ROS and Orbbec Astra Cameras
This guide contains instructions for installing and using the Rototic Operating System (ROS) and the Astra camera driver on Ubuntu 18.04 LTS. Similar packages may be available on other package manager repositories for different Linux distributions.

## Installation and Setup

### Cloning This Guide
Please enter the following on the command line, which can be invoked by pressing `Ctrl` + `Alt` + `T`:
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

### Brief Introduction to ROS
The Robotic Operating System is a collection of Linux packages that enables communication between different components of a system via a graph. Such graphs contain nodes, which can be represented each component, that can interact with each other via topics. Specifically, each node can subscribe and publish to one or more topics where once data has been published to a topic every node that's subscribed get pushed with that data.

More details are available at [http://wiki.ros.org/ROS/Tutorials](http://wiki.ros.org/ROS/Tutorials "Official ROS Tutorials")

### Rospy and Camera
The ROS implementation for Python can be imported like so:
```python
import rospy
```
I have written a simple subscriber class that listens for images from a given camera topic. Feel free to inspect the source. To use it, make sure the script you are writing is in `~/ws/src/pkg/src/` then import the class:
```python
from submod import *
```
This wildcard import will import all modules in `submod.py` including `rospy`, `cv2`, and `ImageSubscriber`. To create an example of an `ImageSubscriber` object, invoke the constructor:
```python
obj = ImageSubscriber('/camera/rgb/image_raw')
```
The parameter can be changed to other image topics the camera publishes. To see a list of topics available, type the following in the command line:
```bash
rostopic list
```
To show an image from the camera, continuing from the previous example:
```python
obj.display_image()
```
To obtain an OpenCV object from the camera which can be used for further `cv2` processing:
```python
obj.get_image()
```