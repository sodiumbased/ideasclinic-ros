# The Incomplete Guide to ROS and Orbbec Astra Cameras
This guide contains instructions for installing and using the Rototic Operating System (ROS) and the Astra camera driver on Ubuntu 18.04 LTS. Similar packages may be available on other package manager repositories for different Linux distributions.

In order to address the difference between bash commands and source code examples, a `$` at the beginning of a line is used to indicate that the following is meant to be entered to the command line. If you wish to copy the commands, remember to exclude the `$`.

## Installation and Setup

### Cloning This Guide
Please enter the following on the command line, which can be invoked by pressing `Ctrl` + `Alt` + `T`:
```bash
$ sudo apt update && sudo apt install git
$ git clone https://github.com/sodiumbased/me780.git
$ cd me780
```
Explanation:

These commands install git on your machine and download this guide

### ROS Installation
Please run the following on the command line:
```bash
$ ./install.sh
```
Explanation:

This script contains instructions for the package manager `apt-get` which updates the repository and installs the necessary ROS packages.

### Catkin Workspace and Package Setup
Please run the following on the command line:
```bash
$ source setup.sh
```
Explanation:

This scripts initializes and builds a catkin workspace, which is used to build custom ROS messages and services as well as compiling source code. Note that if you write Python scripts after the initial setup the workspace does not need to be rebuilt every time a modification is made because the scripts get directly interpreted by the CPython interpreter. 

The script also creates a package and it will change the directory for the current user to the source folder of the package where Python/C++ source files can be added. Note that if you create C++ source files you must also modify the `CMakeLists.txt` so that `catkin_build` can find them.

Additionally, this script installs the driver written for the Astra cameras.

## Using ROS/Camera

### Brief Introduction to ROS
The Robotic Operating System is a collection of Linux packages that enables communication between different components of a system. Such graphs contain nodes, which can represent each component, that can interact with each other via topics. The network formed by these nodes and topics is known as a graph. Specifically, each node can subscribe and publish to one or more topics where once data has been published to a topic every node that's subscribed get pushed with that data. The following is a graphical representation of the interactions:

![Block diagram of ROS on the Jetson Nano](https://github.com/sodiumbased/me780/blob/master/rosflowchart.png)

More details are available at [http://wiki.ros.org/ROS/Tutorials](http://wiki.ros.org/ROS/Tutorials "Official ROS Tutorials")

### Rospy and Camera
The ROS implementation for Python can be imported like so:
```python
import rospy
```
To properly utilize the ROS graph, you need to write a script for each node to specify their functionalities. Like mentioned earlier, each node can subscribe, publish to different topics, or both:
```python
# creates a node on the ROS graph
ode = rospy.init_node('name_of_the_node')

# subscribes the node to a topic where 'Msg' is an ROS message class and 'foo'
# is a function that takes in one argument as an ROS message object
sub = rospy.Subscriber('/namespace/topic_name', Msg, callback=foo)
```
I have written a simple subscriber class that listens for images from a given camera topic. Feel free to inspect the source. To use it, make sure the script you are writing is in `~/ws/src/pkg/src/` then import:
```python
from submod import *
```
This wildcard import will import all modules in `submod.py` including `rospy`, `cv2`, and `ImageSubscriber`. To create an example of an `ImageSubscriber` object, invoke the constructor:
```python
obj = ImageSubscriber('/camera/rgb/image_raw')
```
The parameter can be changed to other image topics the camera publishes. To see a list of topics available, type the following in the command line:
```bash
$ rostopic list
```
To show an image from the camera, continuing from the previous example:
```python
obj.display_image()
```
To obtain an OpenCV object from the camera which can be used for further `cv2` processing:
```python
obj.get_image()
```

### Running ROS
An ROS graph performs its tasks when the nodes are running on top of the ROS core service. The core service may be initialized on its own or it can be launched with an existing configuration. To start an empty ROS core (keep in mind that only one instance is allowed at a time):
```bash
$ roscore
```
To run the core service with the cameras enabled:
```bash
$ roslaunch astra_camera astrapro.launch
```
To run a node, first make sure that the Python script is executable:
```bash
$ cd ~/ws/src/pkg/src
$ chmod +x [NAME_OF_FILE] # where NAME_OF_FILE is like 'listener.py', for example
```
Or in the case of C++, make sure the `CMakeLists.txt` of the package is properly modified and that the workspace is rebuilt using `catkin_make`. To run a node in an existing core:
```bash
$ rosrun [PKG_NAME] [NODE_NAME] # where PKG_NAME is the name of the package and NODE_NAME is the name of the file that the node is written in
```
