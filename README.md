# Robotics software Development with ROS1
Learning robotics using ROS1 on a 7DoF Kuka robotic arm.

Synopsis:
- ROS concepts
- urdf and xacro
- kinematics and inverse kinematics with TF2 transforms
- 

## Setup

source ROS before starting VScode if using ROS extension
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3.8

## Projects

### task 1: basic publisher-listener

![Publisher-Subscriber communication](_images/01_pub-sub.png)

### task 2: 

![Transform publisher](_images/02_markers.png)
![Markers in RVIZ](_images/02_rviz.png)


### task 3: 


## VScode ROS extension

settings

CTRL+SHIFT+P : ros:start/stop roscore, click on ROS1.<distrib> bring roscore status
CTRL+SHIFT+B : catkin_make options for building and sourcing automatically
CTRL+SHIFT+P ros:create terminal --> create Terminal on clien (usefull if running VScode over SSH)
CTRL+SHIFT+P ros:urdf --> uses ROS web tool to display urdf

## VScode ROS debugging

ROS extension only allows launch file debugging
Run & Debug > Select Launch File > ...
1 - Select "ROS" (make sure you do not have a python or cpp file open)
2 - Select "ROS: Launch"
3 - Select package to be debugged
4 - Select the launch file
5 - Optional: In the launch.json file, set filter using "launch" to specify nodes NOT to attach the debugger to.

Best practice: Organize project in multiple launch files with few number of nodes to make it easier to debug (combine nodes that work together)