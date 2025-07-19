# Robotics software Development with ROS1
Learning robotics using ROS1 on a 7DoF Kuka robotic arm.

**Synopsis:**  
- ROS concepts
- urdf and xacro
- publisher-listener ROS concept
- kinematics & IK with TF2 transforms
- Cartesian control and differential kinematics
- Control space and motion control
- Path planning and collision avoidance with A* algorithm

## Setup and dependencies
sudo apt install python3-tk


## Tutorials

### turorial 1: basic publisher-listener
This tuorial is based on ROS pub-sub tutorial and simply showcases how to publish a msg (topic) with 2 integers and how to subscribe to it, perform a simple addition of the integers and print the sum on the screen. Nothing fancy. 

<img src="_images/01_pub-sub.png" alt="Publisher-Subscriber communication" width="1024">

### turorial 2: TF transforms between object frames
This tutorial teaches how to work with TFs, an essential concept in robotics and ROS.

<img src="_images/02_markers.png" alt="Transform publisher" width="1024">
<img src="_images/02_rviz.png" alt="Markers in RVIZ" width="1024">


### turorial 3: TF transforms on KUKA LWR 7-DOF joint frames
This tutorial build on the previous ones and illustrates how to evaluate a robot's inverse kinematics and publish the TFs for a 7-DOF KUKA robot.

<img src="_images/03_kuka_home.png" alt="Robot home" height="500">
<img src="_images/03_kuka_max.png" alt="Robot flex" height="500">

### turorial 4: Cartesian control for the End-Effector
This tutorial illustrates how to control the position of the End-Effector relatively to its current position.  
This concept is slightly more advanced and deals with kinematics, inverse kinematics, differential kinematics and singularities avoidance.  One important concept introduced is of the **numerical Jacobian** computation, an essential tool for robot manpulation.   

This tutorial also introduces the concept of **null space control** whereby a redundant robot (a robot with more DOFs than the minimum required to achieve a given pose) can achieve a target EE pose in mutliple ways similarly to the human arm.  

<img src="_images/04_Kuka_manual.png" alt="Manual EE control" height="500">
<img src="_images/04_kuka_markers.png" alt="Marker EE control" height="500">

### turorial 5: Motion planning with A* algorithm
This tutorial builds up on prior tutorials and illustrates how to compute a path for the End-Effector in the  the **configuration space** of the robot i.e the space of possible joint configurations of the robot, using the **A* algorithm**.  

The A* algorithm finds the shortest route between two points by combining the **actual cost** from the start and an **estimated cost** to the goal to decide which path to explore next. This approach balances speed and accuracy, and efficiently picks the most promising route forward.



## VScode ROS extension

Some shortcuts:
- CTRL+SHIFT+P : select "ros:start" to start roscore and source devel (click on ROS1.<distrib> to bring roscore status)  
- CTRL+SHIFT+P : select "ros: stop" to stop roscore   
- CTRL+SHIFT+P : select "ros: Update Python path", and ensure python interpreter is correct (/usr/bin/python3)     
- CTRL+SHIFT+P : select "ros: Update C++ properties"   
- CTRL+SHIFT+B : select "catkin_make: build" to  build project and sourcing automatically  
- CTRL+SHIFT+P : "ros:create terminal" --> create Terminal on clien (usefull if running VScode over SSH)  
- CTRL+SHIFT+P : "ros:urdf" --> uses ROS web tool to display urdf  

<img src="_images/00_ROS_extension.png" alt="ROS extension menu" height="400">

## VScode ROS debugging

ROS extension only allows launch file debugging  
Run & Debug > Select Launch File > ...  
1 - Select "ROS" (make sure you do not have a python or cpp file open)  
2 - Select "ROS: Launch"  
3 - Select package to be debugged  
4 - Select the launch file  
5 - Optional: In the launch.json file, set filter using "launch" to specify nodes NOT to attach the debugger to.  

**Best practice:**  
Organize project in multiple launch files with few number of nodes to make it easier to debug (combine nodes that work together)  

**Note:**  
source the following:
```bash
export ROSCONSOLE_FORMAT="[\${severity}] [\${time:%Y-%m-%d %H:%M:%S}]: \${message}"
```

## Acknowledgements:

Big thanks to:
- RobotWebTools for their github repo (https://github.com/RobotWebTools/tf2_web_republisher)
- Ranch-Hand-Robotics for their ROS1 (& ROS2) VScode extensions
- ROS cheatsheet: https://mirror.umd.edu/roswiki/attachments/de/ROScheatsheet.pdf

## TODOs:
- implement on ROS2
