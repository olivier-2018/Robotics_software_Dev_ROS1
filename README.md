# Robotics software Development with ROS1
Learning robotics using ROS1 on a 7DoF Kuka robotic arm.

Synopsis:
- ROS concepts
- urdf and xacro
- kinematics  with TF2 transforms
- 

## Setup and dependencies
sudo apt install python3-tk


## Tutorials

### turorial 1: basic publisher-listener

<img src="_images/01_pub-sub.png" alt="Publisher-Subscriber communication" width="800">

### turorial 2: TF transforms between object frames

<img src="_images/02_markers.png" alt="Transform publisher" width="400">
<img src="_images/02_rviz.png" alt="Markers in RVIZ" width="400">


### turorial 3: TF transforms on KUKA LWR 7-DOF joint frames

<img src="_images/03_kuka_home.png" alt="Robot home" height="500">
<img src="_images/03_kuka_max.png" alt="Robot flex" height="500">


## VScode ROS extension

Some shortcuts:
- CTRL+SHIFT+P : select "ros:start" to start roscore and source devel (click on ROS1.<distrib> to bring roscore status)  
- CTRL+SHIFT+P : select "ros: stop" to stop roscore   
- CTRL+SHIFT+P : select "ros: Update Python path", and ensure python interpreter is correct (/usr/bin/python3)     
- CTRL+SHIFT+P : select "ros: Update C++ properties"   
- CTRL+SHIFT+B : select "catkin_make: build" to  build project and sourcing automatically  
- CTRL+SHIFT+P : "ros:create terminal" --> create Terminal on clien (usefull if running VScode over SSH)  
- CTRL+SHIFT+P : "ros:urdf" --> uses ROS web tool to display urdf  

## VScode ROS debugging

ROS extension only allows launch file debugging  
Run & Debug > Select Launch File > ...  
1 - Select "ROS" (make sure you do not have a python or cpp file open)  
2 - Select "ROS: Launch"  
3 - Select package to be debugged  
4 - Select the launch file  
5 - Optional: In the launch.json file, set filter using "launch" to specify nodes NOT to attach the debugger to.  

Best practice: Organize project in multiple launch files with few number of nodes to make it easier to debug (combine nodes that work together)  

Note: source the following:
```bash
export ROSCONSOLE_FORMAT="[\${severity}] [\${time:%Y-%m-%d %H:%M:%S}]: \${message}"
```
