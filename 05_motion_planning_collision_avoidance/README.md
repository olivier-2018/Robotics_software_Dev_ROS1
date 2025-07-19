# Path planning and collision avoidance 

This tutorial builds up on prior tutorials and illustrates how to compute a path for the End-Effector in the  the **configuration space** of the robot i.e the space of possible joint configurations of the robot, using the **A* algorithm**.  

The A* algorithm finds the shortest route between two points by combining the **actual cost** from the start and an **estimated cost** to the goal to decide which path to explore next. This approach balances speed and accuracy, and efficiently picks the most promising route forward.

## Setup

### Build the project manually:
```bash
cd 05_motion_planning_collision_avoidance
catkin_make
source devel/setup.zsh
```

### Build the project with VScode ROS extension

- install VScode extension
- source ROS distro 
- From VScode,
    - initiate roscore (ctrl+shft+P, then select "ros: start")
    - Update python path (ctrl+shft+P, then select "ros: Update Python path")
    - Check using correct python interpreter (ctrl+shft+P, then select "Python: select interpreter")
    - Build (ctrl+shft+B), select "catkin_build: build"


## Launch the project

### Run project using roslaunch:
```bash
roslaunch src/cartesian_control/launch/.launch
```


### Run project manually:
```bash
roscore

```
Then, in RVIZ:
- set 



## Packages

- **cartesian_control**

- **robot_sim**

- **tf2_web_republisher**



## Folder structure
```bash

```