# Differential kinematics

This project  

## Setup Terminal

Build the project:
```bash
cd 04_differential-kinematics
catkin_make
source devel/setup.zsh
```

Execute:

```bash
roscore
rosrun rviz rviz
rosrun tf2_web_republisher tf2_web_republisher 
rosparam set robot_description --textfile "$(rospack find lwr_defs)/urdf/kuka_lwr_arm.urdf"
rosrun robot_sim robot_sim_bringup
rosrun robot_state_publisher robot_state_publisher
rosrun cartesian_control marker_control.py
rosrun cartesian_control cartesian_control.py 
```
Then, in RVIZ:
- set FixedFrame to "world_link"
- add robotModel
- add TF
- add interative markers

Alternatively, use roslaunch
```bash
```


## Packages

- **cartesian_control**

- **robot_sim**

- **tf2_web_republisher**



## Folder structure
```bash

