# Differential kinematics


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
rosparam set robot_description --textfile "$(rospack find robot_sim)/urdf/kuka_lwr_arm.urdf"
rosrun robot_sim robot_sim_bringup
rosrun robot_state_publisher robot_state_publisher
rosrun cartesian_control marker_control.py
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

