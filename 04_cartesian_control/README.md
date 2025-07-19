# Differential kinematics

This tutorial illustrates how to control the position of the End-Effector relatively to its current position.  
This concept is slightly more advanced and deals with kinematics, inverse kinematics, differential kinematics and singularities avoidance.  One important concept introduced is of the **numerical Jacobian** computation, an essential tool for robot manpulation.   

This tutorial also introduces the concept of **null space control** whereby a redundant robot (a robot with more DOFs than the minimum required to achieve a given pose) can achieve a target EE pose in mutliple ways similarly to the human arm.  


## Setup

### Build the project manually:
```bash
cd 04_differential-kinematics
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
roslaunch src/cartesian_control/launch/launch_marker_control.launch
```
or
```bash
roslaunch src/cartesian_control/launch/launch_manual_control.launch
```

### Run project manually:
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


## Packages

- **robot_sim** (see tutorial 3 on Forward kinematics)

- **lwr_defs** (see tutorial 3 on Forward kinematics)
  
- the **cartesian_control** package evaluates the differential kinematics between the current EE pose and a desired target pose (in the viscinity of the current EE pose). It also handles singularities under the hood.

- the **tf2_web_republisher** package is an external package published by RobotWebTools used as a TF bridge for web clients.


## Folder structure
```bash
src/.
├── cartesian_control
│   ├── CMakeLists.txt
│   ├── config
│   │   └── rviz.rviz
│   ├── launch
│   │   ├── launch_manual_control.launch
│   │   └── launch_marker_control.launch
│   ├── package.xml
│   ├── scripts
│   │   ├── cartesian_control.py
│   │   ├── grader.py
│   │   ├── marker_control.py
│   │   └── __pycache__
│   │       └── grader.cpython-38.pyc
│   └── src
├── CMakeLists.txt -> /opt/ros/noetic/share/catkin/cmake/toplevel.cmake
├── lwr_defs
│   ├── calibration
│   │   ├── default_cal.xml
│   │   └── ias_cal_190809.xml
│   ├── CMakeLists.txt
│   ├── defs
│   │   ├── gazebo_defs.xml
│   │   ├── kuka_lwr_arm_defs.xml
│   │   ├── materials.xml
│   │   └── util_defs.xml
│   ├── meshes
│   │   └── lwr
│   │       ├── arm_base.dae
│   │       ├── arm_base.mesh
│   │       ├── arm_segment_a.dae
│   │       ├── arm_segment_a.mesh
│   │       ├── arm_segment_b.dae
│   │       ├── arm_segment_b.mesh
│   │       ├── arm_segment_last.dae
│   │       ├── arm_segment_last.mesh
│   │       ├── arm_wrist.dae
│   │       ├── arm_wrist.mesh
│   │       ├── base.STL
│   │       ├── convex
│   │       │   ├── arm_base_convex.stl
│   │       │   ├── arm_segment_a_convex.stl
│   │       │   ├── arm_segment_b_convex.stl
│   │       │   ├── arm_segment_last_convex.stl
│   │       │   ├── arm_wrist_convex.stl
│   │       │   └── README
│   │       ├── COPYRIGHT
│   │       ├── link_1.STL
│   │       ├── link_2.STL
│   │       ├── link_3.STL
│   │       ├── link_4.STL
│   │       ├── link_5.STL
│   │       ├── link_6.STL
│   │       └── link_7.STL
│   ├── package.xml
│   ├── robots
│   │   └── kuka_lwr_arm.urdf.xml
│   └── urdf
│       └── kuka_lwr_arm.urdf
├── robot_sim
│   ├── CMakeLists.txt
│   ├── config
│   │   └── rviz.rviz
│   ├── include
│   │   └── robot_sim
│   │       ├── joint_state_publisher.h
│   │       ├── position_controller.h
│   │       ├── robot.h
│   │       └── velocity_controller.h
│   ├── launch
│   │   └── kuka_lwr.launch
│   ├── package.xml
│   ├── scripts
│   │   └── position_command.py
│   └── src
│       ├── joint_state_publisher.cpp
│       ├── position_controller.cpp
│       ├── robot.cpp
│       ├── robot_sim_bringup.cpp
│       └── velocity_controller.cpp
└── tf2_web_republisher
    ├── action
    │   └── TFSubscription.action
    ├── AUTHORS.md
    ├── CHANGELOG.rst
    ├── CMakeLists.txt
    ├── include
    │   └── tf_pair.h
    ├── LICENSE
    ├── mainpage.dox
    ├── msg
    │   └── TFArray.msg
    ├── package.xml
    ├── README.md
    ├── services
    │   └── RepublishTFs.srv
    ├── src
    │   ├── test
    │   │   ├── dummy_transform_publisher.py
    │   │   └── test_tf_web_republisher.py
    │   └── tf_web_republisher.cpp
    └── test
        └── test_tf2_web_republisher.test
```
