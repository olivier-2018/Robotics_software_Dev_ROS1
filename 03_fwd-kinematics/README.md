# Forward kinematics

This goal of this chapter is to implement the forward kinematics from the **Denavit-Hartenberg** parameters of a 7-DOF KUKA LWR robot arm defined in a URDF file and running in a ROS environment.   

## Setup

### Build the project manually:
```bash
source /opt/ros/noetic/setup.zsh
cd 03_kinematics
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
roslaunch src/robot_mover/launch/launch_robot-mover.launch

roslaunch src/forward_kinematics/launch/launch_fwd_kinematics.launch
```

### Run project manually:

```bash
roscore
rosrun rviz rviz
rosparam set robot_description --textfile "$(rospack find lwr_defs)/urdf/kuka_lwr_arm.urdf"
rosrun robot_sim robot_sim_bringup
rosrun robot_mover mover
rosrun forward_kinematics my_fwd_kinematics.py
```

Then, in RVIZ:
- set FixedFrame to "world_link"
- add robotModel
- add TF


## Packages

- the **FORWARD_KINEMATICS** package consists of the **ForwardsKinematics** class, which:   
    - subscribes to the topic *joint_states* and publishes transforms to *tf*  
    - loads a URDF description of the robot from the ROS parameter server  
    - includes the **compute_transforms** function which performs the main forward kinematics computation and returns the transforms from the world frame to all the links and uses the following input parameters:  
        -  the information on joints and links of the robot  
        -  the current values of all the joints  
    
- the **ROBOT_SIM** package contains  4 libraries:  
    - **robot.cpp** defines the robot class which provides a basic robot simulation with multiple joints, allowing for control over joint positions and velocities and using threading to continuously update joint states and ensures thread safety with mutexes.  
    - **joint_state_publisher.cpp** publishes the states of the robot actuators to the *joint_states* topic through the *sensor_msgs::JointState* message (including position, name, velocity, effort ).
    - **position_controller.cpp** defines the *PositionController* and *PositionCommand* classes which are responsible for subscribing to the *joint_positions* and *joint_command* topics respectively and providing joint position commands as well as updating the robot's joint positions accordingly.  
    - **velocity_controller.cpp** subscribes to the *joint_velocities* topic and update the robot's joint velocities with the velocities from the message. 

- the **ROBOT_MOVER** package contains 1 library:  
    - **mover.cpp** publishes the jointState msg to the *joint_velocities* topic.

- the **LWR_ROBOT** package contains the KUKA LWR robot definition.


## Folder structure
```bash
src
├── CMakeLists.txt -> /opt/ros/noetic/share/catkin/cmake/toplevel.cmake
├── forward_kinematics
│   ├── CMakeLists.txt
│   ├── launch
│   │   └── fwd_kinematics.config.rviz
│   ├── package.xml
│   ├── scripts
│   │   └── my_fwd_kinematics.py
│   └── src
├── lwr_robot
│   └── lwr_defs
│       ├── calibration
│       │   ├── default_cal.xml
│       │   └── ias_cal_190809.xml
│       ├── CMakeLists.txt
│       ├── defs
│       │   ├── gazebo_defs.xml
│       │   ├── kuka_lwr_arm_defs.xml
│       │   ├── materials.xml
│       │   └── util_defs.xml
│       ├── meshes
│       │   └── lwr
│       │       ├── arm_base.dae
│       │       ├── arm_base.mesh
│       │       ├── arm_segment_a.dae
│       │       ├── arm_segment_a.mesh
│       │       ├── arm_segment_b.dae
│       │       ├── arm_segment_b.mesh
│       │       ├── arm_segment_last.dae
│       │       ├── arm_segment_last.mesh
│       │       ├── arm_wrist.dae
│       │       ├── arm_wrist.mesh
│       │       ├── base.STL
│       │       ├── convex
│       │       │   ├── arm_base_convex.stl
│       │       │   ├── arm_segment_a_convex.stl
│       │       │   ├── arm_segment_b_convex.stl
│       │       │   ├── arm_segment_last_convex.stl
│       │       │   ├── arm_wrist_convex.stl
│       │       │   └── README
│       │       ├── COPYRIGHT
│       │       ├── link_1.STL
│       │       ├── link_2.STL
│       │       ├── link_3.STL
│       │       ├── link_4.STL
│       │       ├── link_5.STL
│       │       ├── link_6.STL
│       │       └── link_7.STL
│       ├── package.xml
│       └── robots
│           └── kuka_lwr_arm.urdf.xml
├── robot_mover
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── src
│       └── mover.cpp
└── robot_sim
    ├── CMakeLists.txt
    ├── config
    │   └── rviz.rviz
    ├── include
    │   └── robot_sim
    │       ├── joint_state_publisher.h
    │       ├── position_controller.h
    │       ├── robot.h
    │       └── velocity_controller.h
    ├── package.xml
    ├── scripts
    │   └── position_command.py
    └── src
        ├── joint_state_publisher.cpp
        ├── position_controller.cpp
        ├── robot.cpp
        ├── robot_sim_bringup.cpp
        └── velocity_controller.cpp
```

## References:

- https://www.youtube.com/watch?v=ssHDF_4baJ0

