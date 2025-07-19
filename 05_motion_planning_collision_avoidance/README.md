# Path planning and collision avoidance 

This tutorial builds up on prior tutorials and illustrates how to compute a path for the End-Effector in the  the **configuration space** of the robot i.e the space of possible joint configurations of the robot, using the **A* algorithm**.  

The A* algorithm finds the shortest route between two points by combining the **actual cost** from the start and an **estimated cost** to the goal to decide which path to explore next. This approach balances speed and accuracy, and efficiently picks the most promising route forward.

## Setup

### Build the project manually:
```bash
cd 05_motion_planning_collision_avoidance
source /opt/ros/noetic/setup.zsh            
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
roslaunch src/motion_planning/launch/motion_planning.launch
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
rosrun motion_planning motion_planning.py
```
Then, in RVIZ:
- set FixedFrame to "world_link"
- add robotModel
- add TF
- add interative markers



## Packages

- **motion_planning**

- **lwr_robot**

- **lwr_robot_moveit_config**

- **tf2_web_republisher**



## Folder structure
```bash
src/.
├── CMakeLists.txt -> /opt/ros/noetic/share/catkin/cmake/toplevel.cmake
├── lwr_robot
│   └── lwr_defs
│       ├── calibration
│       │   ├── default_cal.xml
│       │   └── ias_cal_190809.xml
│       ├── CMakeLists.txt
│       ├── defs
│       │   ├── gazebo_defs.xml
│       │   ├── kuka_lwr_arm_defs.xml
│       │   ├── materials.xml
│       │   └── util_defs.xml
│       ├── meshes
│       │   └── lwr
│       │       ├── arm_base.dae
│       │       ├── arm_base.mesh
│       │       ├── arm_segment_a.dae
│       │       ├── arm_segment_a.mesh
│       │       ├── arm_segment_b.dae
│       │       ├── arm_segment_b.mesh
│       │       ├── arm_segment_last.dae
│       │       ├── arm_segment_last.mesh
│       │       ├── arm_wrist.dae
│       │       ├── arm_wrist.mesh
│       │       ├── base.STL
│       │       ├── convex
│       │       │   ├── arm_base_convex.stl
│       │       │   ├── arm_segment_a_convex.stl
│       │       │   ├── arm_segment_b_convex.stl
│       │       │   ├── arm_segment_last_convex.stl
│       │       │   ├── arm_wrist_convex.stl
│       │       │   └── README
│       │       ├── COPYRIGHT
│       │       ├── link_1.STL
│       │       ├── link_2.STL
│       │       ├── link_3.STL
│       │       ├── link_4.STL
│       │       ├── link_5.STL
│       │       ├── link_6.STL
│       │       └── link_7.STL
│       ├── package.xml
│       ├── robots
│       │   ├── kuka_lwr_arm.urdf
│       │   └── kuka_lwr_arm.urdf.xml
│       └── urdf
│           └── kuka_lwr_arm.urdf
├── lwr_robot_moveit_config
│   ├── CMakeLists.txt
│   ├── config
│   │   ├── fake_controllers.yaml
│   │   ├── joint_limits.yaml
│   │   ├── kinematics.yaml
│   │   ├── lwr.srdf
│   │   └── ompl_planning.yaml
│   ├── launch
│   │   ├── default_warehouse_db.launch
│   │   ├── demo.launch
│   │   ├── fake_moveit_controller_manager.launch.xml
│   │   ├── joystick_control.launch
│   │   ├── lwr_moveit_controller_manager.launch.xml
│   │   ├── lwr_moveit_sensor_manager.launch.xml
│   │   ├── move_group.launch
│   │   ├── moveit.rviz
│   │   ├── moveit_rviz.launch
│   │   ├── ompl_planning_pipeline.launch.xml
│   │   ├── planning_context.launch
│   │   ├── planning_pipeline.launch.xml
│   │   ├── run_benchmark_ompl.launch
│   │   ├── sensor_manager.launch.xml
│   │   ├── setup_assistant.launch
│   │   ├── trajectory_execution.launch.xml
│   │   ├── warehouse.launch
│   │   └── warehouse_settings.launch.xml
│   └── package.xml
├── motion_planning
│   ├── CMakeLists.txt
│   ├── config
│   │   └── mp.rviz
│   ├── launch
│   │   ├── motion_planning.launch
│   │   └── mp.launch
│   ├── package.xml
│   ├── scripts
│   │   ├── grader.py
│   │   ├── marker_control.py
│   │   ├── motion_planning_orig.py
│   │   ├── motion_planning.py
│   │   ├── obstacle_generator.py
│   │   └── __pycache__
│   │       ├── grader.cpython-38.pyc
│   │       └── obstacle_generator.cpython-38.pyc
│   └── src
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