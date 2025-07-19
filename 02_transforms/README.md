# Transform manipulations

## Goal
- Learn TF

## Setup

### Build the project manually:
```bash
source /opt/ros/noetic/setup.zsh
cd 02_transforms
# rm -rf build devel (optional)
catkin_make
# option: -DCMAKE_EXPORT_COMPILE_COMMANDS=1
cp build/compile_commands.json src/.
source devel/setup.zsh 
```

Package dependencies:
```bash
sudo apt-get install ros-noetic-rviz
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
roslaunch src/transform_publisher/launch/launch_TF_publisher.launch     
```

### Run project manually:
```bash
# Launch Roscor
roscore
# In a different terminal, launch the publisher
rosrun marker_publisher marker_publisher
# In a different terminal, launch the subscriber
rosrun transform_publisher my_transform.py 
# Launch RVIZ
# on WSL2, you may need: export LIBGL_ALWAYS_INDIRECT=
rosrun rviz rviz
```
Then, in rviz:
- change the Fixed Frame option on the left of the UI. 
- Select "base_frame", and notice that the Global Status now reads "Ok".

- Click Add and select tab "By topic". 
- Select topic /visualization_marker>Marker. 
- you should be able to see the block, cylinder and arrow. 

- You can also add the item "TF" if you want to see a visual representation of the frames.


## VScode Setup

- install VScode extension
- source ROS distro 
- From VScode,
    - initiate roscore (ctrl+shft+P, then select "ros: start")
    - Update python path (ctrl+shft+P, then select "ros: Update Python path")
    - Check using correct python interpreter (ctrl+shft+P, then select "Python: select interpreter")
    - Build (ctrl+shft+B), select "catkin_build: build"


## Packages

- **marker_publisher** publishes visualization markers to the */visualization_marker* 
 topic (purely to vizualise markers). 

- **transform_publisher** uses the tf2_ros.TransformBroadcaster mechanism to publish transforms between different frames. The transforms are evaluated based on the position of objects but this position is currently hard coded.
TODO: read from object position topics


## Folder structure

02_transforms/src
├── CMakeLists.txt -> /opt/ros/noetic/share/catkin/cmake/toplevel.cmake
├── compile_commands.json
├── marker_publisher
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── src
│       └── marker_publisher.cpp
└── transform_publisher
    ├── CMakeLists.txt
    ├── launch
    │   └── launch_TF_publisher.launch
    ├── package.xml
    ├── rviz
    │   └── transform_publisher.config.rviz
    ├── scripts
    │   └── my_transform.py
    └── src
