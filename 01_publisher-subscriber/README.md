# Publisher-subscriber communication


## Goal
- Setup CmakeLists and package.xml dependencies to define new package "two_int_listener"
- publish the topic "two_int_talker"
- subscribe to the topic "two_int_listener" and display the sum of the published 2 integers.


## Setup

### Build the project manually:
```bash
source /opt/ros/noetic/setup.zsh
cd 01_publisher-subscriber
# rm -rf build devel (optional)
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

Execute:
```bash
# Launch Roscor
roscore
# In a different terminal, launch the publisher
rosrun two_int_talker two_int_talker.py
# In a different terminal, launch the subscriber
rosrun my_listener_p solution.py
```


Alternatively, use the launch files:

```bash
roslaunch src/two_int_listener/launch/talker_and_listener.launch   
```


## VScode Setup

- install VScode extension
- source ROS distro 
- From VScode,
    - initiate roscore (ctrl+shft+P, then select "ros: start")
    - Update python path (ctrl+shft+P, then select "ros: Update Python path")
    - Check using correct python interpreter (ctrl+shft+P, then select "Python: select interpreter")
    - Build (ctrl+shft+B), select "catkin_build: build"


## Packages

- **two_int_talker** publishes messages containing two integers to *a topic named *two_ints* topic. 

- **two_int_listener** subscribes to the *two_ints* topic and calculates the sum of the two integers (a and b) contained in the message, logs the sum, and then publishes the sum to another topic named *sum*.


## Folder structure
src
├── CMakeLists.txt -> /opt/ros/noetic/share/catkin/cmake/toplevel.cmake
├── two_int_listener
│   ├── CMakeLists.txt
│   ├── msg
│   │   └── TwoInts.msg
│   ├── package.xml
│   ├── scripts
│   │   └── my_listener.py
│   └── src
└── two_int_talker
    ├── CMakeLists.txt
    ├── package.xml
    ├── scripts
    │   └── two_int_talker.py
    └── src
