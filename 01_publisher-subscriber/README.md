# Publisher-subscriber communication


## Goal
- Setup CmakeLists and package.xml dependencies to define new package "two_int_listener"
- publish the topic "two_int_talker"
- subscribe to the topic "two_int_listener" and display the sum of the published 2 integers.


## Setup

Build the project:
```bash
source /opt/ros/noetic/setup.zsh
cd 01_publisher-subscriber
# rm -rf build devel (optional)
catkin_make
source devel/setup.zsh 
```

Execute:
```bash
# Launch Roscor
roscore
# In a different terminal, launch the publisher
rosrun two_int_talker two_int_talker.py
# In a different terminal, launch the subscriber
rosrun my_listener_p solution.py
```
