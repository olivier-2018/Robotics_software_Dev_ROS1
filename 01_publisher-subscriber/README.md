# ROS1 task1

```bash
cd setup.bash 
source /opt/ros/noetic/setup.bash 
catkin_make
source devel/setup.bash 
```

```bash
# Launch Roscor
roscore
# In a different terminal, launch the publisher
rosrun two_int_talker two_int_talker.py
# In a different terminal, launch the subscriber
rosrun my_listener solution.py
```
