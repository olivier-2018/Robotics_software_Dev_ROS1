# Transform manipulations


## Goal
- Learn TF

## Setup

Build the project:
```bash
source /opt/ros/noetic/setup.zsh
cd 02_transforms
# rm -rf build devel (optional)
catkin_make -DCMAKE_EXPORT_COMPILE_COMMANDS=1
cp build/compile_commands.json src/.
source devel/setup.zsh 
```
Note: Copying the "compile_commands.json" file to the src folder allows for cpp ros function completion (need the C/C++ configuration VScode extension).

Package dependencies:
```bash
sudo apt-get install ros-noetic-rviz

```

Execute:
```bash
# Launch Roscor
roscore
# In a different terminal, launch the publisher
rosrun marker_publisher marker_publisher
# In a different terminal, launch the subscriber
rosrun my_transform solution.py
# Launch RVIZ
# on WSL2, you may need: export LIBGL_ALWAYS_INDIRECT=
rosrun rviz rviz
```

In rviz:
- change the Fixed Frame option on the left of the UI. 
- Select "base_frame", and notice that the Global Status now reads "Ok".

- Click Add and select tab "By topic". 
- Select topic /visualization_marker>Marker. 
- you should be able to see the block, cylinder and arrow. 

- You can also add the item "TF" if you want to see a visual representation of the frames.

