# Dynamic-Window-Approach
A implement of global planner (A*) and local planner (Dynamic Window Approach). This package using ROS navigation stack for mapping and locallization, and write the global planner and local planner.

## Requirement:
ROS, Gazebo, ROS navigation stack, numpy, cv2, sys, time
## Step:

Clone mm3 under ~/catkin_src/src and type the following command
```
cd ~/catkin_ws
catkin_make 
source devel/setup.bash
```
### Control Robot with Keyboard 
```
roslaunch mm3 demo.launch

# In a new terminal, type:
rosrun mm3 arm_control.py
```
### Robot Navigation and Arm Manipulation Demo 
```
roslaunch mm3 demo.launch
# Arm Manipulation Demo, make the dual-arm follow a predefined trajectory by using Jacobian 
# In a new terminal, type:
rosrun mm3 self_control.py
```
### path planning and local planner demo
https://www.youtube.com/watch?v=q2g9haRAJWM&ab_channel=Po-LunChen
### Arm control demo
https://www.youtube.com/watch?v=tZF-rN7shDM&ab_channel=%E9%BB%83%E5%93%81%E7%9A%93
