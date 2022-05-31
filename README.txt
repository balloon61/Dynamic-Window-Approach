# Clone mm3 under ~/catkin_src/src and type the following command

cd ~/catkin_ws
catkin_make 
source devel/setup.bash

=========== Control Robot with Keyboard ===============

roslaunch mm3 demo.launch

# In a new terminal, type:
rosrun mm3 arm_control.py

=========== Robot Navigation and Arm Manipulation Demo =============

roslaunch mm3 demo.launch
# In a new terminal, type:
rosrun mm3 self_control.py
