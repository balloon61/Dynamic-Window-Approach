#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Pose, Point
import sys, select, os
from astar_search import Astar
if os.name == 'nt':
    import msvcrt, time
else:
    import tty, termios
import numpy as np
import cv2
import heapq
from tqdm import tqdm
import argparse
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Float64
from gazebo_msgs.msg import ModelStates
import sympy as sp
from sympy.physics.mechanics import dynamicsymbols, Point, ReferenceFrame
from math import pi

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

# position = [0.0, 0.0, 0.0]

position = [(0.0, 0.0, 0.0)]
orientation = [0.0]
k1 = 5
k2 = 0
k3 = 2
def control_law(x_goal, y_goal, t_goal, px, py, theta, v_d, theta_d):
    if (x_goal - px) * (x_goal - px) + (y_goal - py) * (y_goal - py) < 0.05:
        return v_d, theta_d
    else:
        return min(v_d * np.cos(t_goal - theta) + k1 * (np.cos(theta) * (x_goal - px) + np.sin(theta) * (y_goal - py)), 0.3), \
               k3 * (t_goal - theta)
def callback(data):
#    print(data.pose)
#    print("!", data.pose[0])
#    print("2", data.pose[1])
#    print("3", data.pose[2])

    bot_x = data.pose[-1].position.x
    bot_y = data.pose[-1].position.y
    orien = data.pose[-1].orientation
    g_orien = data.pose[-2].orientation
    _, _, yaw = euler_from_quaternion([orien.x, orien.y, orien.z, orien.w])
    position[0] = (bot_x, bot_y, yaw)
    _,_,goal_theta = euler_from_quaternion([g_orien.x, g_orien.y, g_orien.z, g_orien.w])

def callback1(msg):

    orien = msg.pose.pose.orientation
    _, _, yaw = euler_from_quaternion([orien.x, orien.y, orien.z, orien.w])
    position[0] = msg.pose.pose.position.x
    position[1] = msg.pose.pose.position.y
    position[2] = yaw

def odometryCb(msg):

    global roll, pitch, yaw, gx, gy, gz
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    (gx, gy, gz) = msg.pose.pose

        

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument('-s','--start', nargs='+', type=float, required=True)
    parser.add_argument('-g','--goal', nargs='+', type=float, required=True)
    parser.add_argument('-rpm','--RPM', nargs='+', type=str, required=True)
    parser.add_argument('-c','--clearance', type=float, default=0.1)
    parser.add_argument('-f','--file_name', type=str, default="visualize")
    args = parser.parse_args()

    if len(args.start) != 3:
        raise Exception("{} is an invalid start position. Should be size of 3".format(args.start))
    if len(args.goal) != 2:
        raise Exception("{} is an invalid goal position. Should be size of 2".format(args.goal))
    if len(args.RPM) != 4: # ROS have 2 init arguments after the last arguments input
        raise Exception("{} is an invalid RPM input. Should be size of 2".format(args.RPM))

    args.start[0] = args.start[0] + 5
    args.start[1] = args.start[1] + 5
    args.start[2] = args.start[2]*180/np.pi
    args.goal[0] = args.goal[0] + 5
    args.goal[1] = args.goal[1] + 5
    RPM_tuple = (float(args.RPM[0]), float(args.RPM[1]))

    # Initialize Map
    # map size (x), map size (y), clearance, diameter of robot, radius of wheel,
    # rpm(left/right), goal threshold, resolution for searching ([x,y], theta)
    # resolution for visualize

    ROBOT_L = 0.2921
    WHEEL_RADIUS = 0.0381
    Low_level_gain = 2.03811951707
    TIME_STEP = 1
    GOAL_TRHES = 0.1

    astar = Astar(10, 10,                 #map_size (x,y)
                  args.clearance,         # clearance
                  ROBOT_L,                # Robot diameter
                  WHEEL_RADIUS,           # Wheel radius
                  TIME_STEP,              # Time duration for each step
                  GOAL_TRHES,                    # goal threshold
                  RPM_tuple,             # rpm for left/rigth wheel
                  (0.1, 15),              # resolution for searching (position/radius)
                   0.01)                  # resolution for visualization

    # Transformation from map to Gazebo

    print(args.start, args.goal)
    # Specify the start position and end position
    # set_start_goal((X,Y,THETA), (X,Y))
    astar.set_start_goal(tuple(args.start), tuple(args.goal))

    astar.search()
    
    # Save to video
    # COMMENT OUT TO VISUALIZE
    astar.visualize("{}.mp4".format(args.file_name))

    #==========================================
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('turtlebot_teleop')

    # publisher
    pub_linear_slide = rospy.Publisher('/mmrobot/linear_joint/command', Float64, queue_size=10)

    pub_left_arm_j1 = rospy.Publisher('/mmrobot/l_arm_joint1/command', Float64, queue_size=10)
    pub_left_arm_j2 = rospy.Publisher('/mmrobot/l_arm_joint2/command', Float64, queue_size=10)
    pub_left_arm_j3 = rospy.Publisher('/mmrobot/l_arm_joint3/command', Float64, queue_size=10)
    pub_left_gripper = rospy.Publisher('/mmrobot/l_gripper_joint/command', Float64, queue_size=10)

    pub_right_arm_j1 = rospy.Publisher('/mmrobot/r_arm_joint1/command', Float64, queue_size=10)
    pub_right_arm_j2 = rospy.Publisher('/mmrobot/r_arm_joint2/command', Float64, queue_size=10)
    pub_right_arm_j3 = rospy.Publisher('/mmrobot/r_arm_joint3/command', Float64, queue_size=10)
    pub_right_gripper = rospy.Publisher('/mmrobot/r_gripper_joint1/command', Float64, queue_size=10)

    pub_mobile = rospy.Publisher('/mmrobot/mobile_controller/cmd_vel', Twist, queue_size=10)
    #sub_odom = rospy.Subscriber('/mmrobot/mobile_controller/odom', Odometry, callback1)
   # sub_odom = rospy.Subscriber('/mmrobot/mobile_controller/odom', Odometry, callback)
  #  rospy.init_node('oodometry', anonymous=True) #make node

    sub_odom = rospy.Subscriber('/gazebo/model_states', ModelStates, callback)

    status = 0
    target_linear_vel   = 0.0
    target_angular_vel  = 0.0
    control_linear_vel  = 0.0
    control_angular_vel = 0.0
    r = rospy.Rate(10/TIME_STEP)

    #l, desire_x, dwsire_y, desire_t = astar.get_robot_actions()
    robot_actions = astar.get_robot_actions()

    index = -1
    print(position)


    for v, w, desire_x, desire_y, desire_th in robot_actions:
       # print(v, w)
       index = index + 1
       for i in range(10):
           #  action_number = 0
           #  action_list = [0, 0, 0, 1, 1, 1, -1, -1, -1, 0, 0, 0, 3]
           # sub_odom = rospy.Subscriber('odom', Odometry, callback)
            #x = position[0]
            #y = position[1]
            #th = position[2]
            x = position[-1][0]
            y = position[-1][1]
            th = position[-1][2]
            goal_theta = desire_th * np.pi / 180
            if goal_theta > 3.1415927:
                goal_theta = goal_theta - 2 * 3.1415927
            elif goal_theta < -3.1415927:
                goal_theta = goal_theta + 2 * 3.1415927
            print(position)
           # print(x, y, th, desire_x - 5, desire_y - 5, goal_theta)
           # print('Fields are', now.secs, now.nsecs)
            twist = Twist()
            if x != 0 or y != 0 or th != 0:
                vel, ang_vel = control_law(desire_x-5, desire_y-5, goal_theta, x, y, th, v, w)
            else:
                vel = v
                ang_vel = w
           #  control_linear_vel, control_angular_vel = action(int(action_list[action_number]))
           #  vel, ang_vel = control_law(desire_x[index], dwsire_y[index], desire_t[index], x, y, th, 0.0, 0.8)
            print(v, w)
            twist.linear.x = vel * Low_level_gain
            twist.linear.y = 0.0
            twist.linear.z = 0.0

            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = ang_vel * Low_level_gain
           #  print('(v, w)', '(', control_linear_vel, ',', control_angular_vel, ')')
            pub_mobile.publish(twist)

           #  if action_number < int(len(action_list) - 1):
           #     action_number = action_number + 1

            r.sleep()

    twist = Twist()
    twist.linear.x = 0.0;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = 0.0
    pub_mobile.publish(twist)

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

