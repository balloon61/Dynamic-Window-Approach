#!/usr/bin/env python
import rospy

from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, Pose, Point
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion

import sys, select, termios, tty

import sympy as sp
from sympy.physics.mechanics import dynamicsymbols, Point, ReferenceFrame
from math import pi
import numpy as np
import time

PI = 3.141592
l_offset1 = 0.11
l_offset2 = 0.14
l_offset3 = 1.21

r_offset1 = -0.11
r_offset2 = -1.79
r_offset3 = 1.19

LIN_VEL_STEP_SIZE = 0.02
ANG_VEL_STEP_SIZE = 0.1

MAX_LIN_VEL = 0.5
MAX_ANG_VEL = 0.5

position = [(0.0, 0.0, 0.0)]
orientation = [0.0]

goal_x = 4.0
goal_y = 4.0
goal_theta = 0.9



k1 = 5
k2 = 0
k3 = 0.6

def control_law(x_goal, y_goal, t_goal, px, py, theta, v_d, theta_d):
    if (x_goal - px) * (x_goal - px) + (y_goal - py) * (y_goal - py) < 0.05:
        return 0, 0
    else:
        return min(v_d * np.cos(theta_d - theta) + k1 * (np.cos(theta) * (x_goal - px) + np.sin(theta) * (y_goal - py)), 0.5), \
               min(k3 * (t_goal - theta), 0.8)

#def control_law(dx, dy, theta, kp, ka, kb):
#    distant = np.sqrt(dx * dx + dy * dy)
#    print(distant, theta)
#    alpha = np.arctan2(dy, dx) - theta
#    beta = -theta - alpha
#    return kp * distant, ka * alpha + kb * beta


def mobile_inverse_kinematic(velocity, angular_velocity):
    return velocity / r + angular_velocity * width / r, velocity / r - angular_velocity * width / r


def FK(x, y, t, velocity, angular_velocity):
    return x + velocity * np.cos(t) * dt, y + velocity * np.sin(t) * dt, t + angular_velocity * dt


def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def clip(cur, low, high):
    if cur > high: return high
    if cur < low: return low
    return cur


def forward_left(q):
    
    theta, a, d, alpha = dynamicsymbols("theta a d alpha")
    
    T = sp.Matrix([[sp.cos(theta), -sp.sin(theta)*sp.cos(alpha),sp.sin(theta)*sp.sin(alpha),a*sp.cos(theta)], [sp.sin(theta), sp.cos(theta)*sp.cos(alpha),-sp.sin(alpha)*sp.cos(theta),a*sp.sin(theta)],
        [0,sp.sin(alpha),sp.cos(alpha),d],[0,0,0,1]])

    T1 = T.subs({theta:0,a:0.0635, d:0.13716+q[0], alpha:-pi/2})
    T2 = T.subs({theta:q[1],a:0, d:0.09144, alpha:pi/2})
    T3 = T.subs({theta:q[2],a:0.27559, d:0, alpha:0})
    T4 = T.subs({theta:q[3],a:0.2667, d:0, alpha:0})
    return [T1, T2, T3, T4], T1*T2*T3*T4

def forward_right(q):
    
    theta, a, d, alpha = dynamicsymbols("theta a d alpha")
    
    T = sp.Matrix([[sp.cos(theta), -sp.sin(theta)*sp.cos(alpha),sp.sin(theta)*sp.sin(alpha),a*sp.cos(theta)], [sp.sin(theta), sp.cos(theta)*sp.cos(alpha),-sp.sin(alpha)*sp.cos(theta),a*sp.sin(theta)],
        [0,sp.sin(alpha),sp.cos(alpha),d],[0,0,0,1]])

    T1 = T.subs({theta:0,a:0.0635, d:0.13716+q[0], alpha:pi/2})
    T2 = T.subs({theta:q[1],a:0, d:0.09144, alpha:pi/2})
    T3 = T.subs({theta:q[2],a:0.27559, d:0, alpha:0})
    T4 = T.subs({theta:q[3],a:0.2667, d:0, alpha:0})
    return [T1, T2, T3, T4], T1*T2*T3*T4

def Jacobian(T_list):
    T01 = T_list[0]
    T02 = T01*T_list[1]
    T03 = T02*T_list[2]
    T04 = T03*T_list[3]
    
    z0 = np.array([0,0,1])
    o0 = np.array([0,0,0])
    
    z1 = np.array(T01[0:3,2]).astype(np.float64).squeeze(-1)
    o1 = np.array(T01[0:3,3]).astype(np.float64).squeeze(-1)
    
    z2 = np.array(T02[0:3,2]).astype(np.float64).squeeze(-1)
    o2 = np.array(T02[0:3,3]).astype(np.float64).squeeze(-1)
    
    z3 = np.array(T03[0:3,2]).astype(np.float64).squeeze(-1)
    o3 = np.array(T03[0:3,3]).astype(np.float64).squeeze(-1)
    
    o4 = np.array(T04[0:3,3]).astype(np.float64).squeeze(-1)
    
    J1 = np.zeros(6)
    J1[0:3] = z0
    J2 = np.concatenate([np.cross(z1,o4-o1), z1], 0)
    J3 = np.concatenate([np.cross(z2,o4-o2), z2], 0)
    J4 = np.concatenate([np.cross(z3,o4-o3), z3], 0)
   
    J = np.array([J1,J2,J3,J4]).T
    return J

#def inverse(dedt, q):
#    T_list, T = forward(q)
#    J = Jacobian(T_list)
#    dqdt = np.linalg.lstsq(J, dedt)
#    return dqdt 

def callback1(data):
    bot_x = data.pose.pose.position.x
    bot_y = data.pose.pose.position.y
    bot_z = data.pose.pose.position.z
    orien = data.pose.pose.orientation
    position[0] = (bot_x, bot_y, yaw)
    _, _, yaw = euler_from_quaternion([orien.x, orien.y, orien.z, orien.w])
    orientation[0] = yaw

def callback(data):
    bot_x = data.pose[-1].position.x
    bot_y = data.pose[-1].position.y
    orien = data.pose[-1].orientation
    g_orien = data.pose[-2].orientation
    _, _, yaw = euler_from_quaternion([orien.x, orien.y, orien.z, orien.w])
    position[0] = (bot_x, bot_y, yaw)
    _,_,goal_theta = euler_from_quaternion([g_orien.x, g_orien.y, g_orien.z, g_orien.w])


def inverse(left_q, right_q, direction, offset):
    left_dedt = np.zeros(6).astype(np.float64)
    right_dedt = np.zeros(6).astype(np.float64)

    left_dedt[0] += 0.01*direction[0]
    right_dedt[0] += 0.01*direction[0] 
    left_dedt[1] += 0.001*direction[1]
    right_dedt[1] -= 0.001*direction[1]
    left_dedt[2] += 0.01*direction[2]
    right_dedt[2] += 0.01*direction[2]

    left_dedt[-1] += 0.7*direction[1]
    right_dedt[-1] -= 0.7*direction[1]

    T_list, T = forward_left(left_q)
    init_pos = np.array(T[0:3, 3])
    cur_pos = init_pos

    start_time = time.time()

    while ((init_pos-cur_pos)**2).sum()**0.5 <= offset:
        T_list, T = forward_left(left_q)
        cur_pos = np.array(T[0:3, 3])

        l_J = Jacobian(T_list)

        l_dqdt = np.linalg.lstsq(l_J, left_dedt)[0]

        T_list, T = forward_right(right_q)
        r_J = Jacobian(T_list)
        r_dqdt = np.linalg.lstsq(r_J, right_dedt)[0]

        left_q += l_dqdt*0.1
        right_q += r_dqdt*0.1
        slide_height = left_q[0]

        pub_linear_slide.publish(slide_height)

        pub_left_arm_j1.publish(left_q[1] + l_offset1)
        pub_left_arm_j2.publish(left_q[2] + l_offset2)
        pub_left_arm_j3.publish(left_q[3] + l_offset3)

        pub_right_arm_j1.publish(right_q[1] + r_offset1)
        pub_right_arm_j2.publish(right_q[2] + r_offset2)
        pub_right_arm_j3.publish(right_q[3] + r_offset3)


if __name__ == "__main__":
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
    sub_odom = rospy.Subscriber('/gazebo/model_states', ModelStates, callback)

    # parameters
    slide_height = -0.3

    left_q = np.array([slide_height,0,pi/4,-pi/4])
    right_q = np.array([slide_height,0,pi/4,-pi/4])



    left_w = 0.0    
    right_w = 0.0

    target_linear_vel   = 0.0
    target_angular_vel  = 0.0
    control_linear_vel  = 0.0
    control_angular_vel = 0.0

    cnt = 0
    Low_level_gain = 2.03811951707
    r = rospy.Rate(10)
    twist = Twist()
    for t in range(510):
        # print(t)
        if t == 9:
            x_rec = position[-1][0]
        twist.linear.x = 0.2 * Low_level_gain
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0#-0.4 * Low_level_gain
        pub_mobile.publish(twist)
        r.sleep()
    twist.linear.x = 0.0
    pub_mobile.publish(twist)
    x = position[-1][0]
    print("x:", x, x_rec)
    print("data:", x - x_rec)

    y = position[-1][1]
    th = position[-1][2]


"""
    try:
        while 1:
            key = getKey()

            if key == '\x03':
                break

            pub_linear_slide.publish(slide_height)

            pub_left_arm_j1.publish(left_q[1] + l_offset1)
            pub_left_arm_j2.publish(left_q[2] + l_offset2)
            pub_left_arm_j3.publish(left_q[3] + l_offset3)

            pub_right_arm_j1.publish(right_q[1] + r_offset1)
            pub_right_arm_j2.publish(right_q[2] + r_offset2)
            pub_right_arm_j3.publish(right_q[3] + r_offset3)

            x = position[-1][0]
            y = position[-1][1]
            th = position[-1][2]
            print('x', x, 'y', y, 'theta', th)
            if ((x-goal_x)**2 + (y-goal_y)**2)**0.5 <= 0.6: break
            #control_linear_vel, control_angular_vel = control_law(dx,dy,dth,0.1,1,1)
            control_linear_vel, control_angular_vel = control_law(goal_x, goal_y, goal_theta, x, y, th, 0.0, 0.8)
                
            twist = Twist()

            twist.linear.x = control_linear_vel 
            twist.linear.x = 0.1

            twist.linear.y = 0.0 
            twist.linear.z = 0.0

            twist.angular.x = 0.0
            twist.angular.y = 0.0
           # twist.angular.z = control_angular_vel
            twist.angular.z = 0
            pub_mobile.publish(twist)

        inverse(left_q, right_q, np.array([0,0,1]), 0.2)  
        inverse(left_q, right_q, np.array([0,-1,0]), 0.15)  
        inverse(left_q, right_q, np.array([0,0,1]), 0.2)  

        twist = Twist()

        twist.linear.x = 0.0
        twist.linear.y = 0.0 
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.2

        pub_mobile.publish(twist)

    except:
        print(e)

    finally:
        pub_linear_slide.publish(left_q[0])

        pub_left_arm_j1.publish(left_q[1] + l_offset1)
        pub_left_arm_j2.publish(left_q[2] + l_offset2)
        pub_left_arm_j3.publish(left_q[3] + l_offset3)

        pub_right_arm_j1.publish(right_q[1] + r_offset1)
        pub_right_arm_j2.publish(right_q[2] + r_offset2)
        pub_right_arm_j3.publish(right_q[3] + r_offset3)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
"""
