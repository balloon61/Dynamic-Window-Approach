#!/usr/bin/env python
import rospy

from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

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


msg = """
Control Robot!
===================
-------ARM--------
w: Move robot arm in +x direction (x points to the front from the robot)
s: Move robot arm in -x direction
a: Grip (left robot arm in +y direction and right robot arm in -y direction)
d: Ungrip
e: Move robot arm in +z direction
q: Move robot arm in -z direction
c: Lift robot arm (rotation with respect to joint 2 (left, right)
z: Unlift robot arm

----Mobile Robot----
t: Increase linear velocity
b: Decrease linear velocity
f: Increase angular velocity
h: Decrease angular velocity
g: Stop the robot

CTRL-C to quit
"""
print(msg)

def control_law(dx, dy, theta):
    distant = sqrt(dx * dx + dy * dy)
    print(distant, theta)
    alpha = np.arctan2(dy, dx) - theta
    beta = -theta - alpha
    return kp * distant, ka * alpha + kb * beta


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

def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min( input, output + slop )
    elif input < output:
        output = max( input, output - slop )
    else:
        output = input

    return output

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

def inverse(dedt, q):
    T_list, T = forward(q)
    J = Jacobian(T_list)
    dqdt = np.linalg.lstsq(J, dedt)
    return dqdt 


speed = 8
turn = 0.5

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

    # parameters
    slide_height = -0.3

    left_q = np.array([slide_height,0,pi/8,-pi/4])
    right_q = np.array([slide_height,0,pi/8,-pi/4])


    left_w = 0.0    
    right_w = 0.0

    target_linear_vel   = 0.0
    target_angular_vel  = 0.0
    control_linear_vel  = 0.0
    control_angular_vel = 0.0

    try:
        while 1:

            key = getKey()

            if key == '\x03':
                break


            left_dedt = np.zeros([6]).astype(np.float64)
            right_dedt = np.zeros([6]).astype(np.float64)
            
            move = 0.05

            if key == 'w': 
                left_dedt[0] += move
                right_dedt[0] += move
            elif  key == 's':
                left_dedt[0] -= move
                right_dedt[0] -= move

            elif key == 'e':
                left_dedt[2] += move
                right_dedt[2] += move
    
            elif key == 'q':
                left_dedt[2] -= move
                right_dedt[2] -= move
            
            elif key == 'd':
                left_dedt[1] += move
                right_dedt[1] -= move
                left_dedt[-1] = 0.1
                right_dedt[-1] = -0.1

            elif key == 'a':
                left_dedt[1] -= move
                right_dedt[1] += move
                left_dedt[-1] = -0.1
                right_dedt[-1] = 0.1

            elif key == 'z':
                left_q[1] += 0.01 
                right_q[1] -= 0.01
            elif key == 'c':
                left_q[1] -= 0.01 
                right_q[1] += 0.01

            elif key == 't':
                #left_w = 3.0
                #right_w = 3.0
                target_linear_vel = clip(target_linear_vel + LIN_VEL_STEP_SIZE, -3, 5)
            elif key == 'b':
                #left_w = -3.0
                #right_w = -3.0
                target_linear_vel = clip(target_linear_vel - LIN_VEL_STEP_SIZE, -3, 5)
            elif key == 'h':
                #left_w = 1.5
                #right_w = -1.5
                target_angular_vel = clip(target_angular_vel - ANG_VEL_STEP_SIZE, -2,  2)
            elif key == 'f':
                #left_w = -1.5
                #right_w = 1.5
                target_angular_vel = clip(target_angular_vel + ANG_VEL_STEP_SIZE, -2,  2)
            elif key == 'g':
                #left_w = 0.0
                #right_w = 0.0
                target_linear_vel = 0.0
                control_linear_vel = 0.0
                target_angular_vel = 0.0
                control_angular_vel = 0.0
            


            
            if left_dedt.sum() != 0:                
                T_list, T = forward_left(left_q)
                l_J = Jacobian(T_list)

                #l_w_J = np.zeros([6,1]).astype(np.float)
                #l_w_J[0] = 1
                #l_J = np.concatenate([l_J,l_w_J], -1) 

                l_dqdt = np.linalg.lstsq(l_J, left_dedt)[0]

                T_list, T = forward_right(right_q)
                r_J = Jacobian(T_list)
                r_dqdt = np.linalg.lstsq(r_J, right_dedt)[0]

                tmp = left_q[0] + l_dqdt[0]

                if tmp < -0.35 or tmp > 0.45:
                    l_J[:, 0] = 0
                    l_dqdt = np.linalg.lstsq(l_J, left_dedt)[0]
                    r_J[:, 0] = 0
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

            #left_w = left_v/0.0381
            #right_w = right_v/0.0381
            #print(left_w, right_w)

            #pub_left_wheel.publish(left_w)
            #pub_right_wheel.publish(right_w)
            twist = Twist()

            control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
            twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0

            control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel

            pub_mobile.publish(twist)
    except:
        print(e)

    finally:

        pub_linear_slide.publish(slide_height)

        pub_left_arm_j1.publish(left_q[1] + l_offset1)
        pub_left_arm_j2.publish(left_q[2] + l_offset2)
        pub_left_arm_j3.publish(left_q[3] + l_offset3)

        pub_right_arm_j1.publish(right_q[1] + r_offset1)
        pub_right_arm_j2.publish(right_q[2] + r_offset2)
        pub_right_arm_j3.publish(right_q[3] + r_offset3)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
