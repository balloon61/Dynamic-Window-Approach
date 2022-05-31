#!/usr/bin/env python
import roslib
roslib.load_manifest('learning_tf')

import rospy
import tf
import math
import rospy
import sys, select, os
from astar_search import Astar


from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Float64
from gazebo_msgs.msg import ModelStates
position = [(0.0, 0.0, 0.0)]

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

if __name__ == '__main__':
    rospy.init_node('dynamic_tf_broadcaster')
    br = tf.TransformBroadcaster()
    sub_odom = rospy.Subscriber('/gazebo/model_states', ModelStates, callback)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        t = rospy.Time.now().to_sec() * math.pi
        br.sendTransform((position[-1][0], position[-1][1], 0.0),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "odom1",
                         "map")
        rate.sleep()