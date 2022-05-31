#! /usr/bin/env python

import rospy
from std_srvs.srv import Empty, EmptyRequest

rospy.init_node('init_particle_node')
rospy.wait_for_service('/global_localization')
service = rospy.ServiceProxy('/global_localization', Empty)
req= EmptyRequest()
res= service(req)
print(res)