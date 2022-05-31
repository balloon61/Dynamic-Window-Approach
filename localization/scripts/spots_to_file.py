#! /usr/bin/env python

import rospy
from my_turtlebot_localization.srv import PoseSaverMessage, PoseSaverMessageResponse
from geometry_msgs.msg import PoseWithCovarianceStamped

class SaveSpots:
    def __init__(self, srv_name= '/save_spot'):
        self._srv_name= srv_name
        self._pose= PoseWithCovarianceStamped()
        self.pose_srv= rospy.Service(self._srv_name, PoseSaverMessage, self.srv_callback)
        self.sub= rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.sub_callback)
        self.pose_table = {"corner1":self._pose, "corner2":self._pose, "pedestrian":self._pose}

    
    def sub_callback(self, msg):
        self._pose= msg

    def srv_callback(self, req):
        label = req.label
        response = PoseSaverMessageResponse()

        if label == "corner1":
            self.pose_table["corner1"] = self._pose
            response.message = "Saved Pose for corner1 spot"
            
        elif label == "corner2":
            self.pose_table["corner2"] = self._pose
            response.message = "Saved Pose for corner2 spot"
            
        elif label == "pedestrian":
            self.pose_table["pedestrian"] = self._pose
            response.message = "Saved Pose for pedestrian spot"
            
        elif label == "end":
            with open('spots.txt', 'w') as file:
                
                for key, value in self.pose_table.items():
                    if value:
                        file.write(str(key) + ':\n----------\n' + str(value) + '\n===========\n')
                
                response.message = "Written Poses to spots.txt file"
                
        else:
            response.message = "No label with this name. Try with corner1, corner2, pedestrian or end(to write the file)"
        
        
        response.navigation_successfull = True

        return response


if __name__ == "__main__":
    rospy.init_node('pose_save_node')
    pose_saver= SaveSpots()
    rospy.spin()
        

    

