#!/usr/bin/env python3

import rospy

# Brings in the SimpleActionClient
import actionlib
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Twist
from apriltag_ros.msg import AprilTagDetectionArray
from fellowme_navigation.cfg import ParametersConfig
from dynamic_reconfigure.server import Server

import tf
from tf import TransformListener
from tf.transformations import *
import numpy as np

class VisServoNode():
    def __init__(self) -> None:
        rospy.init_node('fellowme_visual_servoing')
        self.kf_tag_sub = rospy.Subscriber('kf/tag_detections/projected', PoseStamped, self.tag_cb)
        self.cmd_vel_pub = rospy.Publisher('/fellowme/mobile_base_controller/cmd_vel',Twist,queue_size=100)
        self.srv = Server(ParametersConfig, self.cfg_callback)
        self.target_angle = 0.
        self.eps = rospy.get_param('/fellowme_visual_servoing/angle_eps',3) # deg
        self.thresh = rospy.get_param('/fellowme_visual_servoing/angle_thresh',15) # deg  
        self.fix_orientation = False
        
    def cfg_callback(self, config, level):

            rospy.loginfo("""Reconfigure Request: {k}""".format(**config))
            self.k = config.k

            return config
        
    def tag_cb(self,msg:PoseStamped):
        vel = Twist()
        tag_in_base_projected = msg
        theta_to_tag = np.arctan2(tag_in_base_projected.pose.position.y,
                            tag_in_base_projected.pose.position.x)
        theta_to_tag_deg = np.rad2deg(theta_to_tag)
        if not self.fix_orientation:
            if np.abs(theta_to_tag_deg)>=self.thresh:
                rospy.loginfo(f"angle to tag>{self.thresh}[deg] --> start fixing rotation to tag!")
                self.fix_orientation = True
            else:
                pass
        else:
            if np.abs(theta_to_tag_deg)>=self.eps:
                vel.angular.z = self.k * theta_to_tag
            else:
                rospy.loginfo(f"angle to tag<{self.eps}[deg] --> stop fixing rotation to tag!")
                self.fix_orientation = False

        self.cmd_vel_pub.publish(vel)


    
        
# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
       # Initializes a rospy node to let the SimpleActionClient publish and subscribe
        vs = VisServoNode()
        rospy.spin()



    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")

