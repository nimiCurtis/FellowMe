#!/usr/bin/env python3

import rospy

# Brings in the SimpleActionClient
import actionlib
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from apriltag_ros.msg import AprilTagDetectionArray

class NavigationManager():
    
    def __init__(self):
        self.apritag_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray,self.tag_to_path)
        


