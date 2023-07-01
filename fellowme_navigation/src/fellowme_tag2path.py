#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from apriltag_ros.msg import AprilTagDetectionArray
import tf
from tf import TransformListener

class Tag2Path:
    def __init__(self):
        self.path_pub = rospy.Publisher('/tag_detections_trajectory', Path, latch=True, queue_size=10)
        self.apritag_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray,self.tag_to_path)
        self.path = Path()
        self.tl = TransformListener()


    def tag_to_path(self, msg:AprilTagDetectionArray):
        if len(msg.detections)>0:
            tag_pose = PoseStamped()
            tag_pose.header = msg.detections[-1].pose.header
            tag_pose.pose = msg.detections[-1].pose.pose.pose
            tag_in_base_link = self.tl.transformPose(target_frame='base_link',ps=tag_pose)
            
            self.path.header = tag_in_base_link.header
            self.path.poses.append(tag_in_base_link)
            self.path_pub.publish(self.path)


if __name__ == '__main__':
    rospy.init_node('tag_to_path_node')
    tag_to_path = Tag2Path()
    rospy.spin()