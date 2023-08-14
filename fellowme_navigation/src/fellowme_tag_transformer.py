#!/usr/bin/env python3

import numpy as np
from utils.kalmanfilter import KalmanFilter

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from apriltag_ros.msg import AprilTagDetectionArray
import tf
from tf import TransformListener
from tf.transformations import *

class TagTransformerNode:
    def __init__(self):
        """
        Initializes the TagTransformerNode.
        """
        rospy.init_node('fellowme_tag_transformer')
        
        # Publishers for projected tag pose and path
        self.path_pub = rospy.Publisher('/tag_detections/projected/path', Path, latch=True, queue_size=1000)
        self.tag_pose_projected_pub = rospy.Publisher('/tag_detections/projected', PoseStamped, queue_size=1000)
        
        # Subscriber for raw AprilTag detections
        self.apriltag_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tag_callback)
        
        # Path and TransformListener initialization
        self.path = Path()
        self.tl = TransformListener()

    def get_tag_transformed(self, tag_pose, target_frame):
        """
        Transforms tag pose to the specified target frame.
        """
        if self.tl.canTransform(target_frame, 'camera_color_optical_frame', rospy.Time()):
            tag_in_odom = self.tl.transformPose(target_frame=target_frame, ps=tag_pose)
            return tag_in_odom
        else:
            rospy.logwarn("AR tag transform error")
            return None
        
    def tag_received(self, detections_list):
        """
        Checks if AprilTag detections are received.
        """
        return len(detections_list) > 0

    def tag_callback(self, msg: AprilTagDetectionArray):
        """
        Callback function for AprilTag detections.
        """
        tag_pose = PoseStamped()
        if self.tag_received(msg.detections):
            tag_pose.header = msg.detections[-1].pose.header
            tag_pose.pose = msg.detections[-1].pose.pose.pose

            tag_in_base = self.get_tag_transformed(tag_pose=tag_pose, target_frame='base_link')
            
            # Projected tag (set z position and orientation x, y to zero)
            tag_in_base.pose.position.z = 0
            tag_in_base.pose.orientation.x = 0
            tag_in_base.pose.orientation.y = 0
            
            tag_in_map = self.get_tag_transformed(tag_pose=tag_pose, target_frame='map')

            if tag_in_map is not None:
                self.path.header = tag_in_map.header
                self.path.poses.append(tag_in_map)
                self.path_pub.publish(self.path)

            if tag_in_base is not None:
                self.tag_pose_projected_pub.publish(tag_in_base)

if __name__ == '__main__':
    try:
        transform_tag = TagTransformerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Tag transformation finished.")
