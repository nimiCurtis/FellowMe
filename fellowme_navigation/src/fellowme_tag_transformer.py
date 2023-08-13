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
        self.path_pub = rospy.Publisher('/tag_detections_trajectory', Path, latch=True, queue_size=1000)
        self.tag_base_pose_pub = rospy.Publisher('/tag_detections_in_base', PoseWithCovarianceStamped, queue_size=1000)
        self.apritag_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray,self.tag_callback)
        self.path = Path()
        self.tl = TransformListener()
        self.initial_covariance = rospy.get_param('/apriltag_ros_continuous_node/initial_estimate_pose_covariance')

    def get_tag_transformed(self,tag_pose,target_frame):
        if self.tl.canTransform(target_frame,'camera_color_optical_frame', rospy.Time()):
            tag_in_odom = self.tl.transformPose(target_frame=target_frame,ps=tag_pose)
            return tag_in_odom
        else:
            rospy.logwarn("AR tag transform error")
            return None
        
    def tag_recieved(self,detections_list):
        return len(detections_list)>0

    def tag_callback(self, msg:AprilTagDetectionArray):
            tag_pose = PoseStamped()
            tag_pose_in_base_with_cov = PoseWithCovarianceStamped()
            if self.tag_recieved(msg.detections):
                tag_pose.header = msg.detections[-1].pose.header
                tag_pose.pose = msg.detections[-1].pose.pose.pose
                
                tag_in_base = self.get_tag_transformed(tag_pose=tag_pose,target_frame='base_link')
                
                tag_in_odom = self.get_tag_transformed(tag_pose=tag_pose,target_frame='odom')

                if tag_in_odom is not None:
                    self.path.header = tag_in_odom.header
                    self.path.poses.append(tag_in_odom)
                    self.path_pub.publish(self.path)

                if tag_in_base is not None:
                    tag_pose_in_base_with_cov.header = tag_in_base.header
                    tag_pose_in_base_with_cov.pose.pose = tag_in_base.pose
                    tag_pose_in_base_with_cov.pose.covariance = self.initial_covariance
                    self.tag_base_pose_pub.publish(tag_pose_in_base_with_cov)


if __name__ == '__main__':
    rospy.init_node('fellowme_tag_transformer')
    tag_to_path = TagTransformerNode()
    rospy.spin()