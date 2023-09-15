#!/usr/bin/env python3

import numpy as np
from utils.kalmanfilter import KalmanFilter

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from apriltag_ros.msg import AprilTagDetectionArray
import tf
# from tf import TransformListener
# from tf.transformations import *
import tf2_ros 
# from tf2_ros import TransformListener
import tf2_geometry_msgs
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
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0))  # tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        


    def get_tag_transformed(self, tag_pose, target_frame, time=rospy.Duration(0.0)):
        """
        Transforms tag pose to the specified target frame.
        """
        transform = self.tf_buffer.lookup_transform(target_frame,
                                       # source frame:
                                       tag_pose.header.frame_id,
                                       # get the tf at the time the pose was valid
                                       tag_pose.header.stamp,
                                       # wait for at most 1 second for transform, otherwise throw
                                       rospy.Duration(1.0))

        tag_transformed = tf2_geometry_msgs.do_transform_pose(tag_pose, transform)
        
        # if self.tl.can_transform(target_frame, 'camera_optical_frame',time=rospy.Time.now(),timeout=rospy.Duration(0.01)):
        #     tag_trasnformed = self.tl.transform(target_frame=target_frame, object_stamped=tag_pose)
        #     return tag_trasnformed
        # else:
        #     print(target_frame)
        #     rospy.logwarn("AR tag transform error")
        #     return None
        return tag_transformed
        
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
            # tag_pose.header.frame_id = msg.detections[-1].pose.header.frame_id
            # tag_pose.header.stamp = rospy.Time.now()
            tag_pose.pose = msg.detections[-1].pose.pose.pose

            tag_in_base = self.get_tag_transformed(tag_pose=tag_pose, target_frame='base_link')
            tag_in_map = self.get_tag_transformed(tag_pose=tag_pose, target_frame='map')

            # Projected tag (set z position and orientation x, y to zero)
            tag_in_base.pose.position.z = 0
            tag_in_base.pose.orientation.x = 0
            tag_in_base.pose.orientation.y = 0
            

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
