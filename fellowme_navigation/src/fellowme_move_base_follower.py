#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
import tf
from tf import TransformListener
from tf.transformations import *
import numpy as np
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class NavNode():
    def __init__(self) -> None:
        """
        Initializes the navigation node.
        Subscribes to tag and odometry topics, sets up action client for move_base.
        """
        rospy.init_node('fellowme_move_base_follower')
        
        # Subscribers for tag and odometry data
        self.kf_tag_sub = rospy.Subscriber('kf/tag_detections/projected', PoseStamped, self.tag_cb)
        self.odom_sub = rospy.Subscriber('/fellowme/mobile_base_controller/odom', Odometry, self.odom_cb)
        
        # Initialize action client for move_base
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        wait_server = self.client.wait_for_server(rospy.Duration(5.0))
        if not wait_server:
            rospy.logerr("Action server not available!")
            return
        
        # Transform listener and pose variables
        self.tl = TransformListener()
        self.robot_pose = Pose()
        self.last_goal_pose = PoseStamped()
        self.tag_in_base = PoseStamped()
        self.odom_received = False
        self.tag_received = False
        self.exec_flag = False
        self.ctrl_c = False
        
        # Shutdown hook
        rospy.on_shutdown(self.shutdown_hook)
        self.last_log = "Waiting for goals"
        rospy.loginfo(self.last_log)
    
    def shutdown_hook(self):
        """Callback function on node shutdown."""
        rospy.loginfo("Node shutting down")
        self.ctrl_c = True
    
    def odom_cb(self, msg: Odometry):
        """Callback function for odometry data."""
        self.odom_received = True
        self.robot_pose = msg.pose
    
    def tag_cb(self, msg: PoseWithCovarianceStamped):
        """Callback function for tag detection data."""
        if msg is not None:
            self.tag_in_base = msg
            self.tag_received = True

    def distance_thresh(self, pose1, pose2, thresh):
        """Calculate the distance between two poses and compare it to a threshold."""
        pose1_xy_pos = np.array([pose1.pose.position.x, pose1.pose.position.y])
        pose2_xy_pos = np.array([pose2.pose.position.x, pose2.pose.position.y])
        d = np.linalg.norm(pose1_xy_pos - pose2_xy_pos)
        return d >= thresh 

    def movebase_client(self, target_pose: PoseStamped):
        """
        Use the move_base action client to navigate the robot using the navigation stack.
        """
        goal = MoveBaseGoal()
        goal.target_pose = target_pose
        self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)

        while not self.ctrl_c:
            wait = self.client.wait_for_result(rospy.Duration(0.2))

            if wait:
                break
            try:
                if self.is_subscribers_receiving():
                    goal_pose = self.get_target_pose_from_tag()
                    if self.distance_thresh(self.robot_pose, goal_pose, 0.3 + 0.17):
                        if self.distance_thresh(self.last_goal_pose, goal_pose, 0.1):
                            self.last_goal_pose = goal_pose
                            self.client.cancel_goal()
                            self.exec_flag = False
                            result = self.movebase_client(self.last_goal_pose)
                            return None
                    else:
                        self.exec_flag = True
                        self.client.cancel_goal()
                        return None
            except:
                pass

        if not wait:
            rospy.logerr("Action server not available!")
            self.client.cancel_goal()
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.client.get_result()
        
    def is_subscribers_receiving(self):
        """Check if both tag and odometry data are received."""
        return self.tag_received and self.odom_received
        
    def navigate(self):
        """Navigate the robot based on received data."""
        if not self.ctrl_c:
            if self.is_subscribers_receiving():
                target_pose = self.get_target_pose_from_tag()
                if self.distance_thresh(self.robot_pose, target_pose, 0.3 + 0.17):
                    if self.distance_thresh(self.last_goal_pose, target_pose, 0.1): 
                        self.movebase_client(target_pose)
            else:
                self.client.cancel_goal()
                rospy.loginfo(self.last_log)
        
    def get_target_pose_from_tag(self):
        """Transform tag position from base frame to map frame."""
        target_pose_in_base = self.tag_in_base
        target_pose_in_map = self.tl.transformPose(target_frame='map', ps=target_pose_in_base)
        return target_pose_in_map

    def done_cb(self, status, result):
        """Callback function on goal completion or termination."""
        if status == 2:
            rospy.loginfo("Goal pose received a cancel request after it started executing, successfully cancelled!")
        elif status == 8:
            rospy.loginfo("Goal pose received a cancel request before it started executing, successfully cancelled!")
        elif status == 3:
            rospy.loginfo("Goal pose REACHED!") 
        elif status == 4:
            rospy.logerr("Goal pose aborted, stopping sequence execution, check for any errors!")
        elif status == 5:
            rospy.logerr("Goal pose has been rejected by the Action Server. Moving to the next goal.")

    def active_cb(self):
        """Callback function when goal becomes active."""
        rospy.loginfo("Following target pose!")

    def feedback_cb(self, feedback):
        """Callback function for receiving feedback during goal execution."""
        pass

if __name__ == '__main__':
    try:
        # Initialize the navigation node
        nav = NavNode()
        
        # Control loop rate
        rate = rospy.Rate(15)
        while not nav.ctrl_c:
            try:
                # Navigate the robot based on received data
                nav.navigate()
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                if nav.exec_flag:
                    continue
    
            rate.sleep()

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
