#!/usr/bin/env python3

import rospy

# Brings in the SimpleActionClient
import actionlib
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from apriltag_ros.msg import AprilTagDetectionArray
import tf
from tf import TransformListener
from tf.transformations import *
import numpy as np

class NavNode():
    def __init__(self) -> None:
        rospy.init_node('fellowme_navigation_manager')
        self.apritag_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray,self.tag_cb)
        self.odom_sub = rospy.Subscriber('/fellowme/mobile_base_controller/odom', Odometry,self.odom_cb)
        self.move_base_seq_pub = rospy.Publisher('fellowme/fellowme_navigation/move_base_sequence/add_goal_pose',PoseStamped,queue_size=1000)
        self.tl = TransformListener()
        # self.tl.canTransform()
        self.robot_pose = Pose()
        self.last_goal_pose = PoseStamped()
        self.tag_in_base = PoseStamped()
        self.odom_recieved = False
        self.tag_recieved = False
        
    def odom_cb(self,msg:Odometry):
        self.odom_recieved = True
        self.robot_pose = msg.pose
    
    def tag_cb(self,msg):
        if len(msg.detections)>0:
            self.tag_recieved = True
            tag_pose = PoseStamped()
            tag_pose.header = msg.detections[-1].pose.header
            tag_pose.pose = msg.detections[-1].pose.pose.pose
            self.tag_in_base = self.tl.transformPose(target_frame='/base_footprint',ps=tag_pose)
        else:
            self.tag_recieved = False


    def send_goal(self):
        now = rospy.Time.now()
        if self.tag_recieved and self.odom_recieved:
            # if self.tl.canTransform('/odom','/tag_13',now):
                theta_tag_in_robot = np.arctan2(self.tag_in_base.pose.position.x,
                                                self.tag_in_base.pose.position.y)
                
                # print(f"angle tag to robot : {np.rad2deg(theta_tag_in_robot)}")
                euler_odom_robot = euler_from_quaternion([self.robot_pose.pose.orientation.x,
                                                        self.robot_pose.pose.orientation.y,
                                                        self.robot_pose.pose.orientation.z,
                                                        self.robot_pose.pose.orientation.w])
                
                # print(f"orientation odom robot in euler : {np.rad2deg(euler_odom_robot)}")
                
                goal_orientation_euler = [euler_odom_robot[0] - ((np.pi/2)-theta_tag_in_robot),
                                        euler_odom_robot[1] - ((np.pi/2)-theta_tag_in_robot),
                                        euler_odom_robot[2]]
                
                # print(f"orientation odom goal in euler : {np.rad2deg(goal_orientation_euler)}")
                goal_orientation_quat = quaternion_from_euler(goal_orientation_euler[0],
                                                            goal_orientation_euler[1],
                                                            goal_orientation_euler[2])
                # print(f"quat odom goal {goal_orientation_quat}")
                tag_in_odom = self.tl.transformPose(target_frame='/odom',ps=self.tag_in_base)
                goal_pose = PoseStamped()
                goal_pose.pose.position.x = tag_in_odom.pose.position.x
                goal_pose.pose.position.y = tag_in_odom.pose.position.y
                goal_pose.pose.position.z = 0
                goal_pose.pose.orientation.x = 0
                goal_pose.pose.orientation.y = 0
                goal_pose.pose.orientation.z = goal_orientation_quat[2]
                goal_pose.pose.orientation.w = goal_orientation_quat[3]

                if self.distance_thresh(self.robot_pose,goal_pose,0.3+0.17):
                    print("robot dist requirment!")
                    if self.distance_thresh(self.last_goal_pose,goal_pose,0.1):
                        print("goal dist requirment")
                        self.move_base_seq_pub.publish(goal_pose)
                        self.last_goal_pose = goal_pose
            
            # else:
            #     print("cannot transform")
        else:
            pass
    
    def distance_thresh(self,pose1,pose2,thresh):
        pose1_xy_pos = np.array([pose1.pose.position.x,pose1.pose.position.y])
        pose2_xy_pos = np.array([pose2.pose.position.x,pose2.pose.position.y])
        d = np.linalg.norm(pose1_xy_pos-pose2_xy_pos)
        return d >= thresh 
        
# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
       # Initializes a rospy node to let the SimpleActionClient publish and subscribe
        nav = NavNode()
        
        rate = rospy.Rate(3)
        while not rospy.is_shutdown():
            nav.send_goal()
            rate.sleep()


    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
