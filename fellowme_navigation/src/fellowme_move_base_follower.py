#!/usr/bin/env python3

import rospy

# Brings in the SimpleActionClient
import actionlib
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseStamped,PoseWithCovarianceStamped, Quaternion
import tf
from tf import TransformListener
from tf.transformations import *
import numpy as np
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal



class NavNode():
    def __init__(self) -> None:
        rospy.init_node('fellowme_move_base_follower')
        self.apritag_sub = rospy.Subscriber('kf/tag_detections_in_base', PoseWithCovarianceStamped,self.tag_cb)
        self.odom_sub = rospy.Subscriber('/fellowme/mobile_base_controller/odom', Odometry,self.odom_cb)
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        wait_server = self.client.wait_for_server(rospy.Duration(5.0))
        if not wait_server:
            rospy.logerr("Action server not available!")
            #rospy.signal_shutdown("Action server not available!")
            return
        self.tl = TransformListener()
        # self.tl.canTransform()
        self.robot_pose = Pose()
        self.last_goal_pose = PoseStamped()
        self.tag_in_base = PoseWithCovarianceStamped()
        self.odom_recieved = False
        self.tag_recieved = False
        self.exec_flag = False
        ## shutdownhook process
        self.ctrl_c = False
        
        rospy.on_shutdown(self.shutdownhook)
        self.last_log = "Waiting for goals"
        rospy.loginfo(self.last_log)
    
    def shutdownhook(self):
        rospy.loginfo("node shutting down")
        self.ctrl_c = True
    
    def odom_cb(self,msg:Odometry):
        self.odom_recieved = True
        self.robot_pose = msg.pose
    
    def tag_cb(self,msg:PoseWithCovarianceStamped):
        if msg is not None:
            self.tag_in_base = msg
            self.tag_recieved = True
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
                    if self.distance_thresh(self.last_goal_pose,goal_pose,0.3):
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
    
    def movebase_client(self,target_pose:PoseStamped):
        """This function is responsible to use the move_base action client to use with navigation stack to control the robot in the environment.
            It has an inbuilt SLAM implementation to avoid the obstacles. Also, this function constantly monitors the aruco marker transformations
            to get the desired goal coordinates.
        """

        goal = MoveBaseGoal()
        goal.target_pose = target_pose
        self.client.send_goal(goal)

        while not self.ctrl_c:
            wait = self.client.wait_for_result(rospy.Duration(0.2))

            # Check if the robot has reached the goal
            if wait:
                break
                    
            try:
                
                if self.is_subscribers_recieving():
                    goal_pose = self.get_target_pose_from_tag()
                    if self.distance_thresh(self.robot_pose,goal_pose,0.3+0.17):
                        rospy.loginfo(f'fellowme is at least >> {0.3+0.17} from the current goal')
                        if self.distance_thresh(self.last_goal_pose,goal_pose,0.3):
                            self.last_goal_pose = goal_pose
                            self.client.cancel_goal()
                            self.exec_flag = False
                            if not self.last_log == f'last goal is at least >> {0.3} from the current goal >> following the tag':
                                self.last_log = f'last goal is at least >> {0.3} from the current goal >> following the tag'
                                rospy.loginfo(self.last_log)
                            else:
                                self.last_log = f'last goal is at least >> {0.3} from the current goal >> following the tag'
                    
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
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.client.get_result()
        
    def is_subscribers_recieving(self):
        return self.tag_recieved and self.odom_recieved
        
    def navigate(self):
        
            if self.is_subscribers_recieving():
                target_pose = self.get_target_pose_from_tag()
                self.movebase_client(target_pose)
            else:
                rospy.loginfo(self.last_log)

    def get_target_pose_from_tag(self):
        
        
        theta_tag_in_robot = np.arctan2(self.tag_in_base.pose.pose.position.x,
                                                    self.tag_in_base.pose.pose.position.y)
                    
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
        temp_pose = PoseStamped()
        temp_pose.header = self.tag_in_base.header
        temp_pose.pose = self.tag_in_base.pose.pose
        
        tag_in_map = self.tl.transformPose(target_frame='map',ps=temp_pose)
        
        goal_pose = PoseStamped()
        goal_pose.header = tag_in_map.header
        goal_pose.pose.position.x = tag_in_map.pose.position.x
        goal_pose.pose.position.y = tag_in_map.pose.position.y
        goal_pose.pose.position.z = 0
        goal_pose.pose.orientation.x = 0
        goal_pose.pose.orientation.y = 0
        goal_pose.pose.orientation.z = goal_orientation_quat[2]
        goal_pose.pose.orientation.w = goal_orientation_quat[3]
        
        return goal_pose
    
    
# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
       # Initializes a rospy node to let the SimpleActionClient publish and subscribe
        nav = NavNode()
        
        rate = rospy.Rate(10)
        while not nav.ctrl_c:
            try:
                nav.navigate()
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                if nav.exec_flag:
                    continue
    
            rate.sleep()
    


    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
