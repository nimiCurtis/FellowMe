#!/usr/bin/env python3

import rospy
import smach
from smach import CBState
import smach_ros
import smach_msgs

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, Twist
import numpy as np
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from apriltag_ros.msg import AprilTagDetectionArray
from fellowme_navigation.cfg import ParametersConfig
from utils.smach import Servo
from dynamic_reconfigure.server import Server
import tf2_ros 
import tf2_geometry_msgs

class NavManager():
    def __init__(self) -> None:
        

        self.eps = rospy.get_param('/fellowme_visual_servoing/angle_eps',3) # deg
        self.thresh = rospy.get_param('/fellowme_visual_servoing/angle_thresh',10) # deg  
        
        self.odom_sub = rospy.Subscriber('/fellowme/mobile_base_controller/odom', Odometry, self.odom_cb)
        self.kf_tag_sub = rospy.Subscriber('kf/tag_detections/projected', PoseStamped, self.tag_cb)
        self.cmd_vel_pub = rospy.Publisher('/fellowme/mobile_base_controller/cmd_vel',Twist,queue_size=100)
        
        self.srv = Server(ParametersConfig, self.cfg_callback)
        
        self.tag_in_base_projected = PoseStamped()
        self.robot_pose = Pose()
        
        self.tag_received = False
        self.odom_received = False

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0))  # tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        self.d = 0.8

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdown_hook)
    

        # Initialize action client for move_base
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        wait_server = self.client.wait_for_server(rospy.Duration(5.0))
        if not wait_server:
            rospy.logerr("Action server not available!")
            return

        rospy.sleep(rospy.Duration(1))
    
    
    
    def shutdown_hook(self):
        """Callback function on node shutdown."""
        
        self.ctrl_c = True
        rospy.logerr_once("Node shutting down")
        # Shutdown hook
        rospy.signal_shutdown("Node shutting down")
    
    def cfg_callback(self, config, level):

            rospy.loginfo("""Reconfigure Request: servo coeff = {k}""".format(**config))
            self.k = config.k

            return config
        
    def get_tag_pose(self):
        return self.tag_in_base_projected
    
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
        
        return tag_transformed
    
    def tag_cb(self,msg:PoseStamped):
        if msg is not None:
            self.tag_in_base_projected = msg
            self.tag_received = True
        else:
            self.tag_received = False
    
    def odom_cb(self, msg: Odometry):
        """Callback function for odometry data."""
        if msg is not None:
            self.odom_received = True
            self.robot_pose = msg.pose
        else:
            self.odom_received = False
    
    def calc_poses_distance(self, pose1, pose2):
        """Calculate the distance between two poses and compare it to a threshold."""
        pose1_xy_pos = np.array([pose1.pose.position.x, pose1.pose.position.y])
        pose2_xy_pos = np.array([pose2.pose.position.x, pose2.pose.position.y])
        d = np.linalg.norm(pose1_xy_pos - pose2_xy_pos)
        return d 
    
    def distance_to_tag(self, tag_pose):
        xy_pos = np.array([tag_pose.pose.position.x, tag_pose.pose.position.y])
        d = np.linalg.norm(xy_pos)
        return d
    
    def is_subscribers_receiving(self):
        """Check if both tag and odometry data are received."""
        return self.tag_received and self.odom_received



# define state follow
class Follow(smach.State):
    def __init__(self,nav_manager:NavManager):
        smach.State.__init__(self, outcomes=['SUCCEEDED','ABORTED','DONE'])
        self.nav_manager = nav_manager
        self.exec_flag = False

    def execute(self, userdata):
        rospy.loginfo('Executing state FOLLOW')
        # if not self.ctrl_c:
        while (self.nav_manager.is_subscribers_receiving()) and not self.nav_manager.ctrl_c:
            tag_in_base_projected = self.nav_manager.tag_in_base_projected
            target_pose = self.nav_manager.get_tag_transformed(tag_pose=tag_in_base_projected,
                                                                    target_frame='map')
            if self.nav_manager.distance_to_tag(tag_in_base_projected)>self.nav_manager.d:
                self.movebase_client(target_pose)
            else:
                #self.nav_manager.client.cancel_goal()
                self.nav_manager.client.cancel_all_goals()
                return 'ABORTED'
        
        #self.nav_manager.client.cancel_goal()
        # self.nav_manager.client.cancel_all_goals()
        return 'ABORTED'

    def movebase_client(self, target_pose: PoseStamped):
        """
        Use the move_base action client to navigate the robot using the navigation stack.
        """
        goal = MoveBaseGoal()
        goal.target_pose = target_pose
        self.nav_manager.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)

        while not self.nav_manager.ctrl_c:
            wait = self.nav_manager.client.wait_for_result(rospy.Duration(0.1))

            if wait:
                break
            try:
                if self.nav_manager.is_subscribers_receiving():
                    tag_in_base_projected = self.nav_manager.tag_in_base_projected
                    goal_pose = self.nav_manager.get_tag_transformed(tag_pose=tag_in_base_projected,
                                                                    target_frame='map')

                    if (self.nav_manager.distance_to_tag(tag_in_base_projected)>self.nav_manager.d):
                            self.nav_manager.client.cancel_goal()
                            # self.nav_manager.client.cancel_all_goals()
                            self.exec_flag = False
                            result = self.movebase_client(goal_pose)
                            return None
                    else:
                        self.exec_flag = True
                        self.nav_manager.client.cancel_goal()
                        # self.nav_manager.client.cancel_all_goals()
                        return None
            except:
                pass

        if not wait:
            rospy.logerr("Action server not available!")
            self.nav_manager.client.cancel_all_goals()

            rospy.signal_shutdown("Action server not available!")
        else:
            return self.nav_manager.client.get_result()
    
    
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

# define state servo
class Servo(smach.State):
    def __init__(self, nav_manager:NavManager):
        smach.State.__init__(self, outcomes=['SUCCEEDED','ABORTED','DONE'])
        self.nav_manager = nav_manager
        self.fix_orientation = False


    def execute(self, userdata):
        rospy.loginfo('Executing state SERVO')
        
        while (self.nav_manager.is_subscribers_receiving()) and not self.nav_manager.ctrl_c:
            tag_in_base_projected = self.nav_manager.tag_in_base_projected
            if (self.nav_manager.distance_to_tag(tag_in_base_projected)<=self.nav_manager.d):
                vel = Twist()
                theta_to_tag = np.arctan2(tag_in_base_projected.pose.position.y,
                                    tag_in_base_projected.pose.position.x)
                theta_to_tag_deg = np.rad2deg(theta_to_tag)
                if not self.fix_orientation:
                    if np.abs(theta_to_tag_deg)>=self.nav_manager.thresh:
                        rospy.loginfo(f"angle to tag>{self.nav_manager.thresh}[deg] --> start fixing rotation to tag!")
                        self.fix_orientation = True
                    else:
                        pass
                else:
                    if np.abs(theta_to_tag_deg)>=self.nav_manager.eps:
                        vel.angular.z = self.nav_manager.k * theta_to_tag
                    else:
                        rospy.loginfo(f"angle to tag<{self.nav_manager.eps}[deg] --> stop fixing rotation to tag!")
                        self.fix_orientation = False
                        #return 'SUCCEEDED'
                    
                self.nav_manager.cmd_vel_pub.publish(vel)
            else:
                return 'ABORTED'
            
        return 'ABORTED'



class NavSmNode():
    def __init__(self) -> None:
        rospy.init_node('fellowme_navigation_manager_node')
        self.nav_manager = NavManager()
        rospy.on_shutdown(self.nav_manager.shutdown_hook)
    
    def navigate(self):
        while not self.nav_manager.ctrl_c:
            sm = smach.StateMachine(outcomes=['DONE'])
            with sm:
                smach.StateMachine.add('SERVO', Servo(nav_manager=self.nav_manager),
                                    transitions={'SUCCEEDED':'DONE',
                                                    'ABORTED':'FOLLOW'})
        
                
                smach.StateMachine.add('FOLLOW', Follow(nav_manager=self.nav_manager),
                                    transitions={'SUCCEEDED':'DONE',
                                                    'ABORTED':'SERVO'})


            # Create and start the introspection server
            sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
            sis.start()

            # Execute the state machine
            outcome = sm.execute()

            # Wait for ctrl-c to stop the application
            rospy.spin()
            sis.stop()

def main():
    nav_manager_node = NavSmNode()
    nav_manager_node.navigate()

if __name__ == '__main__':
    main()