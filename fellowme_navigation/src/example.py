#!/usr/bin/env python

# Change yaml file positions and fine tune
# Problem sometimes leader move in reverse direction making it impossible to detect the arucomarker

import tf
import rospy
import time
import rospkg
import actionlib
import numpy as np

from tf.transformations import quaternion_slerp
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist, PoseStamped
from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray

rospack = rospkg.RosPack()


def translation_to_numpy(t):
    """To convert the translation list to numpy array"""

    return np.array([t[0], t[1], t[2]])

def quaternion_to_numpy(q):
    """To convert the quaternion list to numpy array"""

    return np.array([q[0], q[1], q[2], q[3]])


def movebase_client(coordinates):
    """This function is responsible to use the move_base action client to use with navigation stack to control the robot in the environment.
        It has an inbuilt SLAM implementation to avoid the obstacles. Also, this function constantly monitors the aruco marker transformations
        to get the desired goal coordinates.
    """

    client = actionlib.SimpleActionClient('/follower/move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = float(coordinates[0])
    goal.target_pose.pose.position.y = float(coordinates[1])
    goal.target_pose.pose.orientation.w = float(coordinates[2])
    client.send_goal(goal)

    while(1):
        wait = client.wait_for_result(rospy.Duration(0.2))

         # Check if the robot has reached the goal
        if wait:
            break
                
        try:
            (trans,rot) = listener.lookupTransform('/map', '/follower_tf/aruco_marker_frame', rospy.Time(0))
            # print("Goal", [trans[0]- tolerance, trans[1] - tolerance, rot[3]])
            
            alpha = 0.9
            rotation0 = quaternion_to_numpy(rot)
            rotation = quaternion_to_numpy(rot)
            rotation_interpolated = quaternion_slerp(rotation0, rotation, 1 - alpha)

            translation0 = translation_to_numpy(trans)
            translation = translation_to_numpy(trans)
            translation = alpha * translation0 + (1 - alpha) * translation

            latest_pos = [translation[0] - tolerance, translation[1] - tolerance, rotation_interpolated[3]]

            client.cancel_goal()

            if not last_print == "Marker Found \nFollowing the marker":
                rospy.loginfo("Marker Found \nFollowing the marker")
                last_print = "Marker Found \nFollowing the marker"
            else:
                last_print = "Marker Found \nFollowing the marker"
                
            result = movebase_client(latest_pos)
            return None
        
        except:
            # Check if the robot is already at the goal
            (trans1,rot1) = listener.lookupTransform('/map', '/follower_tf/camera_rgb_optical_frame', rospy.Time(0))
            if (abs(trans1[0] - coordinates[0]) < 0.10) or (abs(trans1[1] - coordinates[1]) < 0.10):
                # print("already at location______________________")

                current_goal = rospy.get_param("current_goal")
                if current_goal[0] == "bedroom":
                    if (abs(trans1[0] - (current_goal[1][0]+ 0.5)) < 0.15) and (abs(trans1[1] - (current_goal[1][1] + 0.5)) < 0.15):
                        exec_flag = True
                return None

    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()


if __name__ == '__main__':  
    
    # Iniliaztion of the node
    rospy.init_node('move_follower', anonymous=True)
   
    #Creating a transform listener
    listener = tf.TransformListener()

    # Twist type message
    velocity_msg = Twist()
    
    # Set up a publisher to the /cmd_vel topic
    pub = rospy.Publisher("/follower/cmd_vel", Twist, queue_size=1)
    
    # publish the velocity at 10 Hz (10 times per second)
    rate = rospy.Rate(10.0)

    # To allow the initial transformations to arrive, else it will go in except block for first few times
    time.sleep(1.5) 

    # intitial param
    rospy.set_param('current_goal', ["livingroom", [3.7847505, -1.2305222, 0.0]])
    
    # Setting global variables
    global latest_pos
    latest_pos = []
    global tolerance
    tolerance = 0.2
    global exec_flag
    exec_flag = False
    global last_print
    last_print = ""

    while not rospy.is_shutdown():    
        try:
            (trans,rot) = listener.lookupTransform('/map', '/follower_tf/aruco_marker_frame', rospy.Time(0))
            
            if not last_print == "Following the marker":
                rospy.loginfo("Following the marker")
                last_print = "Following the marker"
            else:
                last_print = "Following the marker"

            # print("Goal", [trans[0] - tolerance, trans[1] - tolerance, rot[3]])
            alpha = 0.9
            rotation0 = quaternion_to_numpy(rot)
            rotation = quaternion_to_numpy(rot)
            rotation_interpolated = quaternion_slerp(rotation0, rotation, 1 - alpha)

            translation0 = translation_to_numpy(trans)
            translation = translation_to_numpy(trans)
            translation = alpha * translation0 + (1 - alpha) * translation

            latest_pos = [translation[0] - tolerance, translation[1] - tolerance, rotation_interpolated[3]]
            result = movebase_client(latest_pos)
            # if result:
                # rospy.loginfo("Waypoint reached!")
            
        # It goes in except block if it doesn't find any transformations
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):

            if exec_flag:
                break
            
            if not last_print == "Looking for marker":
                rospy.loginfo("Looking for marker")
                last_print = "Looking for marker"
            else:
                last_print = "Looking for marker"
            
            # In place rotation to find the marker 
            # print("Rotating")
            for i in range(15):
                velocity_msg.angular.z = -3.0
                pub.publish(velocity_msg)
                rate.sleep()
                try:
                    (trans,rot) = listener.lookupTransform('/map', '/follower_tf/aruco_marker_frame', rospy.Time(0))
                    
                    if not last_print == "Marker Found \nFollowing the marker":
                        rospy.loginfo("Marker Found \nFollowing the marker")
                        last_print = "Marker Found \nFollowing the marker"
                    else:
                        last_print = "Marker Found \nFollowing the marker"
                    
                    alpha = 0.9
                    rotation0 = quaternion_to_numpy(rot)
                    rotation = quaternion_to_numpy(rot)
                    rotation_interpolated = quaternion_slerp(rotation0, rotation, 1 - alpha)

                    translation0 = translation_to_numpy(trans)
                    translation = translation_to_numpy(trans)
                    translation = alpha * translation0 + (1 - alpha) * translation

                    latest_pos = [translation[0] - tolerance, translation[1] - tolerance, rotation_interpolated[3]]
                    break
                except:
                    continue
            else:
                # It goes in this else block if it doesn't find the marker even after rotating. 
                # It gets the parameter value to find where leader is headed and makes the follower go to that location to find the marker.
                
                current_goal = rospy.get_param("current_goal")
                # print("current goal", current_goal)
                time.sleep(0.5)
                result = movebase_client([current_goal[1][0] + 0.5, current_goal[1][1] + 0.5, current_goal[1][2]])
                
                if result:
                    if current_goal[0] == "bedroom":
                        print(current_goal[1][0]+ 0.5, current_goal[1][1]+ 0.5)
                        rospy.loginfo("Follower Execution Done")
                        break

        rate.sleep()
