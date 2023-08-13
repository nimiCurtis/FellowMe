import numpy as np

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from apriltag_ros.msg import AprilTagDetectionArray
import tf
from tf import TransformListener
from tf.transformations import *

def numpyStateWithCov_to_pose(numpy_state,numpy_cov):
    pose = PoseWithCovarianceStamped()
    # pose
    # position
    pose.pose.pose.position.x = numpy_state[0]
    pose.pose.pose.position.y = numpy_state[1]
    pose.pose.pose.position.z = numpy_state[2]

    # orientation
    euler_angels = numpy_state[3:6].flatten().tolist()
    q = quaternion_from_euler(euler_angels[0],euler_angels[1],euler_angels[2])
    pose.pose.pose.orientation.x = q[0]
    pose.pose.pose.orientation.y = q[1]
    pose.pose.pose.orientation.z = q[2]
    pose.pose.pose.orientation.w = q[3]

    # covariance
    pose.pose.covariance = numpy_cov.reshape(1,-1)[0].tolist()[:36]
    
    return pose


def pose_to_numpyStateWithCov(pose:PoseWithCovarianceStamped,vel_state=None,vel_cov=None):
    numpy_state = pose_to_numpyState(pose.pose.pose,vel_state)
    
    if vel_cov is not None:
        numpy_cov = np.zeros((12,12))
        numpy_cov[:6,:6] = np.array(pose.pose.covariance).reshape(6,6)
        numpy_cov[6:,6:] = np.array(vel_cov).reshape(6,6)
    else:
        numpy_cov = np.array(pose.pose.covariance).reshape(6,6)
    
    return numpy_state, numpy_cov

def pose_to_numpyState(pose,vel_state=None):
    if isinstance(pose,Pose):
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z
        qx = pose.orientation.x
        qy = pose.orientation.y
        qz = pose.orientation.z
        qw = pose.orientation.w

        euler_angles = euler_from_quaternion([qx,qy,qz,qw])
        
        vx = euler_angles[0]
        vy = euler_angles[1]
        vz = euler_angles[2]
        
        numpy_state = np.vstack((x,y,z,vx,vy,vz))
        if vel_state is not None:
            numpy_state = np.vstack((numpy_state,vel_state))
    
        return numpy_state