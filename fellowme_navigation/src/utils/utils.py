import numpy as np

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from apriltag_ros.msg import AprilTagDetectionArray
import tf
from tf import TransformListener
from tf.transformations import *

def numpyState_to_pose(numpy_state):
    """
    Convert numpy state to a PoseStamped message.
    """
    pose = PoseStamped()
    
    # Set position
    pose.pose.position.x = numpy_state[0]
    pose.pose.position.y = numpy_state[1]
    
    # Set orientation
    q = quaternion_from_euler(0, 0, normalize_angle(numpy_state[2]))
    pose.pose.orientation.x = 0
    pose.pose.orientation.y = 0
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]

    return pose

def pose_to_numpyStateWithCov(pose: PoseWithCovarianceStamped, vel_state=None, vel_cov=None):
    """
    Convert PoseWithCovarianceStamped message to numpy state with covariance.
    """
    numpy_state = pose_to_numpyState(pose.pose.pose, vel_state)
    
    if vel_cov is not None:
        numpy_cov = np.zeros((12, 12))
        numpy_cov[:6, :6] = np.array(pose.pose.covariance).reshape(6, 6)
        numpy_cov[6:, 6:] = np.array(vel_cov).reshape(6, 6)
    else:
        numpy_cov = np.array(pose.pose.covariance).reshape(6, 6)
    
    return numpy_state, numpy_cov

def pose_to_numpyState(pose, vel_state=None):
    """
    Convert Pose message to numpy state.
    """
    if isinstance(pose, Pose):
        x = pose.position.x
        y = pose.position.y

        qx = pose.orientation.x
        qy = pose.orientation.y
        qz = pose.orientation.z
        qw = pose.orientation.w

        euler_angles = euler_from_quaternion([qx, qy, qz, qw])
        theta = normalize_angle(euler_angles[2])

        numpy_state = np.vstack((x, y, theta))
        if vel_state is not None:
            numpy_state = np.vstack((numpy_state, vel_state))

        return numpy_state

def normalize_angle(angle):
    """
    Normalize an angle to be within -pi to pi range.
    """
    if -np.pi < angle <= np.pi:
        return angle
    if angle > np.pi:
        angle = angle - 2 * np.pi
    if angle <= -np.pi:
        angle = angle + 2 * np.pi
    return normalize_angle(angle)

def normalize_angles_array(angles):
    """
    Normalize an array of angles to be within -pi to pi range.
    """
    z = np.zeros_like(angles)
    for i in range(angles.shape[0]):
        z[i] = normalize_angle(angles[i])
    return z
