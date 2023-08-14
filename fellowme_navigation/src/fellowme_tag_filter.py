#!/usr/bin/env python3

import numpy as np
from utils.kalmanfilter import KalmanFilter
from utils.utils import pose_to_numpyStateWithCov, pose_to_numpyState, numpyState_to_pose

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from apriltag_ros.msg import AprilTagDetectionArray
import tf
from tf import TransformListener
from tf.transformations import *

class TagFilterNode:
    def __init__(self):
        """
        Initializes the TagFilterNode.
        """
        rospy.init_node('fellowme_tag_filter')
        
        # Publishers for filtered tag pose and path
        self.path_pub = rospy.Publisher('kf/tag_detections/projected/path', Path, latch=True, queue_size=1000)
        self.kf_tag_base_pose_pub = rospy.Publisher('kf/tag_detections/projected', PoseStamped, queue_size=1000)
        
        # Subscriber for the raw tag pose
        self.tag_in_base_sub = rospy.Subscriber('/tag_detections/projected', PoseStamped)
        
        # Path and TransformListener initialization
        self.path = Path()
        self.tl = TransformListener()
        
        # Parameters
        self.initial_covariance = rospy.get_param('/fellowme_tag_filter/initial_estimate_covariance')
        self.sensor_noise = rospy.get_param('/fellowme_tag_filter/observation_noise_covariance')
        self.sigma_n = rospy.get_param('/fellowme_tag_filter/sigma_n')

        # KalmanFilter and state variables
        self.kf = KalmanFilter(sensor_noise=self.sensor_noise)
        self.filter_started = False    
        self.cov_prev = None
        self.mu_prev = None
        self.tag_in_base = None
        
    def start_clock(self):
        """
        Starts the Kalman filter clock.
        """
        self.kf_last_time = rospy.Time().now()
        self.kf_current_time = rospy.Time().now()
        rospy.loginfo("Kalman filter started!")
        return 0.
    
    def start_drck(self):
        pass

    def filtering(self):
        """
        Filtering process for Kalman filtering of tag pose.
        """
        try:
            self.tag_in_base = rospy.wait_for_message(self.tag_in_base_sub.name, PoseStamped, timeout=1/30)
        except:
            self.tag_in_base = None

        if self.tag_in_base is None and not self.filter_started:
            pass
        
        if not self.filter_started:
            if self.tag_in_base is not None:
                dt = self.start_clock()
                self.mu_prev = pose_to_numpyState(self.tag_in_base.pose, vel_state=np.zeros((3,1)))
                self.cov_prev = np.array(self.initial_covariance).reshape((6,6))
                self.filter_started = True
            
        elif self.filter_started:
            dt = self.take_time_diff()
            mu_filtered, cov_filtered = self.filter(self.mu_prev, self.cov_prev, self.tag_in_base, dt)
            self.mu_prev, self.cov_prev = mu_filtered, cov_filtered
            
            pose_filtered = numpyState_to_pose(self.mu_prev)
             
            self.publish(pose_filtered_msg=pose_filtered)
            
    def take_time_diff(self):
        """
        Calculates the time difference between two consecutive filtering steps.
        """
        self.kf_current_time = rospy.Time.now()
        dt = (self.kf_current_time - self.kf_last_time).to_sec()
        self.kf_last_time = self.kf_current_time
        return dt

    def filter(self, mu_prev, cov_prev, tag_measurement: PoseStamped, dt):
        """
        Kalman filter step to predict and correct the tag pose estimate.
        """
        A = np.diag([1.]*6)
        dt_diag = np.diag([dt]*3)
        A[:3,3:] = dt_diag
        
        R = np.diag([1.]*6)
        R[:3,:3] *= dt
        R = (self.sigma_n**2) * R
        
        mu_pred, cov_pred = self.kf.predict(mu_prev=mu_prev, cov_prev=cov_prev, A_t=A, R_t=R)
        
        if tag_measurement is None:
            k_gain = np.zeros((6,3))
            measurement = np.zeros((3,1))
        else:
            k_gain = self.kf.get_kalman_gain(cov_t_pred=cov_pred)
            measurement = pose_to_numpyState(tag_measurement.pose)

        mu, cov = self.kf.correct(measurement, mu_pred, cov_pred, k_gain)

        return mu, cov

    def publish(self, pose_filtered_msg: PoseStamped):
        """
        Publishes the filtered tag pose.
        """
        pose_filtered = pose_filtered_msg
        pose_filtered.header.stamp = self.kf_current_time
        pose_filtered.header.frame_id = 'base_link'
        self.kf_tag_base_pose_pub.publish(pose_filtered)

if __name__ == '__main__':
    try:
        tfn = TagFilterNode()
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            tfn.filtering()
            rate.sleep()
    except rospy.ROSInterruptException:
        rospy.loginfo("Tag filtering finished.")
