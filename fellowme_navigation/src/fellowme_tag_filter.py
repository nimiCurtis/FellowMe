#!/usr/bin/env python3

import numpy as np
from utils.kalmanfilter import KalmanFilter
from utils.utils import pose_to_numpyStateWithCov,pose_to_numpyState, numpyStateWithCov_to_pose

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from apriltag_ros.msg import AprilTagDetectionArray
import tf
from tf import TransformListener
from tf.transformations import *

class TagFilterNode:
    
    def __init__(self):
        rospy.init_node('fellowme_tag_filter')
        
        self.path_pub = rospy.Publisher('kf/tag_detections_trajectory', Path, latch=True, queue_size=1000)
        self.kf_tag_base_pose_pub = rospy.Publisher('kf/tag_detections_in_base', PoseWithCovarianceStamped, queue_size=1000)
        self.tag_in_base_sub = rospy.Subscriber('/tag_detections_in_base', PoseWithCovarianceStamped,self.tag_callback)
        self.tag_in_base = None

        self.path = Path()
        self.tl = TransformListener()
        self.initial_covariance_vel = rospy.get_param('/apriltag_ros_continuous_node/initial_estimate_vel_covariance')
        self.sensor_noise = rospy.get_param('/apriltag_ros_continuous_node/observation_noise_covariance')
        self.sigma_n = rospy.get_param('/apriltag_ros_continuous_node/sigma_n')

        self.kf = KalmanFilter(sensor_noise=self.sensor_noise)
    
    
    def start_filter(self):
        self.kf_last_time = rospy.Time().now()
        self.kf_current_time = rospy.Time().now()
        return 0.
    
    def start_drck(self):
        pass
        
    def tag_callback(self, msg:PoseWithCovarianceStamped):
        self.tag_in_base = msg
        

    def take_time_diff(self):
        self.kf_current_time = rospy.Time.now()
        dt = (self.kf_current_time - self.kf_last_time).to_sec()
        self.kf_last_time = self.kf_current_time
        
        return dt

    def filter(self, mu_prev, cov_prev, tag_measurment,dt):

        # set A - maps the previous state to the current state in the prediction step
        A = np.diag([1.]*12)
        dt_diag = np.diag([dt]*6)
        A[:6,6:] = dt_diag
        
        # set R - the process noise we add to the covariance. TODO: need to add sigma_n
        R = np.diag([1.]*12)
        R[:6,:6]*=dt
        R = (self.sigma_n**2)*R
        
        mu_pred, cov_pred = self.kf.predict(mu_prev=mu_prev, cov_prev=cov_prev, A_t=A,R_t=R)
        
        if tag_measurment is None:
            k_gain = np.zeros((12,6))
            measurment = np.zeros((6,1))
        else:
            k_gain = self.kf.get_kalman_gain(cov_t_pred=cov_pred)
            measurment = pose_to_numpyState(tag_measurment.pose.pose)

        mu, cov = self.kf.correct(measurment,mu_pred,cov_pred,k_gain)

        return mu, cov

    def wait_for_measure(self, start=False):
        try:
            tag_in_base = rospy.wait_for_message('/tag_detections_in_base', PoseWithCovarianceStamped,timeout=1/30)
        except:
            tag_in_base = None
        
        if start:
            while tag_in_base is None:
                try:
                    tag_in_base = rospy.wait_for_message('/tag_detections_in_base', PoseWithCovarianceStamped,timeout=1/30)    
                except:
                    pass
                
            dt = self.start_filter()
        else:
            dt = self.take_time_diff()
            
        return tag_in_base, dt
    
    def publish(self,pose_filtered_msg:PoseWithCovarianceStamped):
        pose_filtered = pose_filtered_msg
        pose_filtered.header.stamp = self.kf_current_time
        pose_filtered.header.frame_id = 'base_link'
        self.kf_tag_base_pose_pub.publish(pose_filtered)
    


if __name__ == '__main__':
    
    tfn = TagFilterNode()
    rate = rospy.Rate(15)
    first_meas,_ = tfn.wait_for_measure(start=True)
    mu_prev, cov_prev = pose_to_numpyStateWithCov(first_meas,vel_state=np.zeros((6,1)),vel_cov=tfn.initial_covariance_vel)
    
    while not rospy.is_shutdown():
        tag_measurment, dt = tfn.wait_for_measure()
        
        mu_filtered, cov_filtered = tfn.filter(mu_prev,cov_prev,tag_measurment,dt)
        
        mu_prev, cov_prev = mu_filtered, cov_filtered
        
        pose_filtered = numpyStateWithCov_to_pose(mu_prev,cov_prev)
        
        tfn.publish(pose_filtered_msg=pose_filtered)
        rate.sleep()