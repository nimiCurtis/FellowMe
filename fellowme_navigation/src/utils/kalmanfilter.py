import numpy as np

class KalmanFilter:
    
    def __init__(self,sensor_noise) -> None:
        
        # maps the observation matrix to the state dimension
        self.H = np.hstack((np.diag([1]*6),np.zeros((6,6))))
        
        # sensor noise matrix
        self.Q = np.array(sensor_noise).reshape(6,6)

    def get_kalman_gain(self,cov_t_pred):
        # Calculate the Kalman gain K
        K_gain = cov_t_pred.dot(self.H.T).dot(np.linalg.inv(self.H.dot(cov_t_pred).dot(self.H.T) + self.Q))
        return K_gain


    def predict(self,mu_prev,cov_prev,A_t,R_t):
        mu_pred = self.get_state(mu_prev,A_t)
        cov_pred = self.get_cov(cov_prev,A_t,R_t)

        return mu_pred, cov_pred
    
    def correct(self,measurment, mu_t_pred, cov_t_pred, k_gain):
        z_t = measurment
        # Update the estimate and error covariance matrices
        mu_t_cor =  mu_t_pred + k_gain.dot(z_t - self.H.dot(mu_t_pred))
        cov_t_cor = (np.eye(cov_t_pred.shape[0]) - k_gain.dot(self.H)).dot(cov_t_pred)

        return mu_t_cor, cov_t_cor

    def get_state(self, mu_prev, A_t):
        return (A_t.dot(mu_prev))

    def get_cov(self,cov_prev,A_t,R_t):
        return (A_t.dot(cov_prev)).dot(A_t.T) + R_t





# class ExtendedKalmanFilter:
#     """
#     class for the implementation of the extended Kalman filter
#     """
#     def __init__():
#         """
#         Args:
#             enu_noise: enu data with noise
#             times: elapsed time in seconds from the first timestamp in the sequence
#             sigma_xy: sigma in the x and y axis as provided in the question
#             sigma_n: hyperparameter used to fine tune the filter
#             yaw_vf_wz: the yaw, forward velocity and angular change rate to be used (either non noisy or noisy, depending on the question)
#             sigma_theta: sigma of the heading
#             sigma_vf: sigma of the forward velocity
#             sigma_wz: sigma of the angular change rate
#             k: hyper parameter to fine tune the filter
#             is_dead_reckoning: should dead reckoning be applied after 5.0 seconds when applying the filter
#             dead_reckoning_start_sec: from what second do we start applying dead reckoning, used for experimentation only
#         """
#         self.enu_gt = enu_gt # for calibration purposes
#         self.enu_noise = enu_noise
#         self.yaw_vf_wz = yaw_vf_wz
#         self.times = times
#         self.sigma_xy = sigma_xy
#         self.sigma_theta = sigma_theta
#         self.sigma_vf = sigma_vf
#         self.sigma_wz = sigma_wz
#         self.sigma_r = sigma_r
#         self.k = k
#         self.is_dead_reckoning = is_dead_reckoning
#         self.dead_reckoning_start_sec = dead_reckoning_start_sec


#     def initialize(self):
#         pass

#     def prediction(self,g,cov_prev,G_t,V_t,R_t1,R_t2):
#         mu_pred = self.get_state(g)
#         cov_pred = self.get_cov(cov_prev,G_t,V_t,R_t1,R_t2)

#         return mu_pred, cov_pred
    
#     def get_state(self,g):
#         return g
    
#     def g(self,ut, mu_prev,dt):
#         vt = ut[0]
#         wt = ut[1]
#         mu = mu_prev + np.array([-(vt/wt)*np.sin(mu_prev[2])+(vt/wt)*np.sin(mu_prev[2]+wt*dt),
#                                 (vt/wt)*np.cos(mu_prev[2])-(vt/wt)*np.cos(mu_prev[2]+wt*dt),
#                                 wt*dt]).T
#         return mu


#     def get_cov(self,cov_prev,G_t,V_t,R_t1,R_t2):
#         return (G_t.dot(cov_prev)).dot(G_t.T) + (V_t.dot(R_t1)).dot(V_t.T) + R_t2

#     def get_kalman_gain(self,cov_t_pred,H,Q_t):
#         # Calculate the Kalman gain K
#         K_gain = cov_t_pred.dot(H.T).dot(np.linalg.inv(H.dot(cov_t_pred).dot(H.T) + Q_t))
#         return K_gain

#     def correction(self,mu_t_pred,cov_t_pred,k_gain,H,step):
#         z_t = self.get_meas(step)
#         # Update the estimate and error covariance matrices
#         mu_t_cor =  mu_t_pred + k_gain.dot(z_t - H.dot(mu_t_pred))
#         cov_t_cor = (np.eye(cov_t_pred.shape[0]) - k_gain.dot(H)).dot(cov_t_pred)

#         return mu_t_cor, cov_t_cor

#     def get_meas(self,step): #mu_t_pred,c):
#         # return (c.dot(mu_t_pred)).T
#         return np.array([self.enu_noise[step,0],self.enu_noise[step,1]]).T #check if velocity need to be 0 ? 
