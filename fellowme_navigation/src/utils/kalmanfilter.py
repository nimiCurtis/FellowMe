import numpy as np
from utils.utils import normalize_angles_array

class KalmanFilter:
    def __init__(self, sensor_noise) -> None:
        """
        Initializes the KalmanFilter with sensor noise properties.
        """
        # Observation matrix H maps to the state dimension
        self.H = np.hstack((np.diag([1]*3), np.zeros((3, 3))))
        
        # Sensor noise matrix Q
        self.Q = np.array(sensor_noise).reshape(3, 3)

    def get_kalman_gain(self, cov_t_pred):
        """
        Calculates the Kalman gain K.
        """
        K_gain = cov_t_pred.dot(self.H.T).dot(np.linalg.inv(self.H.dot(cov_t_pred).dot(self.H.T) + self.Q))
        return K_gain

    def predict(self, mu_prev, cov_prev, A_t, R_t):
        """
        Performs the prediction step of the Kalman filter.
        """
        mu_pred = self.get_state(mu_prev, A_t)
        cov_pred = self.get_cov(cov_prev, A_t, R_t)
        mu_pred[2] = normalize_angles_array(mu_pred[2])
        return mu_pred, cov_pred
    
    def correct(self, measurement, mu_t_pred, cov_t_pred, k_gain):
        """
        Performs the correction step of the Kalman filter.
        """
        z_t = measurement
        # Update the estimate and error covariance matrices
        mu_t_cor =  mu_t_pred + k_gain.dot(z_t - self.H.dot(mu_t_pred))
        cov_t_cor = (np.eye(cov_t_pred.shape[0]) - k_gain.dot(self.H)).dot(cov_t_pred)
        mu_t_cor[2] = normalize_angles_array(mu_t_cor[2])
        return mu_t_cor, cov_t_cor

    def get_state(self, mu_prev, A_t):
        """
        Calculates the predicted state.
        """
        return (A_t.dot(mu_prev))

    def get_cov(self, cov_prev, A_t, R_t):
        """
        Calculates the predicted covariance matrix.
        """
        return (A_t.dot(cov_prev)).dot(A_t.T) + R_t
