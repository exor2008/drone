import numpy as np


class KalmanFilter:
    def __init__(self, dim_x, dim_z):
        # State vector (9 elements for 9DOF)
        self.x = np.zeros(dim_x)

        # State covariance matrix
        self.P = np.eye(dim_x)

        # State transition matrix A
        self.F = np.eye(dim_x)

        # Measurement matrix H
        self.H = np.eye(dim_z, dim_x)

        # Process noise covariance Q
        self.Q = np.eye(dim_x) * 0.01

        # Measurement noise covariance R
        self.R = np.eye(dim_z) * 0.1

    def predict(self):
        # Prediction step
        self.x = np.dot(self.F, self.x)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q

    def update(self, measurement):
        # Update step
        y = measurement - np.dot(self.H, self.x)
        S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        self.x = self.x + np.dot(K, y)
        self.P = np.dot((np.eye(len(self.x)) - np.dot(K, self.H)), self.P)
