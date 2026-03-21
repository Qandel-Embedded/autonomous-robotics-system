"""Kalman Filter for IMU + Encoder sensor fusion."""
import numpy as np


class KalmanFilter:
    """1D Kalman filter for position/velocity estimation."""

    def __init__(self, dt=0.01, process_noise=0.1, measurement_noise=0.5):
        self.dt = dt
        self.A = np.array([[1, dt], [0, 1]])         # State transition
        self.H = np.array([[1, 0]])                   # Measurement
        self.Q = np.eye(2) * process_noise            # Process noise
        self.R = np.array([[measurement_noise]])      # Measurement noise
        self.P = np.eye(2)                            # Error covariance
        self.x = np.zeros((2, 1))                    # State [pos, vel]

    def predict(self):
        self.x = self.A @ self.x
        self.P = self.A @ self.P @ self.A.T + self.Q
        return self.x[0, 0]

    def update(self, measurement):
        y = measurement - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)    # Kalman gain
        self.x = self.x + K * y
        self.P = (np.eye(2) - K @ self.H) @ self.P
        return self.x[0, 0]


class SensorFusion:
    """Fuses IMU accelerometer and wheel encoder data."""

    def __init__(self):
        self.kf_x = KalmanFilter()
        self.kf_y = KalmanFilter()

    def update(self, imu_ax, imu_ay, enc_vx, enc_vy, dt=0.01):
        # Integrate acceleration to velocity (encoder corrects drift)
        self.kf_x.predict()
        self.kf_y.predict()
        pos_x = self.kf_x.update(enc_vx * dt)
        pos_y = self.kf_y.update(enc_vy * dt)
        return pos_x, pos_y
