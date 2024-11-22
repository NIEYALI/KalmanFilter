import numpy as np

class KalmanFilter:
    def __init__(self, initial_state, initial_covariance, process_noise, measurement_noise, transition_matrix, measurement_matrix):
        self.state = initial_state
        self.covariance = initial_covariance
        self.process_noise = process_noise
        self.measurement_noise = measurement_noise
        self.transition_matrix = transition_matrix
        self.measurement_matrix = measurement_matrix

    def predict(self):
        self.state = self.transition_matrix @ self.state
        self.covariance = self.transition_matrix @ self.covariance @ self.transition_matrix.T + self.process_noise

    def update(self, measurement):
        S = self.measurement_matrix @ self.covariance @ self.measurement_matrix.T + self.measurement_noise
        K = self.covariance @ self.measurement_matrix.T @ np.linalg.inv(S)
        self.state = self.state + K @ (measurement - self.measurement_matrix @ self.state)
        I = np.eye(self.covariance.shape[0])
        self.covariance = (I - K @ self.measurement_matrix) @ self.covariance