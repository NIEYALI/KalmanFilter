# -*- encoding: utf-8 -*-
'''
@File    :   kalmanfilter.py
@Time    :   2024/11/22 12:46:47
@Author  :   Yali Nie 
@Version :   1.0
@Contact :   yalinie2015@gmail.com
'''

# import your libs here

import numpy as np
import time
from .car import VehicleState
from .sensors import GyroMeasurement, GPSMeasurement, LidarMeasurement
from .beacons import BeaconMap

class KalmanFilterBase:
    def __init__(self):
        self._initialised = False
        self._state = None
        self._covariance = None

    def reset(self):
        self._initialised = False

    def is_initialised(self):
        return self._initialised

    def get_state(self):
        return self._state

    def get_covariance(self):
        return self._covariance

    def set_state(self, state):
        self._state = state
        self._initialised = True

    def set_covariance(self, cov):
        self._covariance = cov

class KalmanFilter(KalmanFilterBase):
    def initialize(self, initial_state, initial_covariance):
        self.set_state(initial_state)
        self.set_covariance(initial_covariance)
        
    def get_vehicle_state(self):
        state = self.get_state()
        return VehicleState(
            x=state[0],
            y=state[1],
            psi=state[2],
            V=state[3]
        )

    def get_vehicle_state_position_covariance(self):
        cov = self.get_covariance()
        return cov[:2, :2]

    def prediction_step(self, dt):
        state = self.get_state()
        F = np.eye(4)
        F[0, 3] = dt * np.cos(state[2])
        F[1, 3] = dt * np.sin(state[2])
        F[2, 3] = dt
        self.set_state(F @ state)

    def prediction_step_with_gyro(self, gyro, dt):
        state = self.get_state()
        F = np.eye(4)
        F[0, 3] = dt * np.cos(state[2])
        F[1, 3] = dt * np.sin(state[2])
        F[2, 3] = dt
        state[2] += gyro.psi_dot * dt
        self.set_state(F @ state)

    def handle_lidar_measurements(self, measurements, map):
        for meas in measurements:
            self.handle_lidar_measurement(meas, map)

    # def handle_lidar_measurement(self, meas, map):
    #     state = self.get_state()
    #     H = np.zeros((2, 4))
    #     H[0, 0] = 1
    #     H[1, 1] = 1
    #     z = np.array([meas.range * np.cos(meas.theta), meas.range * np.sin(meas.theta)])
    #     y = z - H @ state
    #     S = H @ self.get_covariance() @ H.T
    #     K = self.get_covariance() @ H.T @ np.linalg.inv(S)
    #     self.set_state(state + K @ y)
    #     self.set_covariance((np.eye(4) - K @ H) @ self.get_covariance())
    def handle_lidar_measurement(self, meas, map):
        state = self.get_state()
        H = np.zeros((2, 4))
        H[0, 0] = 1
        H[1, 1] = 1
        z = np.array([meas.range * np.cos(meas.theta), meas.range * np.sin(meas.theta)])
        y = z - H @ state
        P = self.get_covariance()
        R = np.eye(2) * 10  # 增加测量噪声协方差
        S = H @ P @ H.T + R + np.eye(2) * 1e-6  # 正则化 S

        # print(f"H: {H}")
        # print(f"R: {R}")
        # print(f"P: {P}")
        # print(f"S: {S}")

        try:
            K = P @ H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError as e:
            print(f"Error inverting S: {e}")
            return

        self.set_state(state + K @ y)
        self.set_covariance((np.eye(4) - K @ H) @ P)
    # def handle_gps_measurement(self, meas):
    #     state = self.get_state()
    #     H = np.zeros((2, 4))
    #     H[0, 0] = 1
    #     H[1, 1] = 1
    #     z = np.array([meas.x, meas.y])
    #     y = z - H @ state
    #     S = H @ self.get_covariance() @ H.T
    #     K = self.get_covariance() @ H.T @ np.linalg.inv(S)
    #     self.set_state(state + K @ y)
    #     self.set_covariance((np.eye(4) - K @ H) @ self.get_covariance())
    def handle_gps_measurement(self, meas):
        state = self.get_state()
        H = np.zeros((2, 4))
        H[0, 0] = 1
        H[1, 1] = 1
        z = np.array([meas.x, meas.y])
        y = z - H @ state
        P = self.get_covariance()
        R = np.eye(2)*10  # Assuming measurement noise covariance is an identity matrix
        S = H @ P @ H.T + R + np.eye(2) * 1e-6  # Regularize S

        # print(f"H: {H}")
        # print(f"R: {R}")
        # print(f"P: {P}")
        # print(f"S: {S}")

        #time.sleep(5)

        try:
            K = P @ H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError as e:
            print(f"Error inverting S: {e}")
            return

        self.set_state(state + K @ y)
        self.set_covariance((np.eye(4) - K @ H) @ P)