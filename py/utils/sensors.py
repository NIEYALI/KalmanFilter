# -*- encoding: utf-8 -*-
'''
@File    :   sensors.py
@Time    :   2024/11/22 12:47:08
@Author  :   Yali Nie 
@Version :   1.0
@Contact :   yalinie2015@gmail.com
'''

# import your libs here


import numpy as np
import random
from collections import namedtuple
from .utils import wrap_angle

GPSMeasurement = namedtuple('GPSMeasurement', ['x', 'y'])
GyroMeasurement = namedtuple('GyroMeasurement', ['psi_dot'])
LidarMeasurement = namedtuple('LidarMeasurement', ['range', 'theta', 'id'])

class GPSSensor:
    def __init__(self):
        self.reset()
        self.m_noise_std = 0.0
        self.m_error_prob = 0.0
        self.m_gps_denied_x = 0.0
        self.m_gps_denied_y = 0.0
        self.m_gps_denied_range = -1.0

    def reset(self):
        self.m_rand_gen = random.Random()

    def set_gps_noise_std(self, std):
        self.m_noise_std = std

    def set_gps_error_prob(self, prob):
        self.m_error_prob = prob

    def set_gps_denied_zone(self, x, y, r):
        self.m_gps_denied_x = x
        self.m_gps_denied_y = y
        self.m_gps_denied_range = r

    def generate_gps_measurement(self, sensor_x, sensor_y):
        meas = GPSMeasurement(
            x=sensor_x + self.m_rand_gen.gauss(0, self.m_noise_std),
            y=sensor_y + self.m_rand_gen.gauss(0, self.m_noise_std)
        )
        if self.m_rand_gen.uniform(0, 1) < self.m_error_prob:
            meas = GPSMeasurement(x=0, y=0)
        delta_x = sensor_x - self.m_gps_denied_x
        delta_y = sensor_y - self.m_gps_denied_y
        range_ = np.sqrt(delta_x**2 + delta_y**2)
        if range_ < self.m_gps_denied_range:
            meas = GPSMeasurement(x=0, y=0)
        return meas

class GyroSensor:
    def __init__(self):
        self.reset()
        self.m_noise_std = 0.0
        self.m_bias = 0.0

    def reset(self):
        self.m_rand_gen = random.Random()

    def set_gyro_noise_std(self, std):
        self.m_noise_std = std

    def set_gyro_bias(self, bias):
        self.m_bias = bias

    def generate_gyro_measurement(self, sensor_yaw_rate):
        meas = GyroMeasurement(
            psi_dot=sensor_yaw_rate + self.m_bias + self.m_rand_gen.gauss(0, self.m_noise_std)
        )
        return meas

class LidarSensor:
    def __init__(self):
        self.reset()
        self.m_range_noise_std = 0.0
        self.m_theta_noise_std = 0.0
        self.m_max_range = 90.0
        self.m_id_enabled = True

    def reset(self):
        self.m_rand_gen = random.Random()

    def set_lidar_noise_std(self, range_std, theta_std):
        self.m_range_noise_std = range_std
        self.m_theta_noise_std = theta_std

    def set_lidar_max_range(self, range_):
        self.m_max_range = range_

    def set_lidar_da_enabled(self, id_enabled):
        self.m_id_enabled = id_enabled

    def generate_lidar_measurements(self, sensor_x, sensor_y, sensor_yaw, beacon_map):
        measurements = []
        for beacon in beacon_map.get_beacons():
            delta_x = beacon.x - sensor_x
            delta_y = beacon.y - sensor_y
            theta = wrap_angle(np.arctan2(delta_y, delta_x) - sensor_yaw)
            beacon_range = np.sqrt(delta_x**2 + delta_y**2)
            if beacon_range < self.m_max_range:
                beacon_meas = LidarMeasurement(
                    range=abs(beacon_range + self.m_rand_gen.gauss(0, self.m_range_noise_std)),
                    theta=wrap_angle(theta + self.m_rand_gen.gauss(0, self.m_theta_noise_std)),
                    id=beacon.id if self.m_id_enabled else -1
                )
                measurements.append(beacon_meas)
        return measurements