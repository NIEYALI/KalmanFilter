# -*- encoding: utf-8 -*-
'''
@File    :   beacons.py
@Time    :   2024/11/22 12:41:27
@Author  :   Yali Nie 
@Version :   1.0
@Contact :   yalinie2015@gmail.com
'''

# import your libs here

import numpy as np
import random

class Vector2:
    def __init__(self, x=0.0, y=0.0):
        self.x = x
        self.y = y

    def __repr__(self):
        return f"Vector2({self.x}, {self.y})"

def offset_points(points, offset):
    return [Vector2(point.x + offset.x, point.y + offset.y) for point in points]

class BeaconData:
    def __init__(self, x=0.0, y=0.0, beacon_id=-1):
        self.x = x
        self.y = y
        self.id = beacon_id

class BeaconMap:
    def __init__(self):
        self.m_beacon_map = []
        screen_width = 1023
        screen_height = 767
        pos_dis_x = lambda: random.uniform(0.0, screen_width)
        pos_dis_y = lambda: random.uniform(0.0, screen_height)
        for i in range(450):
            x = pos_dis_x()
            y = pos_dis_y()
            self.add_beacon(x, y)

    def add_beacon(self, x, y):
        self.m_beacon_map.append(BeaconData(x, y, len(self.m_beacon_map)))

    def get_beacon_with_id(self, beacon_id):
        for beacon in self.m_beacon_map:
            if beacon.id == beacon_id:
                return beacon
        return BeaconData()

    def get_beacons_within_range(self, x, y, range_):
        beacons = []
        for beacon in self.m_beacon_map:
            delta_x = beacon.x - x
            delta_y = beacon.y - y
            beacon_range = np.sqrt(delta_x**2 + delta_y**2)
            if beacon_range < range_:
                beacons.append(beacon)
        return beacons

    def get_beacons(self):
        return self.m_beacon_map

    def render(self, disp):
        beacon_lines = [Vector2(1, 0), Vector2(0, 1), Vector2(0, -1), Vector2(1, 0)]
        disp.set_draw_colour(255, 255, 0)
        for beacon in self.m_beacon_map:
            disp.draw_lines(offset_points(beacon_lines, Vector2(beacon.x, beacon.y)))