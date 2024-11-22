# -*- encoding: utf-8 -*-
'''
@File    :   simulation.py
@Time    :   2024/11/19 12:47:07
@Author  :   Yali Nie 
@Version :   1.0
@Contact :   yalinie2015@gmail.com
'''

# import your libs here

import numpy as np
from utils.utils import wrap_angle, calculate_rmse, string_format, Vector2, generate_ellipse, generate_circle, offset_points
from utils.car import Car, Camera
from utils.beacons import BeaconMap
from utils.sensors import GyroSensor, GPSSensor, LidarSensor
from utils.kalmanfilter import KalmanFilter


SCREEN_WIDTH = 1024
SCREEN_HEIGHT = 768

class SimulationParams:
    def __init__(self):
        self.profile_name = ""
        self.time_step = 0.1
        self.end_time = 120
        self.gps_enabled = True
        self.gps_update_rate = 1.0
        self.gps_position_noise_std = 3
        self.gps_error_probability = 0.0
        self.gps_denied_x = 0.0
        self.gps_denied_y = 0.0
        self.gps_denied_range = -1.0
        self.lidar_enabled = False
        self.lidar_id_enabled = True
        self.lidar_update_rate = 10.0
        self.lidar_range_noise_std = 3
        self.lidar_theta_noise_std = 0.02
        self.gyro_enabled = True
        self.gyro_update_rate = 10.0
        self.gyro_noise_std = 0.001
        self.gyro_bias = 0.0
        self.car_initial_x = 0.0
        self.car_initial_y = 0.0
        self.car_initial_psi = 0.0
        self.car_initial_velocity = 5.0
        self.car_commands = []

class Simulation:
    def __init__(self, sim_parameters):
        #print(f"Simulation.__init__ called with sim_parameters: {sim_parameters}")
        self.m_sim_parameters = sim_parameters
        self.m_is_paused = False
        self.m_is_running = False
        self.m_time_multiplier = 1
        self.m_zoom_level = 1.0
        self.m_view_size = 100
        self.m_time = 0.0
        self.m_time_till_gyro_measurement = 0.0
        self.m_time_till_gps_measurement = 0.0
        self.m_time_till_lidar_measurement = 0.0
        self.m_kalman_filter = KalmanFilter()
        self.m_car = Car()
        self.m_beacons = BeaconMap()
        self.m_gyro_sensor = GyroSensor()
        self.m_gps_sensor = GPSSensor()
        self.m_lidar_sensor = LidarSensor()
        self.m_gps_measurement_history = []
        self.m_lidar_measurement_history = []
        self.m_vehicle_position_history = []
        self.m_filter_position_history = []
        self.m_filter_error_x_position_history = []
        self.m_filter_error_y_position_history = []
        self.m_filter_error_heading_history = []
        self.m_filter_error_velocity_history = []
        self.m_view_size = 1000
        self.camera = Camera(SCREEN_WIDTH, SCREEN_HEIGHT)
        #self.test_beacon_map_methods()

    # def test_beacon_map_methods(self):
    #     print("Simulation.test_beacon_map_methods called")
    #     # 获取所有信标
    #     beacons = self.m_beacons.get_beacons()
    #     print(f"Total beacons: {len(beacons)}")

    #     # 获取初始信标位置
    #     # initial_positions = self.m_beacons.get_initial_beacon_positions()
    #     # for i, (x, y) in enumerate(initial_positions):
    #     #     print(f"Initial Beacon {i}: ({x}, {y})")

    #     # 获取特定 ID 的信标
    #     beacon_id = 10
    #     beacon = self.m_beacons.get_beacon_with_id(beacon_id)
    #     print(f"Beacon with ID {beacon_id}: ({beacon.x}, {beacon.y})")

    #     # 获取指定范围内的信标
    #     x, y, range_ = 100, 100, 50
    #     beacons_in_range = self.m_beacons.get_beacons_within_range(x, y, range_)
    #     print(f"Beacons within range {range_} of ({x}, {y}): {len(beacons_in_range)}")


    def reset(self, sim_params=None):
        if sim_params is not None:
            self.m_sim_parameters = sim_params
        self.m_kalman_filter.initialize(
            np.array([0, 0, self.m_sim_parameters.car_initial_psi, self.m_sim_parameters.car_initial_velocity]),
            np.eye(4)
        )
        self.m_time = 0.0
        self.m_time_multiplier = 1
        self.m_zoom_level = 1.0
        self.m_time_till_gyro_measurement = 0.0
        self.m_time_till_gps_measurement = 0.0
        self.m_time_till_lidar_measurement = 0.0
        self.m_is_running = True
        self.m_is_paused = False
        self.m_kalman_filter.reset()
        self.m_gps_sensor.reset()
        self.m_gps_sensor.set_gps_noise_std(self.m_sim_parameters.gps_position_noise_std)
        self.m_gps_sensor.set_gps_error_prob(self.m_sim_parameters.gps_error_probability)
        self.m_gps_sensor.set_gps_denied_zone(self.m_sim_parameters.gps_denied_x, self.m_sim_parameters.gps_denied_y, self.m_sim_parameters.gps_denied_range)
        self.m_gyro_sensor.reset()
        self.m_gyro_sensor.set_gyro_noise_std(self.m_sim_parameters.gyro_noise_std)
        self.m_gyro_sensor.set_gyro_bias(self.m_sim_parameters.gyro_bias)
        self.m_lidar_sensor.reset()
        self.m_lidar_sensor.set_lidar_noise_std(self.m_sim_parameters.lidar_range_noise_std, self.m_sim_parameters.lidar_theta_noise_std)
        self.m_lidar_sensor.set_lidar_da_enabled(self.m_sim_parameters.lidar_id_enabled)
        self.m_car.reset(self.m_sim_parameters.car_initial_x, self.m_sim_parameters.car_initial_y, self.m_sim_parameters.car_initial_psi, self.m_sim_parameters.car_initial_velocity)
        for cmd in self.m_sim_parameters.car_commands:
            self.m_car.add_vehicle_command(cmd)
        self.m_gps_measurement_history.clear()
        self.m_lidar_measurement_history.clear()
        self.m_vehicle_position_history.clear()
        self.m_filter_position_history.clear()
        self.m_filter_error_x_position_history.clear()
        self.m_filter_error_y_position_history.clear()
        self.m_filter_error_heading_history.clear()
        self.m_filter_error_velocity_history.clear()
        print("Simulation: Reset")

    def update(self):
        if self.m_is_running and not self.m_is_paused:
            for _ in range(self.m_time_multiplier):
                if self.m_time >= self.m_sim_parameters.end_time:
                    self.m_is_running = False
                    print(f"Simulation: Reached End of Simulation Time ({self.m_time})")
                    return
                self.m_car.update(self.m_time, self.m_sim_parameters.time_step * self.m_time_multiplier)
                self.m_vehicle_position_history.append(Vector2(self.m_car.get_vehicle_state().x, self.m_car.get_vehicle_state().y))
                if self.m_sim_parameters.gyro_enabled:
                    if self.m_time_till_gyro_measurement <= 0:
                        meas = self.m_gyro_sensor.generate_gyro_measurement(self.m_car.get_vehicle_state().yaw_rate)
                        #print(f"Arguments for prediction_step: meas={meas}, time_step={self.m_sim_parameters.time_step}")
                        self.m_kalman_filter.prediction_step_with_gyro(meas, self.m_sim_parameters.time_step * self.m_time_multiplier)
                        self.m_time_till_gyro_measurement += 1.0 / self.m_sim_parameters.gyro_update_rate
                    self.m_time_till_gyro_measurement -= self.m_sim_parameters.time_step * self.m_time_multiplier   
                if self.m_sim_parameters.gps_enabled:
                    if self.m_time_till_gps_measurement <= 0:
                        gps_meas = self.m_gps_sensor.generate_gps_measurement(self.m_car.get_vehicle_state().x, self.m_car.get_vehicle_state().y)
                        self.m_kalman_filter.handle_gps_measurement(gps_meas)
                        self.m_gps_measurement_history.append(gps_meas)
                        self.m_time_till_gps_measurement += 1.0 / self.m_sim_parameters.gps_update_rate
                    self.m_time_till_gps_measurement -= self.m_sim_parameters.time_step * self.m_time_multiplier
                if self.m_sim_parameters.lidar_enabled:
                    if self.m_time_till_lidar_measurement <= 0:
                        lidar_measurements = self.m_lidar_sensor.generate_lidar_measurements(self.m_car.get_vehicle_state().x, self.m_car.get_vehicle_state().y, self.m_car.get_vehicle_state().psi, self.m_beacons)
                        self.m_kalman_filter.handle_lidar_measurements(lidar_measurements, self.m_beacons)
                        print(f"m_beacons: {self.m_beacons.get_beacons()}")
                        self.m_lidar_measurement_history = lidar_measurements
                        self.m_time_till_lidar_measurement += 1.0 / self.m_sim_parameters.lidar_update_rate
                    self.m_time_till_lidar_measurement -= self.m_sim_parameters.time_step * self.m_time_multiplier
                if self.m_kalman_filter.is_initialised():
                    vehicle_state = self.m_car.get_vehicle_state()
                    filter_state = self.m_kalman_filter.get_vehicle_state()
                    self.m_filter_position_history.append(Vector2(filter_state.x, filter_state.y))
                    self.m_filter_error_x_position_history.append(filter_state.x - vehicle_state.x)
                    self.m_filter_error_y_position_history.append(filter_state.y - vehicle_state.y)
                    self.m_filter_error_heading_history.append(wrap_angle(filter_state.psi - vehicle_state.psi))
                    self.m_filter_error_velocity_history.append(filter_state.V - vehicle_state.V)
                self.m_time += self.m_sim_parameters.time_step*self.m_time_multiplier # Increment time
                #print(f"Time: {self.m_time}, Car Position: {self.m_car.get_vehicle_state().x}, {self.m_car.get_vehicle_state().y}, Velocity: {self.m_car.get_vehicle_state().V}")



    def render(self, disp):
        marker_lines1 = [Vector2(0.5, 0.5), Vector2(-0.5, -0.5)]
        marker_lines2 = [Vector2(0.5, -0.5), Vector2(-0.5, 0.5)]
        car_state = self.m_car.get_vehicle_state()

        
        disp.set_view(self.m_view_size * disp.get_screen_aspect_ratio() * self.m_zoom_level, self.m_view_size * self.m_zoom_level, car_state.x, car_state.y)

        self.m_car.render(disp)


        self.m_beacons.render(disp)


        disp.set_draw_colour(0, 100, 0)
        disp.draw_lines(self.m_vehicle_position_history)

        disp.set_draw_colour(100, 0, 0)
        disp.draw_lines(self.m_filter_position_history)

        if self.m_kalman_filter.is_initialised():
            filter_state = self.m_kalman_filter.get_vehicle_state()
            cov = self.m_kalman_filter.get_vehicle_state_position_covariance()
            x = filter_state.x
            y = filter_state.y
            sigma_xx = cov[0, 0]
            sigma_yy = cov[1, 1]
            sigma_xy = cov[0, 1]
            marker_lines1_world = offset_points(marker_lines1, Vector2(x, y))
            marker_lines2_world = offset_points(marker_lines2, Vector2(x, y))
            disp.set_draw_colour(255, 0, 0)
            disp.draw_lines(marker_lines1_world)
            disp.draw_lines(marker_lines2_world)
            cov_world = generate_ellipse(x, y, sigma_xx, sigma_yy, sigma_xy)
            disp.set_draw_colour(255, 0, 0)
            disp.draw_lines(cov_world)

 
        m_gps_marker = [[Vector2(0.5, 0.5), Vector2(-0.5, -0.5)], [Vector2(0.5, -0.5), Vector2(-0.5, 0.5)]]
        disp.set_draw_colour(255, 255, 255)
        for meas in self.m_gps_measurement_history:
            disp.draw_lines(offset_points(m_gps_marker, Vector2(meas.x, meas.y)))


        if self.m_sim_parameters.gps_denied_range > 0:
            zone_lines = generate_circle(self.m_sim_parameters.gps_denied_x, self.m_sim_parameters.gps_denied_y, self.m_sim_parameters.gps_denied_range)
            disp.set_draw_colour(255, 150, 0)
            disp.draw_lines(zone_lines)

      
        for meas in self.m_lidar_measurement_history:
            x0 = car_state.x
            y0 = car_state.y
            delta_x = meas.range * np.cos(meas.theta + car_state.psi)
            delta_y = meas.range * np.sin(meas.theta + car_state.psi)
            disp.set_draw_colour(201, 201, 0)
            disp.draw_line(Vector2(x0, y0), Vector2(x0 + delta_x, y0 + delta_y))

        x_offset, y_offset = 10, 30
        stride = 20
        time_string = string_format("Time: %0.2f s", self.m_time)
        profile_string = string_format("Profile: %s", self.m_sim_parameters.profile_name)
        gps_string = string_format("GPS: %s (%0.1f Hz)", "ON" if self.m_sim_parameters.gps_enabled else "OFF", self.m_sim_parameters.gps_update_rate)
        lidar_string = string_format("LIDAR: %s (%0.1f Hz)", "ON" if self.m_sim_parameters.lidar_enabled else "OFF", self.m_sim_parameters.lidar_update_rate)
        gyro_string = string_format("GYRO: %s (%0.1f Hz)", "ON" if self.m_sim_parameters.gyro_enabled else "OFF", self.m_sim_parameters.gyro_update_rate)

        disp.draw_text_main_font(profile_string, Vector2(x_offset, y_offset + stride * -1), 1.0, (255, 255, 255))
        disp.draw_text_main_font(time_string, Vector2(x_offset, y_offset + stride * 0), 1.0, (255, 255, 255))
        disp.draw_text_main_font(gps_string, Vector2(x_offset, y_offset + stride * 1), 1.0, (255, 255, 255))
        disp.draw_text_main_font(lidar_string, Vector2(x_offset, y_offset + stride * 2), 1.0, (255, 255, 255))
        disp.draw_text_main_font(gyro_string, Vector2(x_offset, y_offset + stride * 3), 1.0, (255, 255, 255))
        if self.m_is_paused:
            disp.draw_text_main_font("PAUSED", Vector2(x_offset, y_offset + stride * 4), 1.0, (255, 0, 0))
        if not self.m_is_running:
            disp.draw_text_main_font("FINISHED", Vector2(x_offset, y_offset + stride * 5), 1.0, (255, 0, 0))
        x_offset, y_offset = 800, 10

        # Vehicle State
        velocity_string = string_format("    Velocity: %0.2f m/s", car_state.V)
        yaw_string = string_format("   Heading: %0.2f deg", car_state.psi * 180.0 / np.pi)
        xpos = string_format("X Position: %0.2f m", car_state.x)
        ypos = string_format("Y Position: %0.2f m", car_state.y)
        disp.draw_text_main_font("Vehicle State", Vector2(x_offset - 5, y_offset + stride * 0), 1.0, (255, 255, 255))
        disp.draw_text_main_font(velocity_string, Vector2(x_offset, y_offset + stride * 1), 1.0, (255, 255, 255))
        disp.draw_text_main_font(yaw_string, Vector2(x_offset, y_offset + stride * 2), 1.0, (255, 255, 255))
        disp.draw_text_main_font(xpos, Vector2(x_offset, y_offset + stride * 3), 1.0, (255, 255, 255))
        disp.draw_text_main_font(ypos, Vector2(x_offset, y_offset + stride * 4), 1.0, (255, 255, 255))

        # Filter State
        kf_velocity_string = string_format("    Velocity: %0.2f m/s", self.m_kalman_filter.get_vehicle_state().V)
        kf_yaw_string = string_format("   Heading: %0.2f deg", self.m_kalman_filter.get_vehicle_state().psi * 180.0 / np.pi)
        kf_xpos = string_format("X Position: %0.2f m", self.m_kalman_filter.get_vehicle_state().x)
        kf_ypos = string_format("Y Position: %0.2f m", self.m_kalman_filter.get_vehicle_state().y)
        disp.draw_text_main_font("Filter State", Vector2(x_offset, y_offset + stride * 6), 1.0, (255, 255, 255))
        disp.draw_text_main_font(kf_velocity_string, Vector2(x_offset, y_offset + stride * 7), 1.0, (255, 255, 255))
        disp.draw_text_main_font(kf_yaw_string, Vector2(x_offset, y_offset + stride * 8), 1.0, (255, 255, 255))
        disp.draw_text_main_font(kf_xpos, Vector2(x_offset, y_offset + stride * 9), 1.0, (255, 255, 255))
        disp.draw_text_main_font(kf_ypos, Vector2(x_offset, y_offset + stride * 10), 1.0, (255, 255, 255))

        # Error Metrics
        x_offset, y_offset = 10, 650
        disp.draw_text_main_font("Reset Key: r", Vector2(x_offset, y_offset + stride * 0), 1.0, (255, 255, 255))
        disp.draw_text_main_font("Pause Key: [space bar]", Vector2(x_offset, y_offset + stride * 1), 1.0, (255, 255, 255))
        #disp.draw_text_main_font("Speed Multiplier Key: [right/left] ", Vector2(x_offset, y_offset + stride * 2), 1.0, (255, 255, 255))
        #disp.draw_text_main_font("Zoom Key: [up/down]", Vector2(x_offset, y_offset + stride * 3), 1.0, (255, 255, 255))
        disp.draw_text_main_font("Motion Profile Key: 1 - 9,0", Vector2(x_offset, y_offset + stride * 2), 1.0, (255, 255, 255))

        x_offset, y_offset = 750, 650
        xpos_error_string = string_format("X Position RMSE: %0.2f m", calculate_rmse(self.m_filter_error_x_position_history))
        ypos_error_string = string_format("Y Position RMSE: %0.2f m", calculate_rmse(self.m_filter_error_y_position_history))
        heading_error_string = string_format("   Heading RMSE: %0.2f deg", 180.0 / np.pi * calculate_rmse(self.m_filter_error_heading_history))
        velocity_error_string = string_format("    Velocity RMSE: %0.2f m/s", calculate_rmse(self.m_filter_error_velocity_history))
        disp.draw_text_main_font(xpos_error_string, Vector2(x_offset, y_offset + stride * 0), 1.0, (255, 255, 255))
        disp.draw_text_main_font(ypos_error_string, Vector2(x_offset, y_offset + stride * 1), 1.0, (255, 255, 255))
        disp.draw_text_main_font(heading_error_string, Vector2(x_offset, y_offset + stride * 2), 1.0, (255, 255, 255))
        disp.draw_text_main_font(velocity_error_string, Vector2(x_offset, y_offset + stride * 3), 1.0, (255, 255, 255))

        
        debug_info = f"P({car_state.x:.2f}, {car_state.y:.2f})"
        disp.draw_text_main_font(debug_info, Vector2(car_state.x + 10, car_state.y + 10), 1.0, (255, 255, 255))

    def toggle_pause_simulation(self):
        self.m_is_paused = not self.m_is_paused  # Toggle pause
        print(f"Simulation paused: {self.m_is_paused}")