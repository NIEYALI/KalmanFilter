# -*- encoding: utf-8 -*-
'''
@File    :   car.py
@Time    :   2024/11/22 12:46:10
@Author  :   Yali Nie 
@Version :   1.0
@Contact :   yalinie2015@gmail.com
'''

# import your libs here

import numpy as np
from collections import deque

class Vector2:
    def __init__(self, x=0.0, y=0.0):
        self.x = x
        self.y = y

    def __repr__(self):
        return f"Vector2({self.x}, {self.y})"

def wrap_angle(angle):
    angle = np.mod(angle, 2.0 * np.pi)
    if angle < -np.pi:
        angle += 2.0 * np.pi
    elif angle >= np.pi:
        angle -= 2.0 * np.pi
    return angle

class VehicleState:
    def __init__(self, x=0.0, y=0.0, psi=0.0, V=0.0, yaw_rate=0.0, steering=0.0,acceleration=0.0):
        self.x = x
        self.y = y
        self.psi = psi
        self.V = V
        self.yaw_rate = yaw_rate
        self.steering = steering
        self.acceleration = acceleration

class MotionCommandBase:
    def __init__(self):
        self.m_velocity_command = 0.0
        self.m_steering_command = 0.0
        self.m_start_time = 0.0
        self.m_start_state = None

    def start_command(self, time, state):
        self.m_start_time = time
        self.m_start_state = state

    def end_command(self, time, dt, state):
        pass

    def update(self, time, dt, state):
        return False

    def get_velocity_command(self):
        return self.m_velocity_command

    def get_steering_command(self):
        return self.m_steering_command

class MotionCommandStraight(MotionCommandBase):
    def __init__(self, command_time, command_velocity):
        super().__init__()
        self.m_command_time = command_time
        self.m_command_velocity = command_velocity

    def update(self, time, dt, state):
        self.m_velocity_command = self.m_command_velocity
        self.m_steering_command = 0.0
        return time > (self.m_start_time + self.m_command_time)

class MotionCommandAccelerate(MotionCommandBase):
    def __init__(self, command_velocity):
        super().__init__()
        self.m_command_velocity = command_velocity

    def update(self, time, dt, state):
        self.m_velocity_command = self.m_command_velocity
        self.m_steering_command = 0.0
        return True  # Always complete

class MotionCommandDecelerate(MotionCommandBase):
    def __init__(self, command_velocity):
        super().__init__()
        self.m_command_velocity = command_velocity

    def update(self, time, dt, state):
        self.m_velocity_command = self.m_command_velocity
        self.m_steering_command = 0.0
        return True  # Always complete

class MotionCommandTurnTo(MotionCommandBase):
    def __init__(self, command_heading, command_velocity):
        super().__init__()
        self.m_command_heading = command_heading
        self.m_command_velocity = command_velocity

    def update(self, time, dt, state):
        self.m_velocity_command = self.m_command_velocity
        angle_error = wrap_angle(self.m_command_heading - state.psi)
        self.m_steering_command = angle_error * (-1.0 if np.signbit(state.V) else 1.0)
        return np.abs(angle_error) < 0.001

class MotionCommandMoveTo(MotionCommandBase):
    def __init__(self, command_x, command_y, command_velocity):
        super().__init__()
        self.m_command_x = command_x
        self.m_command_y = command_y
        self.m_command_velocity = command_velocity

    def update(self, time, dt, state):
        self.m_velocity_command = self.m_command_velocity
        delta_x = self.m_command_x - state.x
        delta_y = self.m_command_y - state.y
        range_ = np.sqrt(delta_x**2 + delta_y**2)
        angle_command = np.arctan2(delta_y, delta_x)
        psi = wrap_angle(state.psi - (np.pi if np.signbit(state.V) else 0.0))
        angle_error = wrap_angle(angle_command - psi)
        self.m_steering_command = angle_error * (-1.0 if np.signbit(state.V) else 1.0)
        return range_ < 5.0

class BicycleMotion:
    def __init__(self, x0=0, y0=0, psi0=0, V0=0):
        self.m_initial_state = VehicleState(x0, y0, psi0, V0)
        self.m_wheel_base = 4.0
        self.m_max_velocity = 28.0
        self.m_max_acceleration = 2.0
        self.m_max_steering = 0.8
        self.reset()

    def reset(self):
        self.m_current_state = self.m_initial_state
        self.m_steering_command = self.m_initial_state.steering
        self.m_velocity_command = self.m_initial_state.V

    def reset_with_state(self, state):
        self.m_initial_state = state
        self.reset()

    def update(self, dt):
        cosPsi = np.cos(self.m_current_state.psi)
        sinPsi = np.sin(self.m_current_state.psi)
        x = self.m_current_state.x + self.m_current_state.V * cosPsi * dt
        y = self.m_current_state.y + self.m_current_state.V * sinPsi * dt

        accel = self.m_velocity_command - self.m_current_state.V
        accel = np.clip(accel, -self.m_max_acceleration, self.m_max_acceleration)

        steer = self.m_steering_command
        steer = np.clip(steer, -self.m_max_steering, self.m_max_steering)

        vel = self.m_current_state.V + accel * dt
        vel = np.clip(vel, -self.m_max_velocity, self.m_max_velocity)

        psi_dot = self.m_current_state.V * steer / self.m_wheel_base
        psi = wrap_angle(self.m_current_state.psi + psi_dot * dt)

        self.m_current_state = VehicleState(x, y, psi, vel, psi_dot, steer, accel)

    def set_steering_cmd(self, steer):
        self.m_steering_command = steer

    def set_velocity_cmd(self, velocity):
        self.m_velocity_command = velocity

    def get_vehicle_state(self):
        return self.m_current_state

class Car:
    def __init__(self):
        self.state = VehicleState()
        self.m_vehicle_model = BicycleMotion()
        self.m_current_command = None
        self.m_vehicle_commands = deque()

        # Create Display Geometry
        # self.m_car_lines_body = [Vector2(2, -1), Vector2(2, 1), Vector2(-2, 1), Vector2(-2, -1), Vector2(2, -1)]
        # self.m_marker_lines = [[Vector2(0.5, 0.5), Vector2(-0.5, -0.5)], [Vector2(0.5, -0.5), Vector2(-0.5, 0.5)], [Vector2(0, 0), Vector2(3.5, 0)]]
        # self.m_wheel_lines = [Vector2(-0.6, 0.3), Vector2(0.6, 0.3), Vector2(0.6, -0.3), Vector2(-0.6, -0.3), Vector2(-0.6, 0.3)]
        # self.m_wheel_fl_offset = Vector2(2, -1.6)
        # self.m_wheel_fr_offset = Vector2(2, 1.6)
        # self.m_wheel_rl_offset = Vector2(-2, -1.6)
        # self.m_wheel_rr_offset = Vector2(-2, 1.6)

        # Create Display Geometry
        # 增大车身尺寸
        self.m_car_lines_body = [Vector2(3, -1.5), Vector2(3, 1.5), Vector2(-3, 1.5), Vector2(-3, -1.5), Vector2(3, -1.5)]

        # 保持标记线不变
        self.m_marker_lines = [[Vector2(0.5, 0.5), Vector2(-0.5, -0.5)], [Vector2(0.5, -0.5), Vector2(-0.5, 0.5)], [Vector2(0, 0), Vector2(3.5, 0)]]

        # 保持轮子尺寸不变或减小
        self.m_wheel_lines = [Vector2(-0.1, 0.05), Vector2(0.1, 0.05), Vector2(0.1, -0.05), Vector2(-0.1, -0.05), Vector2(-0.1, 0.05)]

        # 调整轮子的位置以适应新的车身尺寸
        self.m_wheel_fl_offset = Vector2(3, -1.8)
        self.m_wheel_fr_offset = Vector2(3, 1.8)
        self.m_wheel_rl_offset = Vector2(-3, -1.8)
        self.m_wheel_rr_offset = Vector2(-3, 1.8)

    def reset(self, x0, y0, psi0, V0):
        self.m_vehicle_model.reset_with_state(VehicleState(x0, y0, psi0, V0))
        self.m_vehicle_commands.clear()
        self.m_current_command = None

    def add_vehicle_command(self, cmd):
        if cmd is not None:
            self.m_vehicle_commands.append(cmd)

    def get_vehicle_state(self):
        return self.m_vehicle_model.get_vehicle_state()

    def update(self, time, dt):
        # Update Command
        if self.m_current_command is None and self.m_vehicle_commands:
            self.m_current_command = self.m_vehicle_commands.popleft()
            self.m_current_command.start_command(time, self.m_vehicle_model.get_vehicle_state())

        # Run Command
        if self.m_current_command is not None:
            cmd_complete = self.m_current_command.update(time, dt, self.m_vehicle_model.get_vehicle_state())
            self.m_vehicle_model.set_steering_cmd(self.m_current_command.get_steering_command())
            self.m_vehicle_model.set_velocity_cmd(self.m_current_command.get_velocity_command())
            if cmd_complete:
                self.m_current_command = None
        else:
            self.m_vehicle_model.set_steering_cmd(0.0)
            self.m_vehicle_model.set_velocity_cmd(0.0)

        # Update Vehicle
        # self.state.V += dt * self.state.acceleration  # Update velocity
        # self.state.x += dt * self.state.V * np.cos(self.state.psi)
        # self.state.y += dt * self.state.V * np.sin(self.state.psi)
        # self.state.psi += dt * self.state.yaw_rate
        
        #update the vehicle model
        self.m_vehicle_model.update(dt)
        self.state = self.m_vehicle_model.get_vehicle_state()
        #print(f"Car.update: time={time}, dt={dt}, velocity={self.state.V}")
        return True

    def add_vehicle_command(self, cmd):
        if cmd is not None:
            self.m_vehicle_commands.append(cmd)

    # def render(self, disp):
    #     steeringPsi = self.m_vehicle_model.get_vehicle_state().steering
    #     carPsiOffset = self.m_vehicle_model.get_vehicle_state().psi
    #     carPosOffset = Vector2(self.m_vehicle_model.get_vehicle_state().x, self.m_vehicle_model.get_vehicle_state().y)
        
    #     disp.set_draw_colour(0, 255, 0)
    #     disp.draw_lines(transform_points(self.m_car_lines_body, carPosOffset, carPsiOffset))
    #     flat_marker_lines = [point for sublist in self.m_marker_lines for point in sublist]
    #     disp.draw_lines(transform_points(flat_marker_lines, carPosOffset, carPsiOffset))

    #     disp.set_draw_colour(0, 201, 0)
    #     disp.draw_lines(transform_points(transform_points(self.m_wheel_lines, self.m_wheel_fl_offset, steeringPsi), carPosOffset, carPsiOffset))
    #     disp.draw_lines(transform_points(transform_points(self.m_wheel_lines, self.m_wheel_fr_offset, steeringPsi), carPosOffset, carPsiOffset))
    #     disp.draw_lines(transform_points(offset_points(self.m_wheel_lines, self.m_wheel_rl_offset), carPosOffset, carPsiOffset))
    #     disp.draw_lines(transform_points(offset_points(self.m_wheel_lines, self.m_wheel_rr_offset), carPosOffset, carPsiOffset))
    def render(self, disp):
            steeringPsi = self.m_vehicle_model.get_vehicle_state().steering
            carPsiOffset = self.m_vehicle_model.get_vehicle_state().psi
            carPosOffset = Vector2(self.m_vehicle_model.get_vehicle_state().x, self.m_vehicle_model.get_vehicle_state().y)
            
            scale_factor = 5  # set to 5 for better visualization

            disp.set_draw_colour(0, 255, 0)
            disp.draw_lines(transform_points(self.m_car_lines_body, carPosOffset, carPsiOffset, scale_factor))
            flat_marker_lines = [point for sublist in self.m_marker_lines for point in sublist]
            disp.draw_lines(transform_points(flat_marker_lines, carPosOffset, carPsiOffset, scale_factor))

            disp.set_draw_colour(0, 201, 0)
            disp.draw_lines(transform_points(transform_points(self.m_wheel_lines, self.m_wheel_fl_offset, steeringPsi, scale_factor), carPosOffset, carPsiOffset, scale_factor))
            disp.draw_lines(transform_points(transform_points(self.m_wheel_lines, self.m_wheel_fr_offset, steeringPsi, scale_factor), carPosOffset, carPsiOffset, scale_factor))
            disp.draw_lines(transform_points(offset_points(self.m_wheel_lines, self.m_wheel_rl_offset, scale_factor), carPosOffset, carPsiOffset, scale_factor))
            disp.draw_lines(transform_points(offset_points(self.m_wheel_lines, self.m_wheel_rr_offset, scale_factor), carPosOffset, carPsiOffset, scale_factor))
    # def render(self, disp):
    #     car_state = self.m_vehicle_model.get_vehicle_state()
    #     car_position = Vector2(car_state.x, car_state.y)
    #     car_size = 10  # 设置小车的大小
    #     disp.set_draw_colour(0, 255, 0)
    #     # 调用 display.py 中的 draw_car 函数
    #     disp.draw_car(car_position, car_size)
# def transform_points(points, position, rotation):
#     transformed_points = []
#     cos_r = np.cos(rotation)
#     sin_r = np.sin(rotation)
#     for point in points:
#         x_new = cos_r * point.x - sin_r * point.y + position.x
#         y_new = sin_r * point.x + cos_r * point.y + position.y
#         transformed_points.append(Vector2(x_new, y_new))
#     return transformed_points

# def offset_points(points, offset):
#     return [Vector2(point.x + offset.x, point.y + offset.y) for point in points]
def transform_points(points, offset, angle, scale_factor):
    transformed_points = []
    for point in points:
        x = point.x * scale_factor
        y = point.y * scale_factor
        # Rotate and translate the points
        new_x = x * np.cos(angle) - y * np.sin(angle) + offset.x
        new_y = x * np.sin(angle) + y * np.cos(angle) + offset.y
        transformed_points.append(Vector2(new_x, new_y))
    return transformed_points

# def offset_points(points, offset, scale_factor):
#     return [Vector2(point.x * scale_factor + offset.x, point.y * scale_factor + offset.y) for point in points]
def offset_points(points, offset, scale_factor):
    offset_points = []
    for point in points:
        # Apply scaling
        scaled_x = point.x * scale_factor
        scaled_y = point.y * scale_factor
        
        # Apply offset
        offset_x = scaled_x + offset.x
        offset_y = scaled_y + offset.y
        
        offset_points.append(Vector2(offset_x, offset_y))
    return offset_points

class Camera:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.x = 0
        self.y = 0

    def update(self, target_x, target_y):
        self.x = target_x - self.width // 2
        self.y = target_y - self.height // 2

    def apply(self, entity):
        return entity.move(-self.x, -self.y)