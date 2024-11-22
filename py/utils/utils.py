# -*- encoding: utf-8 -*-
'''
@File    :   utils.py
@Time    :   2024/11/18 12:26:44
@Author  :   Yali Nie 
@Version :   1.0
@Contact :   yalinie2015@gmail.com
'''

# import your libs here
import numpy as np
import pygame
from pygame.locals import *

def wrap_angle(angle):
    angle = np.mod(angle, 2.0 * np.pi)
    if angle <= -np.pi:
        angle += 2.0 * np.pi
    elif angle > np.pi:
        angle -= 2.0 * np.pi
    return angle

def calculate_mean(dataset):
    if len(dataset) == 0:
        return np.nan
    return np.mean(dataset)

def calculate_rmse(dataset):
    if len(dataset) == 0:
        return 0.0
    return np.sqrt(np.mean(np.square(dataset)))

def generate_ellipse(x, y, sigma_xx, sigma_yy, sigma_xy, num_points=50):
    pos_cov = np.array([[sigma_xx, sigma_xy], [sigma_xy, sigma_yy]])
    U, s, _ = np.linalg.svd(pos_cov)
    D = U @ np.diag(3.0 * np.sqrt(s))
    
    theta = np.linspace(0, 2 * np.pi, num_points)
    ellipse = np.array([np.cos(theta), np.sin(theta)])
    A = D @ ellipse
    
    shape_body = np.column_stack((A[0, :], A[1, :]))
    shape_world = shape_body + np.array([x, y])
    
    return shape_world.tolist()

def generate_circle(x, y, radius, num_points=50):
    theta = np.linspace(0, 2 * np.pi, num_points)
    circle = np.column_stack((x + radius * np.cos(theta), y + radius * np.sin(theta)))
    return circle.tolist()

# def string_format(format_string, *args):
#     return format_string.format(*args)

def string_format(format_string, *args):
    return format_string % args

class Vector2:
    def __init__(self, x=0.0, y=0.0):
        self.x = x
        self.y = y

    def __repr__(self):
        return f"Vector2({self.x}, {self.y})"

def transform_points(points, position, rotation):
    transformed_points = []
    cos_r = np.cos(rotation)
    sin_r = np.sin(rotation)
    for point in points:
        x_new = cos_r * point.x - sin_r * point.y + position.x
        y_new = sin_r * point.x + cos_r * point.y + position.y
        transformed_points.append(Vector2(x_new, y_new))
    return transformed_points

def flatten_points(points):
    """Flatten a list of points, handling nested lists."""
    flat_points = []
    for point in points:
        if isinstance(point, list):
            flat_points.extend(flatten_points(point))
        else:
            flat_points.append(point)
    return flat_points

def offset_points(points, offset):
    #print(f"offset_points called with offset={offset}, type={type(offset)}")
    if isinstance(offset, list):
        offset = Vector2(offset[0], offset[1])
    elif not isinstance(offset, Vector2):
        offset = Vector2(offset, offset)
    #print(f"offset converted to Vector2: {offset}")

    points = flatten_points(points)
    corrected_points = []
    for point in points:
        #print(f"Processing point={point}, type={type(point)}")
        if isinstance(point, list):
            point = Vector2(point[0], point[1])
        corrected_points.append(Vector2(point.x + offset.x, point.y + offset.y))
    return corrected_points

class Display:
    def __init__(self):
        self.screen_width = 0
        self.screen_height = 0
        self.view_width = 0
        self.view_height = 0
        self.view_x_offset = 0
        self.view_y_offset = 0
        self.window = None
        self.renderer = None
        self.main_font = None

    def create_renderer(self, title, screen_width, screen_height):
        pygame.init()
        pygame.font.init()
        self.screen_width = screen_width
        self.screen_height = screen_height
        self.view_width = screen_width
        self.view_height = screen_height
        self.view_x_offset = 0.0
        self.view_y_offset = 0.0
        self.window = pygame.display.set_mode((screen_width, screen_height))
        pygame.display.set_caption(title)
        self.renderer = pygame.display.get_surface()
        self.main_font = pygame.font.Font("Roboto-Regular.ttf", 18)
        return True

    def destroy_renderer(self):
        pygame.quit()

    def show_screen(self):
        pygame.display.flip()

    def clear_screen(self):
        self.renderer.fill((0, 0, 0))

    def set_view(self, width, height, x_offset, y_offset):
        self.view_width = abs(width)
        self.view_height = abs(height)
        self.view_x_offset = x_offset - self.view_height / 2.0
        self.view_y_offset = y_offset - self.view_width / 2.0

    def set_view(self, x_offset, y_offset):
        self.view_x_offset = x_offset - self.view_height / 2.0
        self.view_y_offset = y_offset - self.view_width / 2.0

    def set_draw_colour(self, red, green, blue, alpha=0xFF):
        self.draw_colour = (red, green, blue, alpha)

    def draw_line(self, start_pos, end_pos):
        p1 = self.transform_point(start_pos)
        p2 = self.transform_point(end_pos)
        pygame.draw.line(self.renderer, self.draw_colour, (p1.x, p1.y), (p2.x, p2.y))

    def draw_lines(self, points):
        for i in range(1, len(points)):
            self.draw_line(points[i-1], points[i])

    def draw_lines(self, dataset):
        for points in dataset:
            self.draw_lines(points)

    def transform_point(self, point):
        dx = point.x - self.view_x_offset
        dy = point.y - self.view_y_offset
        y = self.screen_height - (dx / self.view_height) * self.screen_height
        x = (dy / self.view_width) * self.screen_width
        return Vector2(x, y)

    def draw_text_main_font(self, text, pos, scale=1, color=(0, 0, 0), centered=False):
        surface = self.main_font.render(text, True, color)
        width = surface.get_width() * scale
        height = surface.get_height() * scale
        x = pos.x
        y = pos.y
        if centered:
            x -= width / 2
            y -= height / 2
        self.renderer.blit(pygame.transform.scale(surface, (width, height)), (x, y))