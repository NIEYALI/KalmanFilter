# -*- encoding: utf-8 -*-
'''
@File    :   display.py
@Time    :   2024/11/22 12:46:28
@Author  :   Yali Nie 
@Version :   1.0
@Contact :   yalinie2015@gmail.com
'''

# import your libs here

import pygame
from pygame.locals import *

class Vector2:
    def __init__(self, x=0.0, y=0.0):
        self.x = x
        self.y = y

    def __repr__(self):
        return f"Vector2({self.x}, {self.y})"

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

    def set_draw_colour(self, red, green, blue, alpha=0xFF):
        self.draw_colour = (red, green, blue, alpha)

    def draw_line(self, start_pos, end_pos):
        if isinstance(start_pos, list):
            start_pos = Vector2(start_pos[0], start_pos[1])
        if isinstance(end_pos, list):
            end_pos = Vector2(end_pos[0], end_pos[1])
        #print(f"Drawing line from ({start_pos.x}, {start_pos.y}) to ({end_pos.x}, {end_pos.y})")
        pygame.draw.line(self.renderer, self.draw_colour, (start_pos.x, start_pos.y), (end_pos.x, end_pos.y))

    def draw_lines(self, points):
        for i in range(1, len(points)):
            self.draw_line(points[i-1], points[i])

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

    def set_view(self, width, height, center_x, center_y):
        self.view_width = width
        self.view_height = height
        self.view_x_offset = center_x - width / 2
        self.view_y_offset = center_y - height / 2

    def get_screen_aspect_ratio(self):
        return self.screen_width / self.screen_height

    def draw_grid(self, grid_spacing):
        self.set_draw_colour(200, 200, 200)
        for x in range(-self.screen_width, self.screen_width * 2, grid_spacing):
            self.draw_line(Vector2(x, -self.screen_height), Vector2(x, self.screen_height * 2))
        for y in range(-self.screen_height, self.screen_height * 2, grid_spacing):
            self.draw_line(Vector2(-self.screen_width, y), Vector2(self.screen_width * 2, y))

