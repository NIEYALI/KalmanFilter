# -*- encoding: utf-8 -*-
'''
@File    :   main.py
@Time    :   2024/11/18 14:24:28
@Author  :   Yali Nie 
@Version :   1.0
@Contact :   yalinie2015@gmail.com
'''

# import your libs here
import os
import sys
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
import numpy as np
import pygame
import math
from utils.display import Display
from utils.car import MotionCommandMoveTo, MotionCommandStraight, MotionCommandTurnTo, MotionCommandStraight

from simulation import Simulation, SimulationParams

# Screen dimension constants
SCREEN_WIDTH = 1024
SCREEN_HEIGHT = 768
GRID_SIZE = 500
GRID_SPACING = 25

# Function Prototypes
def load_simulation1_parameters():
    sim_params = SimulationParams()
    sim_params.profile_name = "1 - Constant Velocity + GPS + GYRO + Zero Initial Conditions"
    sim_params.car_initial_velocity = 5
    sim_params.car_initial_psi = math.pi / 180.0 * 45.0
    sim_params.car_commands.append(MotionCommandMoveTo(924, 668, 5))
    sim_params.end_time = 210
    return sim_params

def load_simulation2_parameters():
    sim_params = SimulationParams()
    sim_params.profile_name = "2 - Constant Velocity + GPS + GYRO + Non-zero Initial Conditions"
    sim_params.car_initial_x = 500
    sim_params.car_initial_y = 500
    sim_params.car_initial_velocity = 6
    sim_params.car_initial_psi = math.pi / 180.0 * -135.0
    sim_params.car_commands.append(MotionCommandMoveTo(0, 0, 5))
    return sim_params

def load_simulation3_parameters():
    sim_params = SimulationParams()
    sim_params.profile_name = "3 - Constant Speed Profile + GPS + GYRO"
    sim_params.car_initial_velocity = 5
    sim_params.car_initial_psi = math.pi / 180.0 * 45.0
    sim_params.car_commands.append(MotionCommandMoveTo(100, 100, 5))
    sim_params.car_commands.append(MotionCommandMoveTo(100, -100, 5))
    sim_params.car_commands.append(MotionCommandMoveTo(0, 100, 5))
    sim_params.car_commands.append(MotionCommandMoveTo(0, 0, 5))
    return sim_params

def load_simulation4_parameters():
    sim_params = SimulationParams()
    sim_params.profile_name = "4 - Variable Speed Profile + GPS + GYRO"
    sim_params.end_time = 200
    sim_params.car_initial_velocity = 0
    sim_params.car_initial_psi = math.pi / 180.0 * 45.0
    sim_params.car_commands.append(MotionCommandMoveTo(100, 100, 2))
    sim_params.car_commands.append(MotionCommandMoveTo(100, -100, 5))
    sim_params.car_commands.append(MotionCommandMoveTo(0, 100, 7))
    sim_params.car_commands.append(MotionCommandMoveTo(0, 0, 2))
    return sim_params

def load_simulation5_parameters():
    sim_params = load_simulation1_parameters()
    sim_params.profile_name = "5 - Constant Velocity + GPS + GYRO + LIDAR+ Zero Initial Conditions"
    sim_params.lidar_enabled = True
    sim_params.lidar_range_noise_std = 0.1
    sim_params.lidar_theta_noise_std = 0.01
    sim_params.lidar_id_enabled = True
    return sim_params

def load_simulation6_parameters():
    sim_params = load_simulation2_parameters()
    sim_params.profile_name = "6 - Constant Velocity + GPS + GYRO + LIDAR + Non-zero Initial Conditions"
    sim_params.lidar_enabled = True
    return sim_params

def load_simulation7_parameters():
    sim_params = load_simulation3_parameters()
    sim_params.profile_name = "7 - Constant Speed Profile + GPS + GYRO + LIDAR"
    sim_params.lidar_enabled = True
    return sim_params

def load_simulation8_parameters():
    sim_params = load_simulation4_parameters()
    sim_params.profile_name = "8 - Variable Speed Profile + GPS + GYRO + LIDAR"
    sim_params.lidar_enabled = True
    return sim_params

def load_simulation9_parameters():
    sim_params = SimulationParams()
    sim_params.profile_name = "9 - CAPSTONE"
    sim_params.gyro_enabled = True
    sim_params.lidar_enabled = True
    sim_params.end_time = 170
    sim_params.car_initial_x = SCREEN_WIDTH -100
    sim_params.car_initial_y = SCREEN_HEIGHT -100
    sim_params.car_initial_velocity = 0
    sim_params.car_initial_psi = math.pi / 180.0 * -90.0
    sim_params.gps_error_probability = 0.05
    sim_params.gps_denied_x = 250.0
    sim_params.gps_denied_y = -250.0
    sim_params.gps_denied_range = 100.0
    sim_params.gyro_bias = -3.1 / 180.0 * math.pi
    sim_params.car_commands.append(MotionCommandStraight(3, -2))
    sim_params.car_commands.append(MotionCommandTurnTo(math.pi / 180.0 * 90.0, -2))
    sim_params.car_commands.append(MotionCommandMoveTo(400, -300, 5))
    sim_params.car_commands.append(MotionCommandMoveTo(350, -300, 2))
    sim_params.car_commands.append(MotionCommandMoveTo(300, -250, 7))
    sim_params.car_commands.append(MotionCommandMoveTo(300, -300, 5))
    sim_params.car_commands.append(MotionCommandMoveTo(250, -250, 5))
    sim_params.car_commands.append(MotionCommandMoveTo(250, -300, 5))
    sim_params.car_commands.append(MotionCommandMoveTo(200, -250, 5))
    sim_params.car_commands.append(MotionCommandMoveTo(200, -300, 5))
    sim_params.car_commands.append(MotionCommandMoveTo(200, -150, 2))
    sim_params.car_commands.append(MotionCommandMoveTo(100, -100, -2))
    sim_params.car_commands.append(MotionCommandMoveTo(200, 0, 7))
    sim_params.car_commands.append(MotionCommandMoveTo(300, -100, 5))
    sim_params.car_commands.append(MotionCommandMoveTo(300, -300, 7))
    sim_params.car_commands.append(MotionCommandMoveTo(400, -300, 3))
    sim_params.car_commands.append(MotionCommandMoveTo(400, -400, 1))
    return sim_params

# Main Loop
def main():
    pygame.init()
    pygame.font.init()

    display = Display()
    sim_params = load_simulation1_parameters()
    simulation = Simulation(sim_params)

    # Initialize the Kalman filter with the initial state and covariance
    initial_state = np.array([0, 0, sim_params.car_initial_psi, sim_params.car_initial_velocity])  # Example initial state
    initial_covariance = np.eye(4)  # Example initial covariance
    simulation.m_kalman_filter.initialize(initial_state, initial_covariance)

    # Start Graphics
    if not display.create_renderer("KalmanF Simulations", SCREEN_WIDTH, SCREEN_HEIGHT):
        return -1

    # Initialize Camera
    # camera = Camera(SCREEN_WIDTH, SCREEN_HEIGHT)
    
    # Main Simulation Loop
    simulation.reset()
    running = True
    clock = pygame.time.Clock()
    while running:
        # Update Simulation
        simulation.update()

        # Update Display
        display.clear_screen()

        # Update Camera
        # car_state = simulation.m_vehicle_model.get_vehicle_state()
        # camera.update(car_state.x, car_state.y)

        # Draw Background Grid
        #display.set_draw_colour(101, 101, 101)
        #display.draw_grid(25)
        # for x in range(-GRID_SIZE, GRID_SIZE + 1, GRID_SPACING):
        #     display.draw_line(Vector2(x, -GRID_SIZE), Vector2(x, GRID_SIZE))
        # for y in range(-GRID_SIZE, GRID_SIZE + 1, GRID_SPACING):
        #     display.draw_line(Vector2(-GRID_SIZE, y), Vector2(GRID_SIZE, y))

        # Draw Simulation
        simulation.render(display)

        display.show_screen()


        # Handle Events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    simulation.toggle_pause_simulation()
                elif event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_UP:
                    simulation.increase_zoom()
                elif event.key == pygame.K_DOWN:
                    simulation.decrease_zoom()
                elif event.key == pygame.K_RIGHT:
                    simulation.increase_time_multiplier()
                elif event.key == pygame.K_LEFT:
                    simulation.decrease_time_multiplier()
                elif event.key == pygame.K_r:
                    simulation.reset()
                elif event.key == pygame.K_1:
                    print("Resetting to simulation 1 parameters")
                    simulation.reset(load_simulation1_parameters())
                elif event.key == pygame.K_2:
                    simulation.reset(load_simulation2_parameters())
                elif event.key == pygame.K_3:
                    simulation.reset(load_simulation3_parameters())
                elif event.key == pygame.K_4:
                    simulation.reset(load_simulation4_parameters())
                elif event.key == pygame.K_5:
                    simulation.reset(load_simulation5_parameters())
                elif event.key == pygame.K_6:
                    simulation.reset(load_simulation6_parameters())
                elif event.key == pygame.K_7:
                    simulation.reset(load_simulation7_parameters())
                elif event.key == pygame.K_8:
                    simulation.reset(load_simulation8_parameters())
                elif event.key == pygame.K_9:
                    simulation.reset(load_simulation9_parameters())
                elif event.key == pygame.K_0:
                    simulation.reset(load_simulation1_parameters())

        clock.tick(60)

    # Destroy Renderer
    display.destroy_renderer()

    # Unload SDL
    pygame.quit()

if __name__ == "__main__":
    main()