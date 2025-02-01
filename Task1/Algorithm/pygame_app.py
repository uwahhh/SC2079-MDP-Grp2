from typing import List
from abc import ABC, abstractmethod

import pygame

import constants
import misc.timer as timer
from misc.direction import Direction
from grid.grid import Grid
from grid.obstacle import Obstacle
from robot.robot import Robot
from misc.positioning import Position
from simulation import Simulation


class AlgoApp(ABC):
    def __init__(self, obstacles: List[Obstacle]):
        self.grid = Grid(obstacles)
        self.robot = Robot(self.grid)
        self.direction = Direction.TOP
        # self.simulation = Simulation()
        self.obstacles = obstacles
        self.index = 0

    @abstractmethod
    def init(self):
        pass

    @abstractmethod
    def execute(self):
        self.init()  # Initialize Pygame properly

        while self.running:
            self.settle_events()  # Process mouse clicks and other events
            self.render()  # Refresh the display
            self.clock.tick(30)  # Control FPS to prevent freezing

class AlgoMinimal(AlgoApp):
    """
    Minimal app to just calculate a path and then send the commands over.
    """

    def __init__(self, obstacles):
        # We run it as a server.
        super().__init__(obstacles)
        self.running = True  # Ensure the app keeps running

    def init(self):
        pygame.init()
        self.running = True
        self.size = self.width, self.height = constants.WINDOW_SIZE
        self.screen = pygame.display.set_mode(self.size, pygame.HWSURFACE | pygame.DOUBLEBUF)
        self.clock = pygame.time.Clock()


    def execute(self):
        """
        Start the interactive Pygame window and listen for user input.
        """
        self.init()  # Initialize Pygame properly

        while self.running:
            self.settle_events()  # Process mouse clicks and other events
            self.render()  # Refresh the display
            self.clock.tick(30)  # Control FPS to prevent freezing


    def settle_events(self):
        """
        Process Pygame events, including obstacle rotation on click.
        """
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False  # Exit if user closes the window

            elif event.type == pygame.MOUSEBUTTONDOWN:
                pos = pygame.mouse.get_pos()
                x = pos[0] // constants.GRID_CELL_LENGTH * 10 + 5
                y = abs(pos[1] // constants.GRID_CELL_LENGTH - 19) * 10 + 5
                print("Click detected at", pos, "Grid coordinates:", x, y)

                # Check if click is on an obstacle
                for obstacle in self.obstacles:
                    if obstacle.position.x == x and obstacle.position.y == y:
                        # Rotate the obstacle's direction
                        current_direction = obstacle.position.direction.value
                        new_direction = (current_direction + 90) % 360  # Rotate clockwise
                        obstacle.position.direction = Direction(new_direction)
                        print(f"Obstacle at ({x},{y}) rotated to {obstacle.position.direction}")
                        break  # Stop checking once we find and rotate an obstacle

                # Refresh the display after rotating
                self.render()

    def simulate(self):
        # Calculate path
        self.simulation.runSimulation(self.robot)

    def render(self):
        """
        Render the screen.
        """
        self.screen.fill(constants.WHITE, None)
        self.grid.draw(self.screen)
        self.robot.draw(self.screen)
        for obstacle in self.obstacles:    # Draw obstacles
            obstacle.draw(self.screen)
        # Really render now.
        pygame.display.flip()