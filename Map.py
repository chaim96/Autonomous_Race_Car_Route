import math
import random


# here we'll implement the map for the RRT simulation

class Map:
    def __init__(self, start, goal, map_dimensions, obstacle_dimensions, obstacles_num):
        # map settings
        self.start = start
        self.goal = goal
        self.map_dimensions = map_dimensions
        self.map_height, self.map_width = map_dimensions

        # window settings
        self.map_windows_name = 'RRT Simulation'
        pygame.display.set_caption(self.map_windows_name)
        self.map = pygame.display.set_mode((self.map_width, self.map_height))
        self.map.fill((255, 255, 255))
        self.node_thickness = 0
        self.vertex_thickness = 5
        self.edge_thickness = 3

        # obstacles
        self.obstacles = []
        self.obstacle_dimensions = obstacle_dimensions
        self.obstacles_num = obstacles_num

        # colors
        self.grey = (70, 70, 70)
        self.blue = (0, 0, 255)
        self.green = (0, 255, 0)
        self.red = (255, 0, 0)
        self.white = (255, 255, 255)

    def get_square_corner(self):
        x_upper_left_corner = int(random.uniform(0, self.map_width - self.obstacle_dimensions))
        y_upper_left_corner = int(random.uniform(0, self.map_height - self.obstacle_dimensions))
        return (x_upper_left_corner, y_upper_left_corner)

    def make_obstacles(self):
        for i in range(0, self.obstacles_num):
            collision_with_start_goal = True
            while collision_with_start_goal:
                upper_left_corner = self.get_square_corner()
                square = pygame.Rect(upper_left_corner, (self.obstacle_dimensions, self.obstacle_dimensions))
                if square.collidepoint(self.start) or square.collidepoint(self.goal):
                    collision_with_start_goal = True
                else:
                    collision_with_start_goal = False
            self.obstacles.append(square)

    def draw_obstacles(self):
        self.make_obstacles()
        for obstacle in self.obstacles:
            pygame.draw.rect(self.map, self.grey, obstacle)

    def draw_map(self):
        pygame.draw.circle(self.map, self.green, self.start, 7)
        pygame.draw.circle(self.map, self.green, self.goal, 15)
        self.draw_obstacles()

    def draw_path(self, path):
        for vertex in path:
            pygame.draw.circle(self.map, self.red, vertex, self.vertex_thickness)