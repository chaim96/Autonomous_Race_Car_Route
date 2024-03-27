import matplotlib.pyplot as plt
import numpy as np

class MapWithObstacles:
    def __init__(self):
        self.MAX_SIZE = 19
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.grid = [[0] * width for _ in range(height)]  # Initialize grid with all zeros

    def add_obstacle(self, x, y):
        if 0 <= x < self.width and 0 <= y < self.height:
            self.grid[y][x] = 1  # Set obstacle at position (x, y)

    def create_track1(self):
        for x in range(self.width):
            self.add_obstacle(x, 0)
            self.add_obstacle(x, 19)
        for y in range(self.height):
            self.add_obstacle(0, y)
            self.add_obstacle(19, y)

        for x in range(6, 14):
            for y in range(6, 14):
                self.add_obstacle(x, y)

        for y in range(6):
            self.add_obstacle(6, y)

    def create_track2(self):
        # new track


    def is_obstacle(self, x, y):
        if 0 <= x < self.width and 0 <= y < self.height:
            return self.grid[y][x] == 1
        return False

    def display(self):
        plt.imshow(self.grid, cmap='binary', origin='lower')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.show()

    def print_matrix(self):
        for row in self.grid:
            print(" ".join(map(str, row)))

    def _mark_adjacent_obstacles(self, x, y):
        # Mark left, right, up, down cells as obstacles if they are within bounds
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nx, ny = x + dx, y + dy
            if 0 <= nx < self.width and 0 <= ny < self.height:
                self.grid[ny][nx] = 1

def create_map():
    map_with_obstacles = MapWithObstacles(20, 20)  # Create a 10x10 map
    map_with_obstacles.create_track()
    return map_with_obstacles


