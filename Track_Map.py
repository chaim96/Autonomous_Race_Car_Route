import matplotlib.pyplot as plt
import numpy as np

class MapWithObstacles:
    def __init__(self):
        self.MAX_SIZE = 19
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.grid = [[0] * width for _ in range(height)]  # Initialize grid with all zeros

    def add_obstacle(self, x, y, adjacent = False):
        if 0 <= x <= self.width and 0 <= y <= self.height:
            self.grid[y][x] = 1  # Set obstacle at position (x, y)
            if (adjacent):
                self._mark_adjacent_obstacles(x, y)

    def create_track1(self):
        start = (1, 1)
        end = (8, 3)
        plt.annotate("start", (start[0], start[1]))
        plt.annotate("end", (end[0], end[1]))
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

        return start, end

    def create_track2(self):
        start = (1, 1)
        end = (16, 16)
        plt.annotate("start", (start[0], start[1]))
        plt.annotate("end", (end[0], end[1]))
        for x in range(self.width):
            self.add_obstacle(x, 0)
            self.add_obstacle(x, 19)
        for y in range(self.height):
            self.add_obstacle(0, y)
            self.add_obstacle(19, y)
        self.add_obstacle(8, 5, True)
        self.add_obstacle(2, 6, True)
        #self.add_obstacle(14, 9, True)
        self.add_obstacle(13, 5, True)
        self.add_obstacle(7, 12, True)
        #self.add_obstacle(10, 16, True)
        return start, end


    def create_track3(self):
        start = (1, 1)
        end = (16, 16)
        plt.annotate("start", (start[0], start[1]))
        plt.annotate("end", (end[0], end[1]))
        for x in range(self.width):
            self.add_obstacle(x, 0)
            self.add_obstacle(x, 19)
        for y in range(self.height):
            self.add_obstacle(0, y)
            self.add_obstacle(19, y)
        for x in range(1, 15):
            self.add_obstacle(x, 5, False)
        for x in range(5, 20):
            self.add_obstacle(x, 12, False)

        return start, end


    def create_track4(self):
        start = (1, 1)
        end = (20, 16)
        plt.annotate("start", (start[0], start[1]))
        plt.annotate("end", (end[0], end[1]))
        # create borders
        for x in range(self.width):
            self.add_obstacle(x, 0)
            self.add_obstacle(x, 19)
        for y in range(self.height):
            self.add_obstacle(0, y)
            self.add_obstacle(39, y)
        # create Slalum
        for y in range(1, 15):
            self.add_obstacle(7, y, False)
        for y in range(5, 20):
            self.add_obstacle(14, y, False)
        # create square
        for x in range(20, 30):
            for y in range(5, 14):
                self.add_obstacle(x, y, False)


        return start, end


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
    map_with_obstacles = MapWithObstacles(20, 20)
    start, end = map_with_obstacles.create_track2()
    return map_with_obstacles, start, end


