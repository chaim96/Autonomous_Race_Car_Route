import math
import random

# Class for graph in which we will find a path from start to goal
class Graph:
    def __init__(self, start, goal, map_dimensions, obstacles):
        # graph attributes
        self.start = start
        self.goal = goal
        self.success = False #if succes we have a path between start and end points
        self.vertexes = [start]
        #self.parent_vertexes = [0]
        self.edges = []
        self.path = []
        self.goal_radius = 15

        # necessary map attributes
        self.map_height = map_dimensions[0]
        self.map_width = map_dimensions[1]
        self.obstacles = obstacles

    # adding & removing vertexes and edges
    def add_vertex(self, vertex, parent_index):
        self.vertexes.append((vertex[0], vertex[1], parent_index))

    def add_edge(self, parent, child):
        self.edges.append((child, parent))

    # returns number of vertexes in the graph
    def get_num_of_vertexes(self):
        return len(self.vertexes)

    # calculates distance between 2 vertexes
    def calc_distance(self, a, b):
        (x1, y1) = (a[0], a[1])
        (x2, y2) = (b[0], b[1])
        px = (float(x1) - float(x2)) ** 2
        py = (float(y1) - float(y2)) ** 2
        distance = (px + py) ** (0.5)
        return distance

    # finds out if there is a collision between a point and an obstacle
    def is_there_obstacle_collision(self, vertex):
        for obstacle in self.obstacles:
            if obstacle.collidepoint(vertex):
                return True
        return False

    # determines if a line between a new vertex and an existing vertex collides with an obstacle
    def cross_obstacle(self, vertex1, vertex2):
        for obstacle in self.obstacles:
            for i in range(0, 101):
                u = i / 100
                x = vertex1[0] * u + vertex2[0] * (1 - u)
                y = vertex1[1] * u + vertex2[1] * (1 - u)
                if obstacle.collidepoint(x, y):
                    return True
        return False

    # samples and returns a random located point in the map
    def create_random_sample(self):
        x = int(random.uniform(0, self.map_width))
        y = int(random.uniform(0, self.map_height))
        vertex = (x, y)
        while self.is_there_obstacle_collision(vertex):
            x = int(random.uniform(0, self.map_width))
            y = int(random.uniform(0, self.map_height))
            vertex = (x, y)
        return vertex

    # adds new vartex to the graph and connects between him, and it's parent
    def connect_vertexes(self, parent_vertex, parent_index, child_vertex):
        if self.cross_obstacle(parent_vertex, child_vertex):
            return False
        else:
            self.add_vertex(child_vertex, parent_index)
            self.add_edge(parent_vertex, child_vertex)
            return True

    # finds the nearest existing vertex to the sample
    def find_vertex_nearest_to_sample(self, sample_vertex):
        min_distance = self.calc_distance(self.vertexes[0], sample_vertex)
        nearest_vertex_to_rand_sample = self.vertexes[0]
        index = 0
        nearest_vertex_to_rand_sample_index = 0
        for vertex in self.vertexes:
            distance = self.calc_distance(vertex, sample_vertex)
            if distance < min_distance:
                min_distance = distance
                nearest_vertex_to_rand_sample = vertex
                nearest_vertex_to_rand_sample_index = index
            index += 1
        return nearest_vertex_to_rand_sample, nearest_vertex_to_rand_sample_index

    # creates a new vertex on the line between the random sample and the nearest vertex to it
    def create_next_vertex_between_sample_and_nearest_vertex(self, nearest_vertex_to_sample, sample, parent_index, max_distance=35):
        distance = self.calc_distance(nearest_vertex_to_sample, sample)
        if distance > max_distance:
            # create the next vertex, located max_distance away from the nearest vertex to random sample
            (xnear, ynear) = (nearest_vertex_to_sample[0], nearest_vertex_to_sample[1])
            (xrand, yrand) = (sample[0], sample[1])
            theta = math.atan2(yrand - ynear, xrand - xnear)
            (x, y) = (int(xnear + max_distance * math.cos(theta)), int(ynear + max_distance * math.sin(theta)))
            # check if new vertex is in the goal radius - means we reached our goal!
            if self.calc_distance(self.goal, (x, y)) < self.goal_radius:
                self.add_vertex(self.goal, parent_index)
                self.success = True
                return self.goal
            else:
                return (x, y)
        else:
            if self.calc_distance(self.goal, sample) < self.goal_radius:
                self.add_vertex(self.goal, parent_index)
                self.success = True
            return sample

    # creates a new vertex in graph towards the goal
    def bias_expanding(self):
        nearest_vertex, nearest_vertex_index = self.find_vertex_nearest_to_sample(self.goal)
        next_vertex = self.create_next_vertex_between_sample_and_nearest_vertex(nearest_vertex, self.goal, nearest_vertex_index)
        result = self.connect_vertexes(nearest_vertex, nearest_vertex_index, next_vertex)
        return result, (next_vertex[0], next_vertex[1]), (nearest_vertex[0], nearest_vertex[1])

    # creates a new vertex in graph towards a random sample
    def random_expanding(self):
        random_sample = self.create_random_sample()
        nearest_vertex, nearest_vertex_index = self.find_vertex_nearest_to_sample(random_sample)
        next_vertex = self.create_next_vertex_between_sample_and_nearest_vertex(nearest_vertex, random_sample, nearest_vertex_index)
        #result is true id connecting vertexes work
        result = self.connect_vertexes(nearest_vertex, nearest_vertex_index, next_vertex)
        return result, random_sample, (next_vertex[0], next_vertex[1]), (nearest_vertex[0], nearest_vertex[1])


    #draw the path in red
    def get_path_to_goal(self):
        if self.success:
            self.path = []
            vertex = self.vertexes[-1]
            self.path.append((vertex[0], vertex[1]))
            parent_index = vertex[2]
            while parent_index != 0:
                vertex = self.vertexes[parent_index]
                self.path.append((vertex[0], vertex[1]))
                parent_index = vertex[2]
        return self.path