import math
import random
import Track_Map
import matplotlib.pyplot as plt
from dubins_path_planner import *

def random_control(closest_x, closest_y, closest_theta, random_x, random_y, velocity):
    wheelbase = 0.35
    alpha = math.atan2(random_y - closest_y, random_x - closest_x) - closest_theta
    lf = max((((random_x - closest_x) ** 2 + (random_y - closest_y) ** 2) ** 0.5), 0.01)
    steering = math.atan2(2.0 * wheelbase * math.sin(alpha) / lf, 1.0)
    theta_dot = velocity * np.tan(steering) / wheelbase
    dt = 0.07
    waypoints_x = []
    waypoints_y = []
    waypoints_theta = []
    waypoints_x.append(closest_x)
    waypoints_y.append(closest_y)
    waypoints_theta.append(closest_theta)
    reach_goal = False
    total_time = 0
    i = 0
    while not reach_goal:
        i = i+1
        print(i)
        total_time += dt
        closest_theta += theta_dot * dt
        x_dot = velocity * np.cos(closest_theta)
        y_dot = velocity * np.sin(closest_theta)
        closest_x += x_dot * dt
        closest_y += y_dot * dt
        waypoints_x.append(closest_x)
        waypoints_y.append(closest_y)
        waypoints_theta.append(closest_theta)
        if ((closest_x - random_x) ** 2 + (closest_y - random_y) ** 2) ** 0.5 < 0.1:
            reach_goal = True
    waypoints_x.append(random_x)
    waypoints_y.append(random_y)
    waypoints_theta.append(closest_theta)
    return [waypoints_x, waypoints_y, waypoints_theta], total_time

class Vertex:
    def __init__(self, x, y, theta=0.01, delta=0, v=1, cost=0, parent_index=0, ver_index=0, edge_way_points=None, scatter_temp=None):
        self.x = x
        self.y = y
        self.theta = theta
        self.delta = delta
        self.v = v
        self.cost = cost
        self.parent_index = parent_index
        self.ver_index = ver_index
        self.edge_way_points = edge_way_points
        self.scatter_temp = scatter_temp

class Graph:
    def __init__(self, start, end, v_change_range, delta_change_range, max_delta_per_v, bias_ratio):
        # graph attributes
        self.start = start
        self.end = end
        self.success = False  # if success we have a path between start and end points
        self.vertexes = [start]  # start that is passed as an argument must be a Vertex class object
        self.path = []
        self.end_radius = 1
        self.v_change_range = v_change_range  # a new v will be within [v_current - v_change_range, v_current + v_change_range]
        self.delta_change_range = delta_change_range  # a new delta will be within [delta_current - delta_change_range, delta_current + delta_change_range]
        self.max_delta_per_v = max_delta_per_v  # an array which contains the max delta possible for a given v
        self.bias_ratio = bias_ratio  # sets how often we expand by bias

    # adds new vertex
    def add_vertex(self, vertex):
        vertex.ver_index = len(self.vertexes)
        self.vertexes.append(vertex)

    # returns number of vertexes in the graph
    def get_num_of_vertexes(self):
        return len(self.vertexes)

    # calculates if new vertex is in end radius
    # returns True is yes
    def is_in_end_radius(self, vertex):
        px = (float(vertex.x) - float(self.end.x)) ** 2
        py = (float(vertex.y) - float(self.end.y)) ** 2
        distance = (px + py) ** (0.5)
        if distance <= self.end_radius:
            return True
        return False

    # returns distance of vertex to end vertex
    def calc_distance_to_end(self, vertex):
        px = (float(vertex.x) - float(self.end.x)) ** 2
        py = (float(vertex.y) - float(self.end.y)) ** 2
        distance = (px + py) ** (0.5)
        return distance

    # returns distance between 2 vertexes
    def calc_distance_between_vertexes(self, vertex1, vertex2):
        px = (float(vertex1.x) - float(vertex2.x)) ** 2
        py = (float(vertex1.y) - float(vertex2.y)) ** 2
        distance = (px + py) ** (0.5)
        return distance


    # calculates cost of target if its parent is source
    def calc_cost(self, source_vertex, target_vertex):
        waypoints, time = random_control(source_vertex.x, source_vertex.y, source_vertex.theta, target_vertex.x, target_vertex.y, source_vertex.v)
        target_cost = source_vertex.cost + time
        return waypoints[0], waypoints[1], target_cost

    # checks if it is possible to go from current vertex to target vertex
    def is_reachable(self, path_x, path_y, map_with_obstacles):
        for i in range(0, len(path_x)):
            x = round(path_x[i])
            y = round(path_y[i])
            if map_with_obstacles.is_obstacle(x, y):
                return False
        return True

    def get_random_point(self, map_with_obstacles):
        collide = True
        while collide:
            x = random.randint(0, map_with_obstacles.width)
            y = random.randint(0, map_with_obstacles.height)
            if not map_with_obstacles.is_obstacle(x, y):
                collide = False
        return x, y


    # get k nearest vertexes to the given vertex
    def get_k_nearest(self, x, y, k=2):
        point = Vertex(x, y)
        k_nearest = []
        k_distances = []
        for v in self.vertexes:
            if len(k_nearest) < k:
                k_nearest.append(v)
                k_distances.append(self.calc_distance_between_vertexes(point, v))
            else:
                max_index = k_distances.index(max(k_distances))
                current_distance = self.calc_distance_between_vertexes(point, v)
                if current_distance < k_distances[max_index]:
                    k_nearest[max_index] = v
                    k_distances[max_index] = current_distance
        return k_nearest


    def find_first_vertex_with_valid_edge_from_K_vertexes(self, random_x, random_y, map_with_obstacles):
        k_nearest_vertexes = self.get_k_nearest(random_x, random_y)
        for vertex in k_nearest_vertexes:
            waypoints, time = random_control(vertex.x, vertex.y, vertex.theta, random_x, random_y, vertex.v)
            if self.is_reachable(waypoints[0], waypoints[1], map_with_obstacles) and self.calc_distance_between_vertexes(vertex, self.end) > self.end_radius:
                return True, vertex, waypoints, time
        return False, 0, 0, 0

    def create_new_vertex(self, vertex, waypoints, time):
        new_vertex = Vertex(waypoints[0][-1], waypoints[1][-1], waypoints[2][-1], waypoints[2][-1], vertex.v,
                            time + vertex.cost, vertex.ver_index, self.get_num_of_vertexes(), waypoints)
        new_vertex.scatter_temp = self.draw_edge(new_vertex.edge_way_points[0], new_vertex.edge_way_points[1])
        return new_vertex


    # loop for k nearest vertexes and find the fastest path to new vertex from start point
    def wiring(self, new_vertex, map_with_obstacles):
        k_nearest = self.get_k_nearest(new_vertex.x, new_vertex.y)
        for index, current_vertex in enumerate(k_nearest):
            # if new vertex is reachable from current vertex -> check if new vertex cost is better
            path_x, path_y, new_cost = self.calc_cost(current_vertex, new_vertex)
            if self.is_reachable(path_x, path_y, map_with_obstacles):
                if new_cost < new_vertex.cost:
                    # current vertex will be the parent of new vertex in the tree
                    new_vertex.cost = new_cost
                    new_vertex.parent_index = k_nearest[index].ver_index
                    new_vertex.edge_way_points = (path_x, path_y)
                    print("########did wiring thing########")
                    new_vertex.scatter_temp.remove()
                    new_vertex.scatter_temp = self.draw_edge(path_x, path_y)
        return

    # loop for k nearest vertexes, for each one ask if the new vertex improves the path to existing vertex from start point
    def rewiring(self, new_vertex, map_with_obstacles):
        k_nearest = self.get_k_nearest(new_vertex.x, new_vertex.y)
        for current_vertex in k_nearest:
            # if current vertex is reachable from new vertex -> check if current vertex cost is better
            path_x, path_y, new_cost = self.calc_cost(new_vertex, current_vertex)
            if self.is_reachable(path_x, path_y, map_with_obstacles):
                if new_cost < current_vertex.cost:
                    # new vertex will be the parent of current vertex in the tree
                    current_vertex.cost = new_cost
                    current_vertex.parent_index = new_vertex.ver_index
                    current_vertex.edge_way_points = (path_x, path_y)
                    print("^^^^^^^^ did RE-wiring thing ^^^^^^^^")
                    current_vertex.scatter_temp.remove()
                    current_vertex.scatter_temp = self.draw_edge(path_x, path_y)
        return

    def draw_winning_path(self, start, new_vertex):
        cur_vertex = new_vertex
        while(cur_vertex is not start):
            print("x: " + str(cur_vertex.x) + "y: " +str(cur_vertex.y))
            plt.scatter(cur_vertex.edge_way_points[0], cur_vertex.edge_way_points[1], marker='o', edgecolors='green', s=10)
            cur_vertex = self.vertexes[cur_vertex.parent_index]
        return

    def draw_edge(self, waypointX, waypointY, i=0):
        # plot new edge between vertexes
        temp = plt.scatter(waypointX, waypointY, marker='o', edgecolors='red', s=0.1)
        plt.pause(0.05)
        return temp


def main():

    # initialize plot items
    plt.ion()
    fig = plt.figure()

    # initialize and display map
    map_with_obstacles, start, end = Track_Map.create_map()
    map_with_obstacles.display()

    # initialize and display graph items
    max_delta_per_v = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
    start = Vertex(start[0], start[1])
    end = Vertex(end[0], end[1])
    graph = Graph(start, end, 1, 1, max_delta_per_v, 0.01)

    # main loop for expansion
    iterations = 100
    vertex = start
    for i in range(0, iterations):

        # calculate next reachable vertex
        collide = True
        while collide:

            # get next guiding point in map - random or end (bias)
            if random.random() <= graph.bias_ratio:
                x = graph.end.x
                y = graph.end.y
            else:
                x, y = graph.get_random_point(map_with_obstacles)

            # find first valid edge from one of the K nearest vertexes, and draw it
            result, vertex, waypoints, time = graph.find_first_vertex_with_valid_edge_from_K_vertexes(x, y, map_with_obstacles)
            collide = not result

        # create new vertex from random point
        new_vertex = graph.create_new_vertex(vertex, waypoints, time)

        # wiring & rewiring
        graph.wiring(new_vertex, map_with_obstacles)
        graph.rewiring(new_vertex, map_with_obstacles)

        # add vertex to graph
        graph.add_vertex(new_vertex)

        # check if we got to the end vertex
        if graph.is_in_end_radius(new_vertex):
            graph.success = True
            print("-------HAYDEH-------")
            graph.draw_winning_path(start, new_vertex)

    plt.ioff()
    plt.show()


if __name__ == "__main__":
    main()
    print("FINISH")


'''
        # Algorithm:
        1. choose random vertex to be the parent.
        2. choose random delta, v, t.
        3. calculate new vertex (using the control command that ori will give us).
        4. wiring - loop for all existing vertexes and find the fastest path to new vertex from start point
            (at the end of this step we should have the new vertex and the path from start point to it in the tree)
        5. rewire - loop for all existing vertexes and for each existing vertex we will ask if the new vertex improves
            the path to existing vertex from start point

        # What is left:
        1. fix winning path color bug
        2. combine the path with with obstacles
        3. Optional - nice lines no circles
'''
