import math
import random
import numpy as np
import matplotlib.pyplot as plt
from dubins_path_planner import *
import Map


class Odom(object):
    def __init__(self):
        self.wheelbase = 0.35
        self.x = 0
        self.y = 0
        self.theta = 0

    def random_control(self, vertex, velocity, steering, time):
        theta_dot = velocity * np.tan(steering) / self.wheelbase
        dt = 0.03
        # print(theta_dot * time)
        waypoints_x = []
        waypoints_y = []
        waypoints_theta = []
        self.x = vertex.x
        self.y = vertex.y
        self.theta = vertex.theta
        waypoints_x.append(vertex.x)
        waypoints_y.append(vertex.y)
        waypoints_theta.append(vertex.theta)
        for _ in range(int(time / dt)):
            self.theta += theta_dot * dt
            x_dot = velocity * np.cos(self.theta)
            y_dot = velocity * np.sin(self.theta)
            self.x += x_dot * dt
            self.y += y_dot * dt
            waypoints_x.append(self.x)
            waypoints_y.append(self.y)
            waypoints_theta.append(self.theta)
        # self.x = waypoints_x[-1]
        # self.y = waypoints_y[-1]
        # self.theta = waypoints_theta[-1]
        return [waypoints_x, waypoints_y, waypoints_theta]


class Vertex:
    def __init__(self, x, y, theta, delta, v, cost, parent_index, ver_index, edge_way_points=None, scatter_temp=None):
        self.x = x
        self.y = y
        self.theta = theta
        self.delta = delta
        self.v = v
        self.cost = cost
        self.parent_index = parent_index
        self.ver_index = ver_index
        self.edge_way_points = edge_way_points  # this is the edge that connect vertex to start point
        self.scatter_temp = scatter_temp


class Graph:
    def __init__(self, start, end, obstacles, v_change_range, delta_change_range, max_delta_per_v, t_max, bias_ratio):
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
        self.t_max = t_max  # max time to new vertex
        self.bias_ratio = bias_ratio  # sets how often we expand by bias

    # adds new vertex
    def add_vertex(self, vertex):
        vertex.ver_index = len(self.vertexes)
        self.vertexes.append(vertex)

    # chooses random vertex to be the parent
    # returns the object of the chosen vertex
    def choose_random_existing_vertex(self):
        num_of_vertexes = len(self.vertexes)
        random_vertex_index = random.randint(0, num_of_vertexes - 1)
        return self.vertexes[random_vertex_index]

    # chooses random delta, v, t and return them
    def choose_random_parameters(self, vertex):
        valid_delta_for_v = False
        while not valid_delta_for_v:
            v = random.randint(max(1, vertex.v - self.v_change_range), vertex.v + self.v_change_range)
            # check if the lower limit of the random delta range is smaller\equal to the max delta allowed by v
            # if not, choose a new v
            if (vertex.delta - self.delta_change_range) <= self.max_delta_per_v[v]:
                valid_delta_for_v = True
                # check if the higher limit of the random delta range is smaller\equal to the max delta allowed by v
                if (vertex.delta + self.delta_change_range) <= self.max_delta_per_v[v]:
                    delta = random.uniform(vertex.delta - self.delta_change_range,
                                           vertex.delta + self.delta_change_range)
                # if not, make the max delta the higher limit of the range
                else:
                    delta = random.uniform(vertex.delta - self.delta_change_range, self.max_delta_per_v[v])
        t = random.randint(1, self.t_max)
        return delta, v, t

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

    # finds vertex nearest to end vertex and returns it (for bias expanding)
    def find_nearest_vertex_to_end(self):
        min_distance = self.calc_distance_to_end(self.start)
        nearest_vertex_to_end = self.start
        for vertex in self.vertexes:
            distance = self.calc_distance_to_end(vertex)
            if distance < min_distance:
                min_distance = distance
                nearest_vertex_to_end = vertex
        return nearest_vertex_to_end

    # calculates cost of target if its parent is source
    def calc_cost(self, source_vertex, terget_vertex):
        # new_cost = source_cost + delta time between them
        # self.calc_cost(current_vertex, new_vertex)
        path_x, path_y, path_yaw, mode, lengths = plan_dubins_path(source_vertex.x, source_vertex.y,
                                                                   source_vertex.theta, terget_vertex.x,
                                                                   terget_vertex.y, terget_vertex.theta, 1.0)
        v = source_vertex.v
        x = sum(lengths)
        t = x / max(1, v)
        target_cost = source_vertex.cost + t
        return path_x, path_y, target_cost

        # TODO: ori should give us
        return 1

    # checks if vertex is within track boundaries
    def is_in_track(self, x, y):
        # TODO: add later on, check if new vertex is within track boundaries
        return True

    # checks if it is possible to go from current vertex to target vertex
    def is_reachable(self, current_vertex, target_vertex):
        for i in range(waypoints[0].length):
            if race_track.inflated_map[waypoints[0][i], waypoints[1][i]].all():
                return False  # TODO: Chaim - must go over
        return True

    """""
    # creates a new vertex in graph
    def expand_graph(self):
        # choose an existing vertex to expand from
        if random.random(0, 1) <= self.bias_ratio:  # bias expanding
            parent_of_next_vertex = self.find_nearest_vertex_to_end(self.end)
        else:  # random expanding
            parent_of_next_vertex = self.choose_random_existing_vertex()
        # keep getting a new vertex to expand to, and stop when it's within the track boundaries
        first_iteration = True
        while not self.is_in_track(x, y) or first_iteration == True:
            delta, v, t = self.choose_random_parameters(parent_of_next_vertex)
            x, y, theta = self.control_command(parent_of_next_vertex, delta, v, t)
            first_iteration = False
        # create the new vertex object and add it to the vertexes list
        next_vertex = Vertex(x, y, theta, delta, v, parent_of_next_vertex.cost + t,
                             self.vertexes.index(parent_of_next_vertex))
        self.add_vertex(next_vertex)
        # check if we got to the end vertex
        if self.is_in_end_radius(next_vertex):
            self.success = True
    """""

    # draws the path in red
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

    # get k nearest vertexes to the given vertex
    def get_k_nearest(self, vertex):
        k = 2
        k_nearest = []
        k_distances = []
        for v in self.vertexes:
            if len(k_nearest) < k:
                k_nearest.append(v)
                k_distances.append(self.calc_distance_between_vertexes(vertex, v))
            else:
                max_index = k_distances.index(max(k_distances))
                current_distance = self.calc_distance_between_vertexes(vertex, v)
                if current_distance < k_distances[max_index]:
                    k_nearest[max_index] = v
                    k_distances[max_index] = current_distance
        return k_nearest

    # loop for k nearest vertexes and find the fastest path to new vertex from start point
    def wiring(self, new_vertex, ax):
        k_nearest = self.get_k_nearest(new_vertex)
        for index, current_vertex in enumerate(k_nearest):
            # if new vertex is reachable from current vertex -> check if new vertex cost is better
            if self.is_reachable(current_vertex, new_vertex):
                path_x, path_y, new_cost = self.calc_cost(current_vertex, new_vertex)
                # print("-------new_cost: " + str(new_cost))
                # print("-------current_cost: " + str(new_vertex.cost))
                if new_cost < new_vertex.cost:
                    # current vertex will be the parent of new vertex in the tree
                    new_vertex.cost = new_cost
                    new_vertex.parent_index = index
                    new_vertex.edge_way_points[0] = path_x
                    new_vertex.edge_way_points[1] = path_y
                    print("########did wiring thing########")
                    new_vertex.scatter_temp.remove()
                    new_vertex.scatter_temp = draw_edge2(path_x, path_y, ax)
        return

    # loop for k nearest vertexes, for each one ask if the new vertex improves the path to existing vertex from start point
    def rewiring(self, new_vertex, ax):
        k_nearest = self.get_k_nearest(new_vertex)
        for current_vertex in k_nearest:
            # if current vertex is reachable from new vertex -> check if current vertex cost is better
            if self.is_reachable(new_vertex, current_vertex):
                path_x, path_y, new_cost = self.calc_cost(new_vertex, current_vertex)
                if new_cost < current_vertex.cost:
                    # new vertex will be the parent of current vertex in the tree
                    current_vertex.cost = new_cost
                    current_vertex.parent_index = new_vertex.ver_index
                    current_vertex.edge_way_points[0] = path_x
                    current_vertex.edge_way_points[1] = path_y
                    print("^^^^^^^^ did RE-wiring thing ^^^^^^^^")
                    current_vertex.scatter_temp.remove()
                    current_vertex.scatter_temp = draw_edge2(path_x, path_y, ax)
        return

    def draw_winning_path(self, start, new_vertex, ax):
        import pdb;
        pdb.set_trace()
        cur_vertex = new_vertex
        while (cur_vertex is not start):
            print("x: " + str(cur_vertex.x) + "y: " + str(cur_vertex.y))
            ax.scatter(cur_vertex.edge_way_points[0], cur_vertex.edge_way_points[1], marker='o', edgecolors='green',
                       s=100)
            cur_vertex = self.vertexes[cur_vertex.parent_index]
        return


def draw_edge(source_vertex, terget_vertex):
    path_x, path_y, path_yaw, mode, lengths = plan_dubins_path(source_vertex.x, source_vertex.y, source_vertex.theta,
                                                               terget_vertex.x, terget_vertex.y, terget_vertex.theta,
                                                               1.0)
    plt.plot(path_x, path_y, label="".join(mode))
    plot_arrow(float(source_vertex.x), float(source_vertex.y), float(source_vertex.theta))
    plot_arrow(float(terget_vertex.x), float(terget_vertex.y), float(terget_vertex.theta))
    plt.legend()
    plt.grid(True)
    plt.axis("equal")
    plt.pause(0.05)


def remove_edge(temp):
    temp.remove()


def draw_edge2(waypointX, waypointY, ax, i=0):
    # plot new edge between vertexes
    temp = ax.scatter(waypointX, waypointY, marker='o', edgecolors='red', s=10)
    ax.annotate(str(i + 1), (waypointX[-1], waypointY[-1]))
    plt.pause(0.05)
    # import pdb;pdb.set_trace()
    return temp


def gui():
    # initialize objects and main variables
    race_track_map = np.array(np.load('smaller_circle.npy'), dtype=int)
    race_track = Map_Class(race_track_map, 1)

    fig = plt.figure()
    ax = fig.add_subplot()
    odom = Odom()
    max_delta_per_v = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
    start = Vertex(0, 0, 0, 0, 0, 0, 0, 0)
    ax.annotate("start", (start.x, start.y))
    end = Vertex(2, 2, 0, 0, 0, 0, 0, 0)
    ax.annotate("end", (end.x, end.y))
    graph = Graph(start, end, 0, 1, 1, max_delta_per_v, 1, 0.5)
    iterations = 20
    vertex = Vertex(odom.x, odom.y, odom.theta, 0, 0, 0, 0, 0)

    # main loop for expansion
    for i in range(0, iterations):

        # get parameters: random steering, velocity and time
        steering, velocity, time = graph.choose_random_parameters(vertex)
        print(
            f'iteration {i + 1}, velocity {velocity}, steering {steering * 180 / math.pi}, time {time}, start vertex {vertex.parent_index + 1}')

        # calculate new vertex from parameters and show it and its edge
        waypoints = odom.random_control(vertex, velocity, steering, time)  # waypoints = [[Xs],[Ys],[thetas]]
        new_vertex = Vertex(waypoints[0][-1], waypoints[1][-1], waypoints[2][-1], steering, velocity,
                            time + vertex.cost, vertex.ver_index, graph.get_num_of_vertexes() + 1, waypoints)

        # draw_edge(vertex, new_vertex)
        new_vertex.scatter_temp = draw_edge2(new_vertex.edge_way_points[0], new_vertex.edge_way_points[1], ax, i)

        # wiring operation
        graph.wiring(new_vertex, ax)
        # rewiring operation
        graph.rewiring(new_vertex, ax)

        # ax.scatter(waypoints[0], waypoints[1],marker='none')

        # check if we got to the end vertex
        if graph.is_in_end_radius(new_vertex):
            graph.success = True
            print("---------------------------------------------HAYDEH------------------------------------------")
            graph.draw_winning_path(start, new_vertex, ax)
            break

        # add new vertex to graph
        graph.add_vertex(new_vertex)

        # choose the next vertex
        if random.random() <= graph.bias_ratio:  # bias expanding
            vertex = graph.find_nearest_vertex_to_end()
            print("next is bias:")
        else:  # random expanding
            vertex = graph.choose_random_existing_vertex()
            print("next is random:")

    ax.set_aspect('equal', 'box')
    plt.show()


if __name__ == "__main__":
    gui()
    print("FINISH")

# TODO: where wiring and rewiring should be called from?
# I think from expand_graph function
'''
        # Algorithm:
        1. choose random vertex to be the parent.
        2. choose random delta, v, t.
        3. calculate new vertex (using the control command that ori will give us).
        4. wiring - loop for all existing vertexes and find the fastest path to new vertex from start point
            (at the end of this step we should have the new vertex and the path from start point to it in the tree)
        5. rewire - loop for all existing vertexes and for each existing vertex we will ask if the new vertex improves
            the path to existing vertex from start point

        # To be delivered by Ori:
         1. control command that by given delta, v and t returns  a destination point (x, y, theta).
         2. control command that by given 2 vertexes returns the time of getting from one to the other.

        # Future notes:
        1. by given point to check if new vertex is within track boundaries.
        2. what if there are a few good paths to the end point
'''
