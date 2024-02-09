import random
import numpy as np
import matplotlib.pyplot as plt 

class Odom(object):
    def __init__(self):
        self.wheelbase = 0.35
        self.x = 0
        self.y = 0
        self.theta = 0

    #get v, delta and time and return list of random [x,y,theta] 
    def random_control(self, velocity, steering, time):
        theta_dot = velocity * np.tan(steering) / self.wheelbase
        dt = 0.05
        self.theta += self.theta * time
        waypoints_x = []
        waypoints_y = []
        waypoints_theta = []
        waypoints_x.append(self.x)
        waypoints_y.append(self.y)
        waypoints_theta.append(self.theta)
        for _ in range(int(time/dt)):
            self.theta += theta_dot * dt
            x_dot = velocity * np.cos(self.theta)
            y_dot = velocity * np.sin(self.theta)
            self.x += x_dot * dt
            self.y += y_dot * dt
            waypoints_x.append(self.x)
            waypoints_y.append(self.y)
            waypoints_theta.append(self.theta)
        return [waypoints_x, waypoints_y, waypoints_theta]
    
class Vertex:
    def __init__(self, x, y, theta, delta, v, cost, parent_index, ver_index):
        self.x = x
        self.y = y
        self.theta = theta
        self.delta = delta
        self.v = v
        self.cost = cost
        self.parent_index = parent_index
        self.ver_index = ver_index


class Graph:
    def __init__(self, start, end, obstacles, v_change_range, delta_change_range, max_delta_per_v, t_max, bias_ratio):
        # graph attributes
        self.start = start
        self.end = end
        self.success = False #if success we have a path between start and end points
        self.vertexes = [start] # start that is passed as an argument must be a Vertex class object
        self.path = []
        self.end_radius = 15
        self.v_change_range = v_change_range # a new v will be within [v_current - v_change_range, v_current + v_change_range]
        self.delta_change_range = delta_change_range # a new delta will be within [delta_current - delta_change_range, delta_current + delta_change_range]
        self.max_delta_per_v = max_delta_per_v # an array which contains the max delta possible for a given v
        self.t_max = t_max # max time to new vertex
        self.bias_ratio = bias_ratio # sets how often we expand by bias


    # adds new vertex
    def add_vertex(self, vertex):
        vertex.ver_index = len(self.vertexes)
        self.vertexes.append(vertex)


    # chooses random vertex to be the parent
    # returns the object of the chosen vertex
    def choose_random_existing_vertex(self):
        num_of_vertexes = len(self.vertexes)
        random_vertex_index = random.randint(0, num_of_vertexes-1)
        return self.vertexes[random_vertex_index]


    # chooses random delta, v, t and return them
    def choose_random_parameters(self, vertex):
        valid_delta_for_v = False
        while not valid_delta_for_v:
            v = random.randint(vertex.v - self.v_change_range, vertex.v + self.v_change_range)
            # check if the lower limit of the random delta range is smaller\equal to the max delta allowed by v
            # if not, choose a new v
            if (vertex.delta - self.delta_change_range) <= self.max_delta_per_v[v]:
                valid_delta_for_v = True
                # check if the higher limit of the random delta range is smaller\equal to the max delta allowed by v
                if(vertex.delta + self.delta_change_range) <= self.max_delta_per_v[v]:
                    delta = random.randint(vertex.delta - self.delta_change_range, vertex.delta + self.delta_change_range)
                # if not, make the max delta the higher limit of the range
                else:
                    delta = random.randint(vertex.delta - self.delta_change_range, self.max_delta_per_v[v])
        t = random.randint(0, self.t_max)
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
        min_distance = self.calc_distance(self.vertexes[0])
        nearest_vertex_to_end = self.vertexes[0]
        for vertex in self.vertexes:
            distance = self.calc_distance(vertex)
            if distance < min_distance:
                min_distance = distance
                nearest_vertex_to_end = vertex
        return nearest_vertex_to_end


    # calculates cost of target if its parent is source
    def calc_cost(self, source, terget):
        #new_cost = source_cost + delta time between them
        # TODO: ori should give us
        return 1


    # by given delta, v and t the function returns new point (x,y,theta)
    def control_command(self, vertex, delta, t, v):
        # TODO: ori should give us
        return


    # checks if vertex is within track boundaries
    def is_in_track(self, x, y):
        # TODO: add later on, check if new vertex is within track boundaries
        return True


    #checks if it is possible to go from current vertex to target vertex
    def is_reachable(self, current_vertex, target_vertex):
        #TODO: show Chaim
        #TODO: ori should give us?
        return True


    # creates a new vertex in graph
    def expand_graph(self):
        # choose an existing vertex to expand from
        if random.random(0, 1) <= self.bias_ratio: # bias expanding
            parent_of_next_vertex = self.find_nearest_vertex_to_end(self.end)
        else: # random expanding
            parent_of_next_vertex = self.choose_random_existing_vertex()
        # keep getting a new vertex to expand to, and stop when it's within the track boundaries
        first_iteration = True
        while not self.is_in_track(x, y) or first_iteration == True:
            delta, v, t = self.choose_random_parameters(parent_of_next_vertex)
            x, y, theta = self.control_command(parent_of_next_vertex, delta, v, t)
            first_iteration = False
        # create the new vertex object and add it to the vertexes list
        next_vertex = Vertex(x, y, theta, delta, v, parent_of_next_vertex.cost + t, self.vertexes.index(parent_of_next_vertex))
        self.add_vertex(next_vertex)
        # check if we got to the end vertex
        if self.is_in_end_radius(next_vertex):
            self.success = True


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

    #get k nearest vertexes to the given vertex
    def get_k_nearest(self, vertex):
        k=2
        k_nearest = []
        k_distances = []
        for v in self.vertexes:
            if len(k_nearest)<k:
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
    def wiring(self, new_vertex):
        k_nearest = self.get_k_nearest(new_vertex)
        new_vertex_current_cost = 10000000 #big initial cost
        for index, current_vertex in k_nearest:
            #if new vertex is reachable from current vertex -> check if new vertex cost is better
            if self.is_reachable(current_vertex, new_vertex):
                new_cost = self.calc_cost(current_vertex, new_vertex)
                if new_cost < new_vertex_current_cost:
                    #current vertex will be the parent of new vertex in the tree
                    new_vertex_current_cost = new_cost
                    new_vertex.parent_index = index
        return


    # loop for k nearest vertexes, for each one ask if the new vertex improves the path to existing vertex from start point
    def rewiring(self, new_vertex):
        k_nearest = self.get_k_nearest(new_vertex)
        for current_vertex in k_nearest:
            # if current vertex is reachable from new vertex -> check if current vertex cost is better
            if self.is_reachable(new_vertex, current_vertex):
                new_cost = self.calc_cost(new_vertex, current_vertex)
                if new_cost < current_vertex.cost:
                    # new vertex will be the parent of current vertex in the tree
                    current_vertex.cost = new_cost
                    current_vertex.parent_index = new_vertex.index
        return


def gui():
    odom = Odom()
    fig = plt.figure()
    ax = fig.add_subplot()
    odom = Odom()
    iterations = 10
    for i in range(0, iterations):

        velocity = np.random.uniform(2.0)
        steering = np.random.uniform(-np.pi/6, np.pi/6)
        time = np.random.uniform(0.5,2)
        #print(f'velocity {velocity}, steering {steering}, time {time}')
        waypoints = odom.random_control(velocity=velocity, steering=steering, time=time)
        ax.scatter(waypoints[0], waypoints[1])
        print("__________________")
        print(waypoints)
        print("__________________")

        ax.set_aspect('equal', 'box')
    waypoints = [5,5,5]
    ax.set_aspect('equal', 'box')
    plt.show()

if __name__ == "__main__":
    gui()
    print("GOOD")


#TODO: where wiring and rewiring should be called from?
    #I think from expand_graph function
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
