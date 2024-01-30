
class Vertex:
    def __init__(self, x, y, theta, delta, v, cost, parent_index):
        self.x = x
        self.y = y
        self.theta = theta
        self.delta = delta
        self.v = v
        self.cost = cost
        self.parent_index = parent_index


class Graph:
    def __init__(self, start, end, obstacles, v_change_range, delta_change_range, max_delta_per_v, t_max):
        # graph attributes
        self.start = start
        self.end = end
        self.success = False #if succes we have a path between start and end points
        self.vertexes = [start]
        self.path = []
        self.end_radius = 15
        self.v_change_range = v_change_range # a new v will be within [v_current - v_change_range, v_current + v_change_range]
        self.delta_change_range = delta_change_range # a new delta will be within [delta_current - delta_change_range, delta_current + delta_change_range]
        self.max_delta_per_v = max_delta_per_v # an array which contains the max delta possible for a given v
        self.t_max = t_max # max time to new vertex
        self.iteration = 0 # stores number of iterations


    # adds new vertex
    def add_vertex(self, vertex):
        self.vertexes.append(vertex)


    # chooses random vertex to be the parent
    # returns the object of the chosen vertex
    def choose_random_existing_vertex(self):
        num_of_vertexes = len(self.vertexes)
        random_vertex_index = random(0, num_of_vertexes-1)
        return self.vertexes[random_vertex_index]


    # chooses random delta, v, t and return them
    def choose_random_parameters(self, vertex):
        valid_delta_for_v = False
        while not valid_delta_for_v:
            v = random(vertex.v - self.v_change_range, vertex.v + self.v_change_range)
            # check if the lower limit of the random delta range is smaller\equal to the max delta allowed by v
            # if not, choose a new v
            if (vertex.delta - self.delta_change_range) <= self.max_delta_per_v[v]:
                valid_delta_for_v = True
                # check if the higher limit of the random delta range is smaller\equal to the max delta allowed by v
                if(vertex.delta + self.delta_change_range) <= self.max_delta_per_v[v]:
                    delta = random(vertex.delta - delta_change_range, vertex.delta + delta_change_range)
                # if not, make the max delta the higher limit of the range
                else:
                    delta = random(vertex.delta - delta_change_range, self.max_delta_per_v[v])
        t = random(0, self.t_max)
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


    # finds vertex nearest to end vertex and returns it (for bias expanding)
    def find_nearest_vertex_to_end(self):
        min_distance = self.calc_distance(self.vertexes[0])
        nearest_vertex_to_end = self.vertexes[0]
        for vertex in self.vertexes:
            distance = self.calc_distance(vertex)
            if distance < min_distance:
                min_distance = distance
                nearest_vertex_to_end = vertex
        return nearest_vertex_to_rand_sample


    # calculates cost between 2 vertexes
    def calc_cost(self, a, b):
        # TODO: ori should give us
        return


    # by given delta, v and t the function returns new point (x,y,theta)
    def control_command(self, vertex, delta, t, v):
        # TODO: ori should give us
        return


    # checks if vertex is within track boundaries
    def is_in_track(self, x, y):
        # TODO: add later on, check if new vertex is within track boundaries
        return True


    # creates a new vertex in graph
    def expand_graph(self):
        # choose an existing vertex to expand from
        if self.iteration % 10 == 0: # bias expanding TODO: ask ori of essence of bias here
            parent_of_next_vertex = self.find_nearest_vertex_to_end(self.goal)
        else: # random expanding
            parent_of_next_vertex = choose_random_existing_vertex()
        # keep getting a new vertex to expand to, and stop when it's within the track boundaries
        first_iteration = True
        while not is_in_track(x, y) or first_iteration == True:
            delta, v, t = choose_random_parameters(parent_of_next_vertex)
            x, y, theta = control_command(parent_of_next_vertex, delta, v, t)
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


    # loop for all existing vertexes and find the fastest path to new vertex from start point
    def wiring(self, new_vertex):  #TODO: Dana the queen
        return


    # loop for all existing vertexes and for each existing vertex we will ask if the new vertex improves the path to existing vertex from start point
    def rewiring(self, new_vertex): #TODO: Dana the queen
        return


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
