
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
    def __init__(self, start, end, obstacles):
        # graph attributes
        self.start = start
        self.end = end
        self.success = False #if succes we have a path between start and end points
        self.vertexes = [start]
        self.path = []
        self.end_radius = 15

    # adding & removing vertexes and edges
    def add_vertex(self, vertex):
        self.vertexes.append(vertex)

    #choose random vertex to be the parent- scan the vertexes list
    #retun the object of the chosen vertex
    def choose_random_existing_vertex(self):
        #TODO: Chaim

    #choose random delta, v, t
    def choose_random_parameters(self, vertex):
        # TODO: Chaim
        #first choose random v and by that choose random delta

    #returns number of vertexes in the graph
    def get_num_of_vertexes(self):
        return len(self.vertexes)

    # calculates if new vertex in end radius
    def is_in_end_radius(self, a, b): #TODO:chaim
        (x1, y1) = (a[0], a[1])
        (x2, y2) = (b[0], b[1])
        px = (float(x1) - float(x2)) ** 2
        py = (float(y1) - float(y2)) ** 2
        distance = (px + py) ** (0.5)
        return distance

    # calculates cost between 2 vertexes
    def calc_cost(self, a, b):
        #TODO: ori should give us

    #by given delta, v and t the function return new point (x,y,theta)
    def control_command(self, a, b):
        # TODO: ori should give us

    # creates a new vertex in graph towards the goal
    def bias_expanding(self): #TODO: chaim
        nearest_vertex, nearest_vertex_index = self.find_vertex_nearest_to_sample(self.goal)
        next_vertex = self.create_next_vertex_between_sample_and_nearest_vertex(nearest_vertex, self.goal,
                                                                                    nearest_vertex_index)
        result = self.connect_vertexes(nearest_vertex, nearest_vertex_index, next_vertex)
        if self.calc_distance(self.goal, sample) < self.goal_radius:
            self.add_vertex(self.goal, parent_index)
            self.success = True
        return sample
        return result, (next_vertex[0], next_vertex[1]), (nearest_vertex[0], nearest_vertex[1])

    # creates a new vertex in graph towards a random sample
    def random_expanding(self):#TODO: chaim
        random_sample = self.create_random_sample()
        nearest_vertex, nearest_vertex_index = self.find_vertex_nearest_to_sample(random_sample)
        next_vertex = self.create_next_vertex_between_sample_and_nearest_vertex(nearest_vertex, random_sample,
                                                                                    nearest_vertex_index)
        # result is true id connecting vertexes work
        result = self.connect_vertexes(nearest_vertex, nearest_vertex_index, next_vertex)
        return result, random_sample, (next_vertex[0], next_vertex[1]), (nearest_vertex[0], nearest_vertex[1])

    # draw the path in red
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

    #loop for all existing vertexes and find the fastest path to new vertex from start point
    def wiring(self, new_vertex):  #TODO: dana

    #loop for all existing vertexes and for each existing vertex we will ask if the new vertex improves the path to existing vertex from start point
    def rewiring(self, new_vertex): #TODO: dana

        '''
        1. choose random vertex to be the parent
        2. choose random delta, v, t
        3. calculate new vertex - this is the control command that ori will give us
        4. wiring - loop for all existing vertexes and find the fastest path to new vertex from start point
            (at the end of this step we should have the new vertex and the path from start point to it in the tree)
        5. rewire - loop for all existing vertexes and for each existing vertex we will ask if the new vertex improves the path to existing vertex from start point

        #ori will give us a control command that by given 2 vertexes it return the tine of getting from one to the other

        #for the future:
        1. by given point to check if new vertex is within track boundaries.
        2. what if there are a few good paths to the end point

        '''
