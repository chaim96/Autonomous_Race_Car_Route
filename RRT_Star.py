
class Vertex:
    def __init__(self, x, y, theta, delta, v, cost):
        self.x = x
        self.y = y
        self.theta = theta
        self.delta = delta
        self.v = v
        self.cost = cost

class Graph:
    def __init__(self, start, end, obstacles):
        # graph attributes
        self.start = start
        self.end = end
        self.success = False #if succes we have a path between start and end points
        self.vertexes = [start]
        self.edges = []
        self.path = []
        self.goal_radius = 15