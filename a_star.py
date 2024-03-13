import numpy as np
import matplotlib.pyplot as plt
import heapq
from scipy import interpolate


class A_Star(object):
    def __init__(self, map_: np.array, inflation=0):
        self.map = map_
        self.inflated_map = self.inflate(inflation)

    def inflate(self, inflation):#, resolution, distance):
        cells_as_obstacle = inflation #int(distance/resolution)
        original_map = self.map.copy()
        inflated_map = self.map.copy()
        self.rows, self.cols = inflated_map.shape
        for j in range(self.cols):
            for i in range(self.rows):
                if original_map[i,j] != 0:
                    i_min = max(0, i-cells_as_obstacle)
                    i_max = min(self.rows, i+cells_as_obstacle)
                    j_min = max(0, j-cells_as_obstacle)
                    j_max = min(self.cols, j+cells_as_obstacle)
                    inflated_map[i_min:i_max, j_min:j_max] = 100
        return inflated_map
    
     
        
    def h(self, current, goal):
        return ((current[0] - goal[0])**2 + (current[1] - goal[1])**2)**0.5
    
    def reconstruct_path(self, current, came_from, start):
        path = []
        while current != start:
            path.append(current)
            current = came_from[current]
        path.append(start)
        return path[::-1]

    def get_neighbors(self, current, closed_list):
        '''
        return neigbors and cost.
        add only neigbors that are not in closed list
        '''
        i, j = current
        neighbors = []
        # right 
        if i < self.rows-1 and self.inflated_map[i+1,j] == 0 and closed_list[i+1,j] == 0: 
            neighbors.append([(i+1,j), 1])
        # left
        if i > 0 and self.inflated_map[i-1,j] == 0 and closed_list[i-1,j] == 0:
            neighbors.append([(i-1, j), 1])
        # up 
        if j < self.cols-1 and self.inflated_map[i,j+1] == 0 and closed_list[i,j+1] == 0:
            neighbors.append([(i,j+1), 1])
        # down
        if j > 0 and self.inflated_map[i,j-1] == 0 and closed_list[i,j-1] == 0:
            neighbors.append([(i, j-1), 1])
        return neighbors
        

    def find_path(self, start, goal):
        open_list = [(self.h(start, goal), start)]
        gscore = dict()
        came_from = dict()
        gscore[start] = 0
        closed_list = np.zeros_like(self.inflated_map, dtype=int)

        heapq.heapify(open_list)
        while open_list:
            current_cost, current = heapq.heappop(open_list)
            closed_list[current[0],current[1]] = 1
            if current == goal:
                return self.reconstruct_path(current, came_from, start)
            neighbors = self.get_neighbors(current, closed_list)
            for neighbor, distance in neighbors:
                tentative_gscore = current_cost + distance
                if neighbor not in gscore.keys():
                    gscore[neighbor] = np.inf
                if tentative_gscore < gscore[neighbor]:
                    came_from[neighbor] = current
                    gscore[neighbor] = tentative_gscore
                    heapq.heappush(open_list, (gscore[neighbor]+self.h(neighbor, goal), neighbor) )
        print('No Path Found')
        return None




map_ = np.array(np.load('maze_1.npy'), dtype=int)

astar = A_Star(map_, inflation=2)
start=(78, 110)
goal=(106,71)
path = astar.find_path(start, goal)


fig = plt.figure()
ax = fig.add_subplot()
ax.imshow(astar.inflated_map, origin="lower")
ax.axis('equal')

if path is not None:
    for vertix in path:
        map_[vertix[0], vertix[1]] = 80
    x,y = [], []
    for idx, vertix in enumerate(path):
        if idx % 10 ==0:
            x.append(vertix[1])
            y.append(vertix[0])
    ax.scatter(x,y)

ax.scatter(goal[0], goal[1])
ax.scatter(start[0], start[1])
plt.show()


