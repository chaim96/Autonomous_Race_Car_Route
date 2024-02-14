import time

from RRT import Graph
from Map import Map
import RRT_Star

""""
def main():
    # setting initial numbers
    map_dimensions = (600, 1000)
    start = (50, 50)
    goal = (800, 510)
    obstacle_dimensions = 30
    obstacles_num = 40

    # setting initial classes and libraries
    pygame.init()
    map = Map(start, goal, map_dimensions, obstacle_dimensions, obstacles_num)
    graph = Graph(start, goal, map_dimensions, map.obstacles)
    map.draw_map()

    iteration = 0

    # main RRT loop
    while not graph.success:

        if iteration % 10 == 0:
            result, new_vertex, parent_vertex = graph.bias_expanding()
            if result:
                pygame.draw.circle(map.map, map.blue, new_vertex, map.vertex_thickness)
                pygame.draw.line(map.map, map.blue, new_vertex, parent_vertex, map.edge_thickness)

        else:
            result, random_sample, new_vertex, parent_vertex = graph.random_expanding()
            if result:
                #pygame.draw.circle(map.map, map.green, random_sample, map.vertex_thickness)
                pygame.draw.circle(map.map, map.blue, new_vertex, map.vertex_thickness)
                pygame.draw.line(map.map, map.blue, new_vertex, parent_vertex, map.edge_thickness)

        if iteration % 10 == 0:
            pygame.display.update()
            time.sleep(0.5)

        iteration += 1

    map.draw_path(graph.get_path_to_goal())
    pygame.display.update()
    pygame.event.clear()
    pygame.event.wait(0)
"""


# if __name__ == '__main__':

