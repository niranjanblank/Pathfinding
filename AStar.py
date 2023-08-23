from itertools import product
import math
class AStar:
    def __init__(self, start, goal, map):
        """
        self.map is a dictionary holding all the position and their respective nodes as key value pair
        self.start is the start position
        self.goal is the position where we want to reach
        self.open is a list to store set of nodes to be evaluated
        self.closed is a list to store set of nodes that are already evaluated
        """
        self.map = self.create_map(map)
        self.start = start
        self.goal = goal
        # adding the starting square to open list
        self.open = [self.start]
        self.closed = []
        self.max_row = len(map)-1
        self.max_col = len(map[0])-1


    def create_map(self, map):
        """ map position and their respective nodes are stored in dictionary for quick access"""
        grid = {}
        rows = len(map)
        cols = len(map[0])
        for row in range(rows):
            for col in range(cols):
                node = Node([row,col],map[row][col])
                grid[(row,col)] = node

        return grid

    def show_map(self):
        for key,value in self.map.items():
            print(f'{key}: {value.status}')

    def get_neighbours(self, pos):
        neighbour_positions = []

        # estimating horizontal neighbour to calculate
        if pos[0] == 0:
            horizontal_moves = [1]
        elif pos[0] == self.max_row:
            horizontal_moves = [-1]
        else:
            horizontal_moves = [1, -1]

        # estimating vertical neighbour to calculate
        if pos[1] == 0:
            vertical_moves = [1]
        elif pos[1] == self.max_col:
            vertical_moves = [-1]
        else:
            vertical_moves = [1, -1]

        # estimating diagonal neighbour to calculate
        diagonal_moves = list(product(horizontal_moves, vertical_moves))
        # get horizontal neighbours
        for position_change in horizontal_moves:
            neighbour = pos.copy()
            neighbour[0] = neighbour[0] + position_change
            node = self.map[tuple(neighbour)]
            if node.status != 0:
                neighbour_positions.append(neighbour)

        # get vertical neighbours
        for position_change in vertical_moves:
            neighbour = pos.copy()
            neighbour[1] = neighbour[1] + position_change
            node = self.map[tuple(neighbour)]
            if node.status != 0:
                neighbour_positions.append(neighbour)

        # get diagonal neighbours
        for position_change in diagonal_moves:
            neighbour = pos.copy()
            neighbour[0] = neighbour[0] + position_change[0]
            neighbour[1] = neighbour[1] + position_change[1]
            node = self.map[tuple(neighbour)]
            if node.status != 0:
                neighbour_positions.append(neighbour)

        return neighbour_positions

    def run(self):

        # loop to check inside open list
        for node in self.open:
            # finding the pos of node with lowest f cost
            current_node_pos = self.find_lowest_node_from_open()
            current_node = self.map[tuple(current_node_pos)]
            current_node.compute_g_value(self.start)
            current_node.compute_h_value(self.goal)
            current_node.compute_f_value()

            # removing from open list and switching it to closed list
            self.closed.append(current_node_pos)
            self.open.remove(current_node_pos)

            # finding the neighbours for the current node
            neighbour_list = self.get_neighbours(current_node.pos)
            for neighbour in neighbour_list:
                if neighbour not in self.open:
                    self.open.append(neighbour)
                    neighbour_node = self.map[tuple(neighbour)]
                    neighbour_node.compute_g_value(self.start)
                    neighbour_node.compute_h_value(self.goal)
                    neighbour_node.compute_f_value()
                    neighbour_node.parent = current_node.pos
        print(self.open)


    def find_lowest_node_from_open(self):
        lowest_f_cost_pos = min(self.open, key=lambda item: self.map[tuple(item)].f)
        return lowest_f_cost_pos
class Node:
    """Node for the pathfinding algorithm"""
    """status = 0 means that path is not walkable """
    def __init__(self, pos, status):
        self.pos = pos
        self.g = None
        self.h = None
        self.f = None
        self.parent = None

        # signifies if the node is obstacle or regular: 1 for regular, 0 for obstacle
        self.status = status

    def euclidean_distance(self, point1, point2):
        """
        Method to calculate distance between two points, which will be used to calculate
        g and h values
        """
        distance = math.sqrt((point2[0]-point1[0])**2 + (point2[1]-point1[1])**2)
        distance = round(distance,1)
        return distance
    def compute_g_value(self, start_node_pos):
        """
        G value is the distance from the node to the start node
        """
        distance = self.euclidean_distance(self.pos, start_node_pos)
        # multiplying with 10 to get whole number, which is faster for computer to work on
        distance = distance * 10
        self.g = int(distance)

    def compute_h_value(self, goal_node_pos):
        """
        H value is the distance from the node to the goal node
        """
        distance = self.euclidean_distance(self.pos, goal_node_pos)
        # multiplying with 10 to get whole number, which is faster for computer to work on
        distance = distance * 10
        self.h = int(distance)

    def compute_f_value(self):
        """
        Compute f-value = g + h
        """
        self.f = self.g + self.h

if __name__ == '__main__':
    map_matrix = [
        [1, 1, 0, 0, 1],
        [1, 1, 0, 0, 1],
        [1, 0, 1, 1, 1],
        [1, 0, 1, 1, 1],
    ]
    start_pos = [0,0]
    goal_pos = [2,2]

    astar_finder = AStar(start_pos, goal_pos, map_matrix)
    # astar_finder.show_map()
    astar_finder.run()
