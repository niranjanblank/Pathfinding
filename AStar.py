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
        self.open = []
        self.closed = []


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

class Node:
    """Node for the pathfinding algorithm"""
    def __init__(self, pos, status):
        self.pos = pos
        self.g = None
        self.h = None
        self.f = None

        # signifies if the node is obstacle or regular: 1 for regular, 0 for obstacle
        self.status = status


if __name__ == '__main__':
    map_matrix = [
        [1, 0, 0, 0, 1],
        [1, 1, 0, 0, 1],
        [1, 0, 1, 1, 1],
        [1, 0, 0, 0, 1],
    ]
    start_pos = [0,0]
    goal_pos = [3,3]

    astar_finder = AStar(start_pos, goal_pos, map_matrix)
    astar_finder.show_map()
