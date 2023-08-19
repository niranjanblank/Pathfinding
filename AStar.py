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
        self.max_row = len(map)
        self.max_col = len(map[0])


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
        horizontal_moves = []
        vertical_moves = []

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
        elif pos[1] == self.max_row:
            vertical_moves = [-1]
        else:
            vertical_moves = [1, -1]

        # get horizontal neighbours
        for position_change in horizontal_moves:
            neighbour = pos.copy()
            neighbour[0] = neighbour[0] + position_change
            neighbour_positions.append(neighbour)

        # get vertical neighbours
        for position_change in vertical_moves:
            neighbour = pos.copy()
            neighbour[1] = neighbour[1] + position_change
            neighbour_positions.append(neighbour)

        # show the neighbours
        print(neighbour_positions)

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
    # astar_finder.show_map()
    astar_finder.get_neighbours([2,2])
