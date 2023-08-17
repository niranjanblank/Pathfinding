class AStar:
    def __init__(self, start, goal, map):
        self.map = self.create_grid(map)
        self.start = start
        self.goal = goal


    def create_grid(self, map):
        grid = []
        rows = len(map)
        cols = len(map[0])
        for row in range(rows):
            temp_node_list = []
            for col in range(cols):
                temp_node_list.append(Node([row,col],map[row][col]))

            grid.append(temp_node_list)
        return grid

    def show_map(self):
        rows = len(self.map)
        cols = len(self.map[0])
        for row in range(rows):
            for col in range(cols):
                print(f'{self.map[row][col].pos} : {self.map[row][col].status}', end=",")
            print()
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
