from AStar import *

if __name__ == '__main__':
    map_matrix = [
        [1, 1, 0, 0, 1],
        [1, 1, 0, 0, 1],
        [1, 0, 1, 1, 1],
        [1, 0, 1, 1, 1],
    ]
    start_pos = [3, 0]
    goal_pos = [0, 4]

    astar_finder = AStar(start_pos, goal_pos, map_matrix)

    shortest_path = astar_finder.run()
    if shortest_path:
        print(shortest_path)
    else:
        print('path not found: destination is unreachable')