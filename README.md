## Pathfinding Algorithms
Pathfinding algorithms are used in various fields, including computer science and video game development, to find the shortest or most efficient path between two points in a graph or grid. These algorithms are particularly relevant in scenarios where navigation or optimization is necessary, such as finding the shortest route between locations on a map or guiding characters in a video game through complex environments.

The pathfinding algorithms I have implemented till now are:
1. AStar: A* is a widely used heuristic search algorithm that combines Dijkstra's algorithm's reliability with heuristics to improve efficiency. It is often used in video games and robotics for 
pathfinding. Its algorithm is as below:
   1. Add start point(node) to the open list
   2. Until the path is found or open list is empty, run the below loop:
     * Compute lowest F cost square on the open list
     * Compute F,G,H cost
     * Remove it from open list and add it to closed list
     * Calculate all the walkable neighbours
     * For each neighbour from the walkable neighbours
       * If it's in the closed list, ignore the neighbour anc continue to next neighbour
       * If it's not in the open list, add it to the open list. Assign current node to be its parent. Calculate F,G and H cost
       * If it's in the open list, compare its G value with previous value. If G value is lower than previous, change the parent to current node,, and recalculate G and F
     * Stop the loop when:
       * Target is in the closed loop
       * Open list is empty, no path is found
   3. If target is in the closed loop, transverse backwards from target to start node through parent to get the shortest path.