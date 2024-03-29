# Classical-Motion-Planning-Algorithms

This repository contains implementations of various classical path planning algorithms for autonomous agents such as robots or self-driving cars. The algorithms include:
- Breadth-first search based path finding
- Depth-first search based path finding
- A* algorithm
- Dijkstra's algorithm
- Probabilistic Roadmaps (PRM)
- Rapidly Exploring Random Trees (RRT, RRT* and Informed RRT*)

### Prerequisites
The code is written in Python and requires the following packages:
- Numpy
- Matplotlib
- csv 

### Usage
To run an algorithm, simply navigate to the relevant folder and run the main.py python file. For example, to run the A* and Dijkstra's implementations, navigate to the Astar, Dijkstra Algorithm Implementation folder and run main.py in your terminal or command prompt.

**Note:** The order of exploration is right, down, left, up for Dijkstra and A*.

**Note:** Manhattan distance is used as the heuristic for A*.

The demo visualizes the path generated by the algorithm, as well as the number of steps the specific algorithm took, i.e. the number of nodes visited to generate the final path.

### Algorithms in detail

#### Breadth-first search
Breadth-first search (BFS) is a graph search algorithm used for finding the shortest path between two vertices in an unweighted graph. It starts at the source vertex and explores all its neighbors before moving on to the neighbors' neighbors and so on. The algorithm uses a queue to keep track of the vertices to be explored, where each vertex is added to the queue as soon as it is discovered. BFS is guaranteed to find the shortest path between two vertices as it visits all vertices at a given depth level before moving on to the next.

#### Depth-first search
Depth-first search (DFS) is a graph search algorithm used for finding a path between two vertices in a graph. It starts at the source vertex and explores as far as possible along each branch before backtracking. The algorithm uses a stack to keep track of the vertices to be explored, where each vertex is added to the stack as soon as it is discovered. DFS may not necessarily find the shortest path between two vertices, as it prioritizes exploring deeper into the graph before backtracking and visiting other vertices at shallower levels.

#### Dijkstra's algorithm
Dijkstra's algorithm is a classic graph search algorithm that finds the shortest path from a start node to a goal node in a weighted graph. The algorithm maintains a priority queue of nodes to be expanded, where the priority is given by the current cost to reach that node. At each step, the node with the lowest cost is expanded and its neighbors are added to the queue, updating their cost if necessary. The algorithm terminates when the goal node is reached.

#### A* algorithm
A* is a best-first search algorithm that finds the shortest path from a start node to a goal node in a weighted graph. Like Dijkstra's, A* maintains a priority queue of nodes to be expanded, but with an additional heuristic that estimates the cost to reach the goal node from each node. The priority of each node is given by the sum of the current cost and the heuristic cost, encouraging the algorithm to prioritize nodes that are likely to lead to the goal.

#### PRM algorithm
PRM is a motion planning algorithm that works by sampling random points in the environment and connecting them to form a graph. The algorithm then uses probabilistic methods to search the graph for a feasible path. There are different mathods to sample the points including uniform, bridge, gaussian, etc.

#### RRT, RRT* and Informed RRT* algorithms
RRT (Rapidly-exploring Random Tree) and RRT* (Rapidly-exploring Random Tree star) are motion planning algorithms commonly used in robotics.

RRT works by incrementally building a tree from a starting position in the search space, exploring the space in a biased manner towards unexplored areas. RRT* is an extension of RRT, which uses an additional step of re-wiring the tree to optimize the paths towards the goal. Both algorithms are probabilistically complete, meaning they will find a solution if one exists with high probability, and are effective in high-dimensional and complex environments. Informed RRT* builds upon the RRT* algorithm by incorporating heuristics that guide the exploration towards promising areas of the search space.

#### D*
D* (pronounced "D-star") is a popular incremental search algorithm that can quickly adapt to changes in an agent's environment. It works by maintaining a search graph of the environment and updating it as the agent moves through it. This is useful for planning paths in a dynamic environment.

### Results

#### BFS and DFS

![alt text](https://github.com/mayankbansal82/Classical-Motion-Planning-Algorithms/blob/main/images/BFSDFS1.png)
The number of steps taken by BFS is 64 while DFS takes 33 steps to find the final path.

![alt text](https://github.com/mayankbansal82/Classical-Motion-Planning-Algorithms/blob/main/images/BFSDFS2.png)
The number of steps taken by BFS is 34 while DFS takes 51 steps to find the final path.

#### Dijkstra's and A*

![alt text](https://github.com/mayankbansal82/Classical-Motion-Planning-Algorithms/blob/main/images/AD1.png)
The number of steps taken by Dijkstra's is 64 while A* takes 51 steps to find the final path.

![alt text](https://github.com/mayankbansal82/Classical-Motion-Planning-Algorithms/blob/main/images/AD2.png)
The number of steps taken by Dijkstra's is 45 while A* takes 15 steps to find the final path.

#### PRM
##### Uniform Sampling
![alt text](https://github.com/mayankbansal82/Classical-Motion-Planning-Algorithms/blob/main/images/Uniform_sampling.png)

The constructed graph has 977 nodes and 4570 edges. The path length is 259.46.

##### Random Sampling
![alt text](https://github.com/mayankbansal82/Classical-Motion-Planning-Algorithms/blob/main/images/Random_sampling.png)

The constructed graph has 827 nodes and 4038 edges. The path length is 259.61.

##### Gaussian Sampling
![alt text](https://github.com/mayankbansal82/Classical-Motion-Planning-Algorithms/blob/main/images/Gaussian_sampling.png)

The constructed graph has 342 nodes and 1281 edges. The path length is 255.44.

##### Bridge Sampling
![alt text](https://github.com/mayankbansal82/Classical-Motion-Planning-Algorithms/blob/main/images/Bridge_sampling.png)

The constructed graph has 322 nodes and 1424 edges. The path length is 264.76.


#### RRT, RRT* and Informed RRT*
##### RRT
![alt text](https://github.com/mayankbansal82/Classical-Motion-Planning-Algorithms/blob/main/images/RRT.png)
It took 613 nodes to find the current path. The path length is 319.43.
##### RRT*
![alt text](https://github.com/mayankbansal82/Classical-Motion-Planning-Algorithms/blob/main/images/RRT_star.png)
It took 1297 nodes to find the current path. The path length is 258.28.
##### Informed RRT*
![alt text](https://github.com/mayankbansal82/Classical-Motion-Planning-Algorithms/blob/main/images/informed_rrt.png)
It took 1158 nodes to find the current path and the path length is 249.26.


#### D*
![alt text](https://github.com/mayankbansal82/Classical-Motion-Planning-Algorithms/blob/main/images/d_star.png)







