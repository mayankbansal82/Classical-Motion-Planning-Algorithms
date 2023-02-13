# Basic searching algorithms
import numpy as np
# Class for each node in the grid
class Node:
    def __init__(self, row, col, is_obs, h):
        self.row = row        # coordinate
        self.col = col        # coordinate
        self.is_obs = is_obs  # obstacle?
        self.g = None         # cost to come (previous g + moving cost)
        self.h = h            # heuristic
        self.cost = None      # total cost (depend on the algorithm)
        self.parent = None    # previous node

def dijkstra(grid, start, goal):
    '''Return a path found by Dijkstra alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> dij_path, dij_steps = dijkstra(grid, start, goal)
    It takes 10 steps to find a path using Dijkstra
    >>> dij_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    path = []
    steps = 0
    found = False
    visited=np.zeros((len(grid),len(grid[0])))
    distance = (np.ones((len(grid),len(grid[0])),dtype=int)*1000)
    queue = []
    parent = np.zeros((len(grid),len(grid[0]),2),dtype=int)

    queue.append([0,start])

    distance[start[0]][start[1]] = 0

    if grid[start[0]][start[1]]==0:
        while(len(queue)>0):
            length = len(queue)
            weights = [queue[i][0] for i in range(length)]
            minimum = np.argmin(weights)
            lst = queue[minimum]
            del queue[minimum] 
            node_dist = lst[0]
            node = lst[1]
            if visited[node[0]][node[1]] == 0: 
                visited[node[0]][node[1]]=1
                steps+=1
                if node == goal:
                    found = True
                    break

                if node[0] >= 0 and (node[1]+1) >= 0 and node[0] < np.shape(grid)[0] and (node[1]+1) < np.shape(grid)[1]:
                    if visited[node[0]][node[1]+1] == 0 and grid[node[0]][node[1]+1] == 0: #right neighbour
                        temp = node_dist + 1
                        if temp < distance[node[0]][node[1]+1]:
                            distance[node[0]][node[1]+1] = temp
                            parent[node[0]][node[1]+1] = node
                            queue.append([distance[node[0]][node[1]+1],[node[0],node[1]+1]])

                if (node[0]+1) >= 0 and node[1] >= 0 and (node[0]+1) < np.shape(grid)[0] and node[1] < np.shape(grid)[1]:
                    if visited[node[0]+1][node[1]] == 0 and grid[node[0]+1][node[1]] == 0: #down neighbour
                        temp = node_dist + 1
                        if temp < distance[node[0]+1][node[1]]:
                            distance[node[0]+1][node[1]] = temp
                            parent[node[0]+1][node[1]] = node
                            queue.append([distance[node[0]+1][node[1]],[node[0]+1,node[1]]])

                if node[0] >= 0 and (node[1]-1) >= 0 and node[0] <= np.shape(grid)[0] and (node[1]-1) < np.shape(grid)[1]:
                    if visited[node[0]][node[1]-1] == 0 and grid[node[0]][node[1]-1] == 0: #left neighbour
                        temp = node_dist + 1
                        if temp < distance[node[0]][node[1]-1]:
                            distance[node[0]][node[1]-1] = temp
                            parent[node[0]][node[1]-1] = node
                            queue.append([distance[node[0]][node[1]-1],[node[0],node[1]-1]])
                        
                if (node[0]-1) >= 0 and node[1] >= 0 and (node[0]-1) < np.shape(grid)[0] and node[1] < np.shape(grid)[1]:
                    if visited[node[0]-1][node[1]] == 0 and grid[node[0]-1][node[1]] == 0: #up neighbour
                        temp = node_dist + 1
                        if temp < distance[node[0]-1][node[1]]:
                            distance[node[0]-1][node[1]] = temp
                            parent[node[0]-1][node[1]] = node
                            queue.append([distance[node[0]-1][node[1]],[node[0]-1,node[1]]])

    node = goal
    if found:
        while(node!=start):
            path.append(node)
            node = [parent[node[0]][node[1]][0],parent[node[0]][node[1]][1]]
        path.append(node)
        path.reverse()
    # print(path)

    if found:
        print(f"It takes {steps} steps to find a path using Dijkstra")
    else:
        print("No path found")
    return path, steps


def astar(grid, start, goal):
    '''Return a path found by A* alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> astar_path, astar_steps = astar(grid, start, goal)
    It takes 7 steps to find a path using A*
    >>> astar_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    path = []
    steps = 0
    found = False
    visited=np.zeros((len(grid),len(grid[0])))
    distance = (np.ones((len(grid),len(grid[0])),dtype=int)*1000)
    g=np.zeros((len(grid),len(grid[0])))
    queue = []
    parent = np.zeros((len(grid),len(grid[0]),2),dtype=int)

    queue.append([0,start])
    distance[start[0]][start[1]] = 0 

    if grid[start[0]][start[1]]==0:
        while(len(queue)>0):
            length = len(queue)
            weights = [queue[i][0] for i in range(length)]
            minimum = np.argmin(weights)
            lst = queue[minimum]
            del queue[minimum] 
            node_dist = lst[0]
            node = lst[1]
        
            if visited[node[0]][node[1]] == 0: 
                visited[node[0]][node[1]]=1
                steps+=1
                if node == goal:
                    found = True
                    break
                if node[0] >= 0 and (node[1]+1) >= 0 and node[0] < np.shape(grid)[0] and (node[1]+1) < np.shape(grid)[1]:
                    if visited[node[0]][node[1]+1] == 0 and grid[node[0]][node[1]+1] == 0: #right neighbour
                        g[node[0]][node[1]+1] = g[node[0]][node[1]] + 1
                        temp =  g[node[0]][node[1]+1] + np.abs((goal[0]-node[0])) + np.abs((goal[1]-(node[1]+1))) #manhattan distance
                        # temp =  g[node[0]][node[1]+1] + np.sqrt((goal[0]-node[0])**2 + (goal[1]-(node[1]+1))**2) #euclidean distance
                        if temp < distance[node[0]][node[1]+1]:
                            distance[node[0]][node[1]+1] = temp
                            parent[node[0]][node[1]+1] = node
                            queue.append([distance[node[0]][node[1]+1],[node[0],node[1]+1]])

                if (node[0]+1) >= 0 and node[1] >= 0 and (node[0]+1) < np.shape(grid)[0] and node[1] < np.shape(grid)[1]:
                    if visited[node[0]+1][node[1]] == 0 and grid[node[0]+1][node[1]] == 0: #down neighbour
                        g[node[0]+1][node[1]] = g[node[0]][node[1]] + 1
                        temp =  g[node[0]+1][node[1]] + np.abs((goal[0]-(node[0]+1))) + np.abs((goal[1]-node[1]))
                        # temp = g[node[0]+1][node[1]] + np.sqrt((goal[0]-(node[0]+1))**2 + (goal[1]-node[1])**2)
                        if temp < distance[node[0]+1][node[1]]:
                            distance[node[0]+1][node[1]] = temp
                            parent[node[0]+1][node[1]] = node
                            queue.append([distance[node[0]+1][node[1]],[node[0]+1,node[1]]])

                if node[0] >= 0 and (node[1]-1) >= 0 and node[0] <= np.shape(grid)[0] and (node[1]-1) < np.shape(grid)[1]:
                    if visited[node[0]][node[1]-1] == 0 and grid[node[0]][node[1]-1] == 0: #left neighbour
                        g[node[0]][node[1]-1] = g[node[0]][node[1]] + 1
                        temp =  g[node[0]][node[1]-1] + np.abs((goal[0]-node[0])) + np.abs((goal[1]-(node[1]-1)))
                        # temp = g[node[0]][node[1]-1] + np.sqrt((goal[0]-node[0])**2 + (goal[1]-(node[1]-1))**2)
                        if temp < distance[node[0]][node[1]-1]:
                            distance[node[0]][node[1]-1] = temp
                            parent[node[0]][node[1]-1] = node
                            queue.append([distance[node[0]][node[1]-1],[node[0],node[1]-1]])
                        
                if (node[0]-1) >= 0 and node[1] >= 0 and (node[0]-1) < np.shape(grid)[0] and node[1] < np.shape(grid)[1]:
                    if visited[node[0]-1][node[1]] == 0 and grid[node[0]-1][node[1]] == 0: #up neighbour
                        g[node[0]-1][node[1]] = g[node[0]][node[1]] + 1
                        temp =  g[node[0]-1][node[1]] + np.abs((goal[0]-(node[0]-1))) + np.abs((goal[1]-node[1]))
                        # temp = g[node[0]-1][node[1]] + np.sqrt((goal[0]-(node[0]-1))**2 + (goal[1]-node[1])**2)
                        if temp < distance[node[0]-1][node[1]]:
                            distance[node[0]-1][node[1]] = temp
                            parent[node[0]-1][node[1]] = node
                            queue.append([distance[node[0]-1][node[1]],[node[0]-1,node[1]]])

    node = goal
    if found:
        while(node!=start):
            path.append(node)
            node = [parent[node[0]][node[1]][0],parent[node[0]][node[1]][1]]
        path.append(node)
        path.reverse()
    # print(path)

    if found:
        print(f"It takes {steps} steps to find a path using A*")
    else:
        print("No path found")
    return path, steps

# Doctest
if __name__ == "__main__":
    # load doc test
    from doctest import testmod, run_docstring_examples
    # Test all the functions
    testmod()
