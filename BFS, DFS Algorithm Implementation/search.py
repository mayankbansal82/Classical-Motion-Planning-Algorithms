# Basic searching algorithms
import numpy as np
# Class for each node in the grid
class Node:
    def __init__(self, row, col, is_obs, h):
        self.row = row        # coordinate
        self.col = col        # coordinate
        self.is_obs = is_obs  # obstacle?
        self.cost = None      # total cost (depend on the algorithm)
        self.parent = None    # previous node


def bfs(grid, start, goal):
    '''Return a path found by BFS alogirhm 
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
    >>> bfs_path, bfs_steps = bfs(grid, start, goal)
    It takes 10 steps to find a path using BFS
    >>> bfs_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    path = []
    steps = 0
    found = False


    #  visited=[0]*V
    #     queue = []
    #     ans = []
       
    #     queue.append(0)
        
    #     while(queue):
    #         node = queue.pop(0)
    #         if visited[node] == 0:
    #             visited[node] = 1
    #             ans.append(node)
    #             for i in adj[node]:
    #                 if visited[i] == 0:
    #                     queue.append(i)

    # print(np.shape(grid))
    # print(len(grid[0]))
    visited=np.zeros((len(grid),len(grid[0])))
    # print(visited.shape)
    # visited=[[0]*(len(grid[0]))]*len(grid) #Initialized visited matrix
    #print(visited)
    #print(visited[0][0])
    queue = []
    ans = []
    parent = np.zeros((len(grid),len(grid[0]),2),dtype=int)

    queue.append(start)
    #print(start[0])
    # print(start)
    # print(goal)
    #print(np.shape(grid)[0])

    #print(grid)
    # print(visited)
    if grid[start[0]][start[1]]==0:
        while(len(queue)>0):
            node = queue.pop(0)
            #print(node)
            # print(node[0])
            # print(node[1])
            # print(visited)
            # print(queue)
            #print(visited[node[0]][node[1]])
            if visited[node[0]][node[1]] == 0: 
                visited[node[0]][node[1]]=1
                steps+=1
                ans.append(node)
                if node == goal:
                    found = True
                    break
                #print(ans)
                #r_nb = [node[0]][node[1]+1]
                if node[0] >= 0 and (node[1]+1) >= 0 and node[0] < np.shape(grid)[0] and (node[1]+1) < np.shape(grid)[1]:
                    if visited[node[0]][node[1]+1] == 0 and grid[node[0]][node[1]+1] == 0: #right neighbour
                        parent[node[0]][node[1]+1] = node
                        queue.append([node[0],node[1]+1])

                if (node[0]+1) >= 0 and node[1] >= 0 and (node[0]+1) < np.shape(grid)[0] and node[1] < np.shape(grid)[1]:
                    if visited[node[0]+1][node[1]] == 0 and grid[node[0]+1][node[1]] == 0: #down neighbour
                        parent[node[0]+1][node[1]] = node
                        queue.append([node[0]+1,node[1]])

                if node[0] >= 0 and (node[1]-1) >= 0 and node[0] <= np.shape(grid)[0] and (node[1]-1) < np.shape(grid)[1]:
                    if visited[node[0]][node[1]-1] == 0 and grid[node[0]][node[1]-1] == 0: #left neighbour
                        parent[node[0]][node[1]-1] = node
                        queue.append([node[0],node[1]-1])

                if (node[0]-1) >= 0 and node[1] >= 0 and (node[0]-1) < np.shape(grid)[0] and node[1] < np.shape(grid)[1]:
                    if visited[node[0]-1][node[1]] == 0 and grid[node[0]-1][node[1]] == 0: #up neighbour
                        parent[node[0]-1][node[1]] = node
                        queue.append([node[0]-1,node[1]])
        
    #print(parent)
    node = goal
    #print(node[0])
    if found:
        while(node!=start):
            path.append(node)
            node = [parent[node[0]][node[1]][0],parent[node[0]][node[1]][1]]
            #print(node)
        path.append(node)
        path.reverse()
    print(path)
    
    if found:
        print(f"It takes {steps} steps to find a path using BFS")
    else:
        print("No path found")
    return path, steps


def dfs(grid, start, goal):
    '''Return a path found by DFS alogirhm 
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
    >>> dfs_path, dfs_steps = dfs(grid, start, goal)
    It takes 9 steps to find a path using DFS
    >>> dfs_path
    [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2], [2, 3], [3, 3], [3, 2], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    path = []
    steps = 0
    found = False


    #  visited=[0]*V
    #     queue = []
    #     ans = []
       
    #     queue.append(0)
        
    #     while(queue):
    #         node = queue.pop(0)
    #         if visited[node] == 0:
    #             visited[node] = 1
    #             ans.append(node)
    #             for i in adj[node]:
    #                 if visited[i] == 0:
    #                     queue.append(i)


    visited=np.zeros((len(grid),len(grid[0])))
    # visited=[[0]*(len(grid[0]))]*len(grid) #Initialized visited matrix
    #print(visited)
    #print(visited[0][0])
    stack = []
    ans = []
    parent = np.zeros((len(grid),len(grid[0]),2),dtype=int)

    stack.append(start)
    #print(start[0])
    # print(start)
    # print(goal)
    #print(np.shape(grid)[0])

    #print(grid)
    # print(visited)
    if grid[start[0]][start[1]]==0:
        while(len(stack)>0):
            node = stack.pop()
            # print(node[0])
            # print(node[1])
            # print(visited)
            # print(queue)
            #print(visited[node[0]][node[1]])
            if visited[node[0]][node[1]] == 0: 
                visited[node[0]][node[1]]=1
                steps+=1
                ans.append(node)
                if node == goal:
                    found = True
                    break
                #print(ans)
                #r_nb = [node[0]][node[1]+1]

                if (node[0]-1) >= 0 and node[1] >= 0 and (node[0]-1) < np.shape(grid)[0] and node[1] < np.shape(grid)[1]:
                    if visited[node[0]-1][node[1]] == 0 and grid[node[0]-1][node[1]] == 0: #up neighbour
                        parent[node[0]-1][node[1]] = node
                        stack.append([node[0]-1,node[1]])

                if node[0] >= 0 and (node[1]-1) >= 0 and node[0] <= np.shape(grid)[0] and (node[1]-1) < np.shape(grid)[1]:
                    if visited[node[0]][node[1]-1] == 0 and grid[node[0]][node[1]-1] == 0: #left neighbour
                        parent[node[0]][node[1]-1] = node
                        stack.append([node[0],node[1]-1])

                if (node[0]+1) >= 0 and node[1] >= 0 and (node[0]+1) < np.shape(grid)[0] and node[1] < np.shape(grid)[1]:
                    if visited[node[0]+1][node[1]] == 0 and grid[node[0]+1][node[1]] == 0: #down neighbour
                        parent[node[0]+1][node[1]] = node
                        stack.append([node[0]+1,node[1]])

                if node[0] >= 0 and (node[1]+1) >= 0 and node[0] < np.shape(grid)[0] and (node[1]+1) < np.shape(grid)[1]:
                    if visited[node[0]][node[1]+1] == 0 and grid[node[0]][node[1]+1] == 0: #right neighbour
                        parent[node[0]][node[1]+1] = node
                        stack.append([node[0],node[1]+1])

            

          

            
         
    
    #print(visited)
    
    #print(parent)
    node = goal
    #print(node[0])
    if found:
        while(node!=start):
            path.append(node)
            node = [parent[node[0]][node[1]][0],parent[node[0]][node[1]][1]]
            #print(node)
        path.append(node)
        path.reverse()
    print(path)
    

    if found:
        print(f"It takes {steps} steps to find a path using DFS")
    else:
        print("No path found")
    return path, steps


# Doctest
if __name__ == "__main__":
    # load doc test
    from doctest import testmod, run_docstring_examples
    # Test all the functions
    testmod()
