# Standard Algorithm Implementation
# Sampling-based Algorithms RRT and RRT*

import matplotlib.pyplot as plt
import numpy as np
from scipy import spatial


# Class for each tree node
class Node:
    def __init__(self, row, col):
        self.row = row        # coordinate
        self.col = col        # coordinate
        self.parent = None    # parent node
        self.cost = 0.0       # cost


# Class for RRT
class RRT:
    # Constructor
    def __init__(self, map_array, start, goal):
        self.map_array = map_array            # map array, 1->free, 0->obstacle
        self.size_row = map_array.shape[0]    # map size
        self.size_col = map_array.shape[1]    # map size

        self.start = Node(start[0], start[1]) # start node
        # print(self.start)
        self.goal = Node(goal[0], goal[1])    # goal node
        self.vertices = []                    # list of nodes
        self.found = False                    # found flag
        

    def init_map(self):
        '''Intialize the map before each search
        '''
        self.found = False
        self.vertices = []
        self.vertices.append(self.start)
        

    
    def dis(self, node1, node2):
        '''Calculate the euclidean distance between two nodes
        arguments:
            node1 - node 1
            node2 - node 2

        return:
            euclidean distance between two nodes
        '''
        ### YOUR CODE HERE ###
        euc_dis = np.sqrt((node2.row-node1.row)**2 + (node2.col-node1.col)**2)
        return euc_dis

    
    def check_collision(self, node1, node2):
        '''Check if the path between two nodes collide with obstacles
        arguments:
            node1 - node 1
            node2 - node 2

        return:
            True if the new node is valid to be connected
        '''
        ### YOUR CODE HERE ###
        obs = False
        p1_x = node1.row
        p1_y = node1.col
        p2_x = node2.row
        p2_y = node2.col
        path_x=[]
        path_y=[]

        if p1_x > p2_x:
            start_x = p2_x
            end_x = p1_x
        else:
            start_x = p1_x
            end_x = p2_x
        if p1_y > p2_y:
            start_y = p2_y
            end_y = p1_y
        else:
            start_y = p1_y
            end_y = p2_y
        path_x_temp = [i for i in range(start_x, end_x+1)]
        path_y_temp = [j for j in range(start_y, end_y+1)]

        if(len(path_x_temp)==1 and len(path_y_temp)==1):
            return True

        if len(path_x_temp) > len(path_y_temp):
            path_x = path_x_temp
            for x in path_x:
                path_y.append(int(((p2_y-p1_y)/(p2_x-p1_x))*(x-p1_x) + p1_y))
        else:
            path_y = path_y_temp
            for y in path_y:
                path_x.append(int(((y-p1_y)*((p2_x-p1_x)/(p2_y-p1_y)))+p1_x))

        for (i,j) in zip(path_x,path_y):
            if self.map_array[i][j]==0:
                obs = True
        return obs


    def get_new_point(self, goal_bias):
        '''Choose the goal or generate a random point
        arguments:
            goal_bias - the possibility of choosing the goal instead of a random point

        return:
            point - the new point
        '''
        ### YOUR CODE HERE ###
        prob = np.random.random()
        if prob > goal_bias:
            point = Node(np.random.randint(0,self.size_row,1)[0],np.random.randint(0,self.size_col,1)[0])
        else:
            point = self.goal
        # print(point)
        # print(np.random.randint(0,self.size_col,1)[0])
        return point

    
    def get_nearest_node(self, point):
        '''Find the nearest node in self.vertices with respect to the new point
        arguments:
            point - the new point

        return:
            the nearest node
        '''
        ### YOUR CODE HERE ###
        kdtree = spatial.KDTree([(v.row,v.col) for v in self.vertices])

        dist,ind = kdtree.query((point.row,point.col),1) 

        return self.vertices[ind]


    def get_neighbors(self, new_node, neighbor_size):
        '''Get the neighbors that are within the neighbor distance from the node
        arguments:
            new_node - a new node
            neighbor_size - the neighbor distance

        return:
            neighbors - a list of neighbors that are within the neighbor distance 
        '''
        ### YOUR CODE HERE ###
        kdtree = spatial.KDTree([(v.row,v.col) for v in self.vertices])

        n = kdtree.query_ball_point((new_node.row,new_node.col),neighbor_size)
        neighbors=[]

        for i in n:
            if(not self.check_collision(self.vertices[i],new_node)):
                neighbors.append(i)
 
        return neighbors


    def rewire(self, new_node, neighbors):
        '''Rewire the new node and all its neighbors
        arguments:
            new_node - the new node
            neighbors - a list of neighbors that are within the neighbor distance from the node

        Rewire the new node if connecting to a new neighbor node will give least cost.
        Rewire all the other neighbor nodes.
        '''
        ### YOUR CODE HERE ###
        # print(neighbors)
        for neighbor in neighbors:
            # print(neighbor)
            neighbor_node = self.vertices[neighbor]
            cost = self.dis(neighbor_node, new_node) + new_node.cost
            if not self.check_collision(neighbor_node, new_node) and cost < neighbor_node.cost:
                neighbor_node.cost = cost
                neighbor_node.parent = new_node
                # Check if the rewiring caused a shorter path to the start node
                # self.update_cost_to_start(neighbor_node)




    
    def draw_map(self):
        '''Visualization of the result
        '''
        # Create empty map
        fig, ax = plt.subplots(1)
        img = 255 * np.dstack((self.map_array, self.map_array, self.map_array))
        ax.imshow(img)

        # Draw Trees or Sample points
        for node in self.vertices[1:-1]:
            plt.plot(node.col, node.row, markersize=3, marker='o', color='y')
            plt.plot([node.col, node.parent.col], [node.row, node.parent.row], color='y')
        
        # Draw Final Path if found
        if self.found:
            cur = self.goal
            while cur.col != self.start.col or cur.row != self.start.row:
                plt.plot([cur.col, cur.parent.col], [cur.row, cur.parent.row], color='b')
                cur = cur.parent
                plt.plot(cur.col, cur.row, markersize=3, marker='o', color='b')

        # Draw start and goal
        plt.plot(self.start.col, self.start.row, markersize=5, marker='o', color='g')
        plt.plot(self.goal.col, self.goal.row, markersize=5, marker='o', color='r')

        # show image
        plt.show()


    def RRT(self, n_pts=1000):
        '''RRT main search function
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        In each step, extend a new node if possible, and check if reached the goal
        '''
        # Remove previous result
        self.init_map()

        ### YOUR CODE HERE ###

        # In each step,
        # get a new point, 
        # get its nearest node, 
        # extend the node and check collision to decide whether to add or drop,
        # if added, check if reach the neighbor region of the goal.

        # print(self.goal.row,self.goal.col)

        for i in range(n_pts):
            # print(self.vertices)
            point = self.get_new_point(0.2)
            # print(point)
            # print(self.start)

            # nearest_node_idx = self.get_nearest_node(point)

            # nearest_node = self.vertices[nearest_node_idx]

            nearest_node = self.get_nearest_node(point)

            # print(nearest_node)
            # if (point != self.goal):
            distance = self.dis(point,nearest_node)
            if(distance>10):
                if(point == self.goal):
                    point1 = Node(point.row,point.col)
                    point = point1
                    # break
                point.row = int(nearest_node.row + (((point.row - nearest_node.row)/distance)*10))
                point.col = int(nearest_node.col + (((point.col - nearest_node.col)/distance)*10))

            # print(point)

            if (self.check_collision(point,nearest_node)==False):
                point.parent =  nearest_node  
                point.cost = point.parent.cost + self.dis(point,nearest_node)
                self.vertices.append(point)  

                if (point == self.goal):
                    self.found = True

        # print(self.vertices)
        # Output
        if self.found:
            steps = len(self.vertices) - 2
            length = self.goal.cost
            print("It took %d nodes to find the current path" %steps)
            print("The path length is %.2f" %length)
        else:
            print("No path found")
        
        # Draw result
        # print(self.goal.row,self.goal.col)

        self.draw_map()


    def RRT_star(self, n_pts=1000, neighbor_size=20):   
        '''RRT* search function
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points
            neighbor_size - the neighbor distance
        
        In each step, extend a new node if possible, and rewire the node and its neighbors
        '''
        # Remove previous result
        self.init_map()

        ### YOUR CODE HERE ###

        # In each step,
        # get a new point, 
        # get its nearest node, 
        # extend the node and check collision to decide whether to add or drop,
        # if added, rewire the node and its neighbors,
        # and check if reach the neighbor region of the goal if the path is not found.

        for i in range(n_pts):
            # print(self.vertices)
            point = self.get_new_point(0.2)
            # print(point)
            # print(self.start)

            # nearest_node_idx = self.get_nearest_node(point)

            # nearest_node = self.vertices[nearest_node_idx]

            nearest_node = self.get_nearest_node(point)

            # print(nearest_node)
            # if (point != self.goal):

            distance = self.dis(point,nearest_node)

            if(distance>10):
                if(point == self.goal):
                    point1 = Node(point.row,point.col)
                    point = point1
                    # break
                point.row = int(nearest_node.row + (((point.row - nearest_node.row)/distance)*10))
                point.col = int(nearest_node.col + (((point.col - nearest_node.col)/distance)*10))

            # print(point)

            neighbors = self.get_neighbors(point,neighbor_size)
            # print(neighbors)
            cost = nearest_node.cost + self.dis(point,nearest_node)
            for i in neighbors:
                if(self.vertices[i].cost + self.dis(point,self.vertices[i])<cost):
                    cost = self.vertices[i].cost + self.dis(point,self.vertices[i])
                    nearest_node = self.vertices[i]
            # point.parent =  nearest_node  
            # point.cost = point.parent.cost + self.dis(point,nearest_node)


            # self.rewire(point,neighbors)

            if (self.check_collision(point,nearest_node)==False):
                point.parent =  nearest_node  
                point.cost = point.parent.cost + self.dis(point,nearest_node)
                self.rewire(point,neighbors)
                self.vertices.append(point)  

                if (point == self.goal):
                    self.found = True

        # Output
        if self.found:
            steps = len(self.vertices) - 2
            length = self.goal.cost
            print("It took %d nodes to find the current path" %steps)
            print("The path length is %.2f" %length)
        else:
            print("No path found")

        # self.found = False
        # Draw result
        self.draw_map()
