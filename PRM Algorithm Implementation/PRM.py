# Standard Algorithm Implementation
# Sampling-based Algorithms PRM

import matplotlib.pyplot as plt
import numpy as np
import networkx as nx
from scipy import spatial

# Class for PRM
class PRM:
    # Constructor
    def __init__(self, map_array):
        self.map_array = map_array            # map array, 1->free, 0->obstacle
        self.size_row = map_array.shape[0]    # map size
        self.size_col = map_array.shape[1]    # map size

        self.samples = []                     # list of sampled points
        self.graph = nx.Graph()               # constructed graph
        self.path = []                        # list of nodes of the found path


    def check_collision(self, p1, p2):
        '''Check if the path between two points collide with obstacles
        arguments:
            p1 - point 1, [row, col]
            p2 - point 2, [row, col]

        return:
            True if there are obstacles between two points
        '''
        ### YOUR CODE HERE ###
        obs = False
        p1_x = p1[0]
        p1_y = p1[1]
        p2_x = p2[0]
        p2_y = p2[1]
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


    def dis(self, point1, point2):
        '''Calculate the euclidean distance between two points
        arguments:
            p1 - point 1, [row, col]
            p2 - point 2, [row, col]

        return:
            euclidean distance between two points
        '''
        ### YOUR CODE HERE ###
        euc_dis = np.sqrt((point2[0]-point1[0])**2 + (point2[1]-point1[1])**2)
        # euc_dis = np.linalg.norm(point1 - point2)
        return euc_dis


    def uniform_sample(self, n_pts):
        '''Use uniform sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # Initialize graph
        self.graph.clear()

        ### YOUR CODE HERE ###
        num = np.floor(np.math.sqrt(n_pts))
        rows =[i for i in range(0,self.size_row,int((self.size_row/num)))]
        cols = [j for j in range(0,self.size_col,int(self.size_col/num))]
        for i in rows:
            for j in cols:
                if self.map_array[i,j] == 1:
                    self.samples.append((i,j))

    
    def random_sample(self, n_pts):
        '''Use random sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # Initialize graph
        self.graph.clear()

        ### YOUR CODE HERE ###
        rows = np.random.randint(0,self.size_row,n_pts)
        cols = np.random.randint(0,self.size_col,n_pts)
        for i in range(n_pts):
            if self.map_array[rows[i],cols[i]] == 1:
                self.samples.append((rows[i],cols[i]))
        


    def gaussian_sample(self, n_pts):
        '''Use gaussian sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # Initialize graph
        self.graph.clear()

        ### YOUR CODE HERE ###
        rows = np.random.randint(0,self.size_row,n_pts)
        cols = np.random.randint(0,self.size_col,n_pts)
        for i in range(n_pts):
            q1 = (rows[i],cols[i])
            q2_row = int(np.random.normal(q1[0],10,1))
            if q2_row>299:
                q2_row = 299
            if q2_row < 0:
                q2_row = 0

            q2_col = int(np.random.normal(q1[1],10,1))
            if q2_col>299:
                q2_col = 299
            if q2_col < 0:
                q2_col = 0
            
            if self.map_array[q1[0],q1[1]] == 1 and self.map_array[q2_row,q2_col] == 0:
                self.samples.append(q1)
            elif self.map_array[q1[0],q1[1]] == 0 and self.map_array[q2_row,q2_col] == 1:
                self.samples.append((q2_row,q2_col))
            else:
                continue
          


    def bridge_sample(self, n_pts):
        '''Use bridge sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # Initialize graph
        self.graph.clear()

        ### YOUR CODE HERE ###
        rows = np.random.randint(0,self.size_row,n_pts)
        cols = np.random.randint(0,self.size_col,n_pts)
        for i in range(n_pts):
            if self.map_array[rows[i],cols[i]] == 0:
                q1 = (rows[i],cols[i])

                q2_row = int(np.random.normal(q1[0],20,1))
                if q2_row>299:
                    q2_row = 299
                if q2_row < 0:
                    q2_row = 0

                q2_col = int(np.random.normal(q1[1],20,1))
                if q2_col>299:
                    q2_col = 299
                if q2_col < 0:
                    q2_col = 0

                if self.map_array[q2_row,q2_col] == 0:
                    mid = (int((q1[0]+q2_row)/2),int((q1[1]+q2_col)/2))
                    if self.map_array[mid[0],mid[1]] == 1:
                        self.samples.append(mid)


    def draw_map(self):
        '''Visualization of the result
        '''
        # Create empty map
        fig, ax = plt.subplots()
        img = 255 * np.dstack((self.map_array, self.map_array, self.map_array))
        ax.imshow(img)

        # Draw graph
        # get node position (swap coordinates)
        node_pos = np.array(self.samples)[:, [1, 0]]
        pos = dict( zip( range( len(self.samples) ), node_pos))
        pos['start'] = (self.samples[-2][1], self.samples[-2][0])
        pos['goal'] = (self.samples[-1][1], self.samples[-1][0])
        
        # draw constructed graph
        nx.draw(self.graph, pos, node_size=3, node_color='y', edge_color='y' ,ax=ax)

        # If found a path
        if self.path:
            # add temporary start and goal edge to the path
            final_path_edge = list(zip(self.path[:-1], self.path[1:]))
            nx.draw_networkx_nodes(self.graph, pos=pos, nodelist=self.path, node_size=8, node_color='b')
            nx.draw_networkx_edges(self.graph, pos=pos, edgelist=final_path_edge, width=2, edge_color='b')

        # draw start and goal
        nx.draw_networkx_nodes(self.graph, pos=pos, nodelist=['start'], node_size=12,  node_color='g')
        nx.draw_networkx_nodes(self.graph, pos=pos, nodelist=['goal'], node_size=12,  node_color='r')

        # show image
        plt.axis('on')
        ax.tick_params(left=True, bottom=True, labelleft=True, labelbottom=True)
        plt.show()


    def sample(self, n_pts=1000, sampling_method="uniform"):
        '''Construct a graph for PRM
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points
            sampling_method - name of the chosen sampling method

        Sample points, connect, and add nodes and edges to self.graph
        '''
        # Initialize before sampling
        self.samples = []
        self.graph.clear()
        self.path = []

        # Sample methods
        if sampling_method == "uniform":
            self.uniform_sample(n_pts)
        elif sampling_method == "random":
            self.random_sample(n_pts)
        elif sampling_method == "gaussian":
            self.gaussian_sample(n_pts)
        elif sampling_method == "bridge":
            self.bridge_sample(n_pts)

        ### YOUR CODE HERE ###

        # Find the pairs of points that need to be connected
        # and compute their distance/weight.
        # Store them as
        # pairs = [(p_id0, p_id1, weight_01), (p_id0, p_id2, weight_02), 
        #          (p_id1, p_id2, weight_12) ...]
        kdtree = spatial.KDTree(self.samples)
        pairs=[]
        for idx,elem in enumerate(self.samples):
            dist,ind = kdtree.query(elem,10)
            for j in ind:
                if self.check_collision(elem,self.samples[j]) == False:
                    pairs.append((idx,j,self.dis(elem,self.samples[j])))


        # Use sampled points and pairs of points to build a graph.
        # To add nodes to the graph, use
        # self.graph.add_nodes_from([p_id0, p_id1, p_id2 ...])
        # To add weighted edges to the graph, use
        # self.graph.add_weighted_edges_from([(p_id0, p_id1, weight_01), 
        #                                     (p_id0, p_id2, weight_02), 
        #                                     (p_id1, p_id2, weight_12) ...])
        # 'p_id' here is an integer, representing the order of 
        # current point in self.samples
        # For example, for self.samples = [(1, 2), (3, 4), (5, 6)],
        # p_id for (1, 2) is 0 and p_id for (3, 4) is 1.
        self.graph.add_nodes_from(range(len(self.samples)))
        self.graph.add_weighted_edges_from(pairs)

        # Print constructed graph information
        n_nodes = self.graph.number_of_nodes()
        n_edges = self.graph.number_of_edges()
        print("The constructed graph has %d nodes and %d edges" %(n_nodes, n_edges))


    def search(self, start, goal):
        '''Search for a path in graph given start and goal location
        arguments:
            start - start point coordinate [row, col]
            goal - goal point coordinate [row, col]

        Temporary add start and goal node, edges of them and their nearest neighbors
        to graph for self.graph to search for a path.
        '''
        # Clear previous path
        self.path = []
        
        # Temporarily add start and goal to the graph
        self.samples.append(start)
        self.samples.append(goal)

        # start and goal id will be 'start' and 'goal' instead of some integer
        self.graph.add_nodes_from(['start', 'goal'])

        ### YOUR CODE HERE ###

        # Find the pairs of points that need to be connected
        # and compute their distance/weight.
        # You could store them as
        # start_pairs = [(start_id, p_id0, weight_s0), (start_id, p_id1, weight_s1), 
        #                (start_id, p_id2, weight_s2) ...]
        
        start_pairs = []
        goal_pairs = []

        kdtree = spatial.KDTree(self.samples)

        dist,ind = kdtree.query(start,30)
        for j in ind:
            if self.check_collision(start,self.samples[j]) == False:
                start_pairs.append(('start',j,self.dis(start,self.samples[j])))

        dist,ind = kdtree.query(goal,30)
        for j in ind:
            if self.check_collision(goal,self.samples[j]) == False:
                goal_pairs.append(('goal',j,self.dis(goal,self.samples[j])))



            # kdtree = spatial.KDTree(self.samples)
            # neighbors = kdtree.query_pairs(20)
            # # print(neighbors)

            # # print(self.samples[0][1])
            # for i,j in neighbors:
            #     if j == len(self.samples)-2:
            #         if self.check_collision(self.samples[i],self.samples[j]) == False:
            #             start_pairs.append((i,'start',self.dis(self.samples[i],self.samples[j])))
            #     if j == len(self.samples)-1:
            #         if self.check_collision(self.samples[i],self.samples[j]) == False:
            #             goal_pairs.append((i,'goal',self.dis(self.samples[i],self.samples[j])))
            # # print(start_pairs)
            # # print(goal_pairs)

            

            # Add the edge to graph
            self.graph.add_weighted_edges_from(start_pairs)
            self.graph.add_weighted_edges_from(goal_pairs)
        
        # Seach using Dijkstra
        try:
            self.path = nx.algorithms.shortest_paths.weighted.dijkstra_path(self.graph, 'start', 'goal')
            path_length = nx.algorithms.shortest_paths.weighted.dijkstra_path_length(self.graph, 'start', 'goal')
            print("The path length is %.2f" %path_length)
        except nx.exception.NetworkXNoPath:
            print("No path found")
        
        # Draw result
        self.draw_map()

        # Remove start and goal node and their edges
        self.samples.pop(-1)
        self.samples.pop(-1)
        self.graph.remove_nodes_from(['start', 'goal'])
        self.graph.remove_edges_from(start_pairs)
        self.graph.remove_edges_from(goal_pairs)
        