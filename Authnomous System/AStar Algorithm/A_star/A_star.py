import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from collections import defaultdict
from typing import List
import csv
from PIL import Image  
import copy as copy
import math
import sys

LEFT        = (-1, 0)
RIGHT       = ( 1, 0) 
DOWN        = ( 0, 1) 
UP          = ( 0,-1)
LEFT_DOWN   = ( 1, 1)
RIGHT_UP    = (-1,-1)
RIGHT_DOWN  = ( 1,-1)
LEFT_UP     = (-1, 1)

class A_star:
    """
    A star class. Implements the A star algorithm.
    """  
    """
    Constant Definition
    """
    __INFI        = 100000000

    def __heauristic(self, cell, goal):
        """
        The heauristic function compute the Eucledian distance from the cell to the goal 
        ...
            
        ...

        Parameters
        -------------
        cell : array

        goal : array
            
        Returns
        -------------
            dist : float
        """
        [x1, y1] = cell
        [x2, y2] = goal
        # Compute Eucledian distance
        dist = ((x2-x1)**2 + (y2-y1)**2)**0.5

        return dist
    
    def __init__(self, graph, start, goal) -> None: 
        """
        Init function
        ...
          
        ...

        Parameters
        -------------
        Private variables:

        __graph     : an array [N x 3]
                    Array contains the number of the polygon the point is belongs to and the position of points in the map.
                    The first row is the starting point and the last row is the goal.
                    N: The number of point.

        __start     : an array
                    Sorted list of edges: S
        
        __goal      : an array [1 x 3]
                    Vertex v: start point

        __closed_set: an array [1 x 3]
                    Vertex vi: whose visibility will be tested

        __open_set  : a list
                    Edge list size M x 2, where M is the number of edges. Each row represents one edge, having the start and the end vertices index of the array of vertices.
        
        __came_from : an array
                    Distance
        
        __gscore    : a list
                    List edges in the visibility list. Each cell contains 2 elements, indexs of the starting point and the end point.
        
        __fscore    : int
                    Index of the next edge to be inserted.

        __A         : an array [1 x N-1]
                    Array contains the indices of the elements of softed alpha.
        Returns
        -------------

        """           
        self.__graph        = graph
        self.__start        = start
        self.__goal         = goal
        
        # nodes already evaluated
        self.__closed_set = []

        # The set of discovered nodes that may need to be (re-)expanded.
        # Initially, only the start node is known.
        self.__open_set = [self.__start]

        # For node n, cameFrom[n] is the node immediately preceding it on the
        # cheapest path from start to n currently known.
        self.__came_from = {}

        # For node n, gScore[n] is the cost of the cheapest path from start
        # to n currently known.
        self.__gscore = {} 
        # intialize cost for every node to inf
        for index in graph:
            self.__gscore[index] = self.__INFI 
        # intialize cost for start node to 0
        self.__gscore[self.__start] = 0

        # For node n, fScore[n] := gScore[n] + h(n). fScore[n] represents our
        # current best guess as to how short a path from start to finish can
        # be if it goes through n.
        self.__fscore = {}

        for index in graph:
            self.__fscore[index] = self.__INFI
        
        self.__fscore[self.__start] = self.__gscore[self.__start] + self.__heauristic(self.__start, self.__goal) 

        # Intialize the optimal path and its' score
        self.__optimal_score   = float()
        self.__optimal_path    = []

    def getOptimalPath(self):
        """
        A public function used to get the optimal Path attribute
        ...
            
        ...

        Parameters
        -------------
        
        Returns
        -------------
            __path: list
                List index of cells on the path
        """
        return self.__optimal_path
    
    def getOptimalScore(self):
        """
        A public function used to get the score attribute of the optimal path
        ...
            
        ...

        Parameters
        -------------
        
        Returns
        -------------
            __optimal_score: list
                float
        """
        return self.__optimal_score
    
    def compute(self):
        """
        The main function of the algorithm
        ...
            
        ...

        Parameters
        -------------
        self : 

        graph : dictionary, which contains connectivities of each cell in the map

        start : 1 by 2 array
            The index of starting point in the map
        goal  : 1 by 2 array
            The index of goal position in the map
            
        Returns
        -------------
            True : If finding the optimal path from the starting point to the goal
            False: If not find
        """

        while self.__open_set:
            min_val = self.__INFI  # find node in openset with lowest fscore value
            # Finding the node in openSet having the lowest fScore[] value
            for current_node in self.__open_set:
                if self.__fscore[current_node] < min_val:
                    min_val     = self.__fscore[current_node]
                    min_node    = current_node

            # set that node to current node
            current_node    = min_node 
            
            if current_node == self.__goal:
                self.__reconstruct_path(self.__came_from, current_node)
                self.__optimal_score = tentative_gscore
                return True
            
            
            # remove current node from set to be evaluated
            self.__open_set.remove(current_node)  

            # add it to set of evaluated nodes
            self.__closed_set.append(current_node)  

            # check neighbors of current node
            for neighbor_node in self.__graph[current_node]: 
                # Ignore neighbor node if its already evaluated
                if neighbor_node in self.__closed_set:  
                    continue
                
                # d(current,neighbour) is the weight of the edge from current to
                # neighbour tentative_gScore is the distance from start to the
                # neighbour through current
                d = self.__heauristic(current_node, neighbor_node) 

                # Distance from start to neighbor through current
                tentative_gscore = self.__gscore[current_node] + d

                # This path to neighbour is better than any previous one
                if tentative_gscore < self.__gscore[neighbor_node]:
                    # record the best path untill now
                    self.__came_from[neighbor_node]     = current_node  
                    self.__gscore[neighbor_node]        = tentative_gscore
                    self.__fscore[neighbor_node]        = self.__gscore[neighbor_node] + self.__heauristic(neighbor_node, self.__goal)
                    
                    # else add it to set of nodes to be evaluated
                    if neighbor_node not in self.__open_set:  
                        self.__open_set.append(neighbor_node)

        # Open set is empty but goal was never reached
        return False

    def __reconstruct_path(self, came_from, current_node):
        """
        The reconstruct path function
        ...
            
        ...

        Parameters
        -------------
        came_from       : array

        current_node    : array
            
        Returns
        -------------
            final_path : array
        """
        self.__optimal_path = [current_node]
        while current_node in came_from:
            current_node = came_from[current_node]
            self.__optimal_path.append(current_node)

class PrimsMaze:
    """
    PrimsMaze Generator class. Implements maze generator.
    """  
    def __init__(self, size=25, show_maze=True):
        self.size = (size // 2) * 2 + 1
        self.show_maze = show_maze
        self.walls_list = []
        self.grid = np.full((self.size, self.size), -50, dtype=int)
        for i in range(size//2+1):
            for j in range(size//2+1):
                self.grid[i*2, j*2] = -1
        self.maze = np.zeros((self.size, self.size), dtype=bool)
        print(self.grid)

    def is_valid(self, curr, dx, dy):
        x, y = curr
        if 0 <= x + dx < self.size and 0 <= y + dy < self.size:
            return True
        return False

    def add_neighbors(self, curr):
        nearby = [LEFT, DOWN, RIGHT, UP]
        for dx, dy in nearby:
            if self.is_valid(curr, dx, dy):
                self.walls_list.append((curr[0]+dx, curr[1]+dy))

    def create_maze(self, start):
        start = ((start[0]//2)*2, (start[1]//2)*2)
        self.grid[start[0], start[1]] = 1
        #self.grid[0, ::2] = self.grid[-1, ::2] = 1
        #self.grid[::2, 0] = self.grid[::2, -1] = 1
        self.add_neighbors(start)
        while len(self.walls_list):
            ind = np.random.randint(0, len(self.walls_list))
            wall_x, wall_y = self.walls_list[ind]
            if self.is_valid((wall_x, wall_y), -1, 0) and self.is_valid((wall_x, wall_y), 1, 0):
                top = wall_x-1, wall_y
                bottom = wall_x+1, wall_y
                if (self.grid[top] == 1 and self.grid[bottom] == -1):
                    self.grid[wall_x, wall_y] = 1
                    self.grid[bottom] = 1
                    self.add_neighbors(bottom)
                elif (self.grid[top] == -1 and self.grid[bottom] == 1):
                    self.grid[wall_x, wall_y] = 1
                    self.grid[top] = 1
                    self.add_neighbors(top)
                self.walls_list.remove((wall_x, wall_y))
            if self.is_valid((wall_x, wall_y), 0, 1) and self.is_valid((wall_x, wall_y), 0, -1):
                left = wall_x, wall_y-1
                right = wall_x, wall_y+1
                if (self.grid[left] == 1 and self.grid[right] == -1):
                    self.grid[wall_x, wall_y] = 1
                    self.grid[right] = 1
                    self.add_neighbors(right)
                elif (self.grid[left] == -1 and self.grid[right] == 1):
                    self.grid[wall_x, wall_y] = 1
                    self.grid[left] = 1
                    self.add_neighbors(left)
                self.walls_list.remove((wall_x, wall_y))

            ''' 
            '''
            if self.show_maze:
                img = self.grid                 # Display maze while building
                plt.figure(1)
                plt.clf()
                plt.imshow(img)
                plt.title('Maze')
                plt.pause(0.005)
                # #plt.pause(5)

        plt.pause(0.01)

        for row in range(self.size):
            for col in range(self.size):
                if self.grid[row, col] == 1:
                    self.maze[row, col] = True

        return self.maze

def gridMat2graph(mat, maze=False):
    """
        The function tranfer the grip map to one graph, which is applied the A star algorithm on
        ...
            
        ...

        Parameters
        -------------
        mat       : array
            The grid map
        Returns
        -------------
            graph : dict

    """
    rows = len(mat)
    cols = len(mat[0])
    graph = defaultdict(list)
    if maze == False:
        for x in range(rows):
            for y in range(cols):
                if mat[x][y] == False:
                    for dx, dy in [LEFT, RIGHT, DOWN, UP, LEFT_DOWN, RIGHT_UP, RIGHT_DOWN, LEFT_UP]:
                        if 0 <= x+dx < rows and 0 <= y+dy < cols and mat[x+dx][y+dy] == False:
                            graph[(x, y)].append((x+dx, y+dy))
    elif maze == True:
        for x in range(rows):
            for y in range(cols):
                if mat[x][y] == True:
                    for dx, dy in [LEFT, RIGHT, DOWN, UP]:
                        if 0 <= x+dx < rows and 0 <= y+dy < cols and mat[x+dx][y+dy] == True:
                            graph[(x, y)].append((x+dx, y+dy))
    return graph

def mat2graph(vertices, edges):
    """
        The function tranfer the map and the visibility graph to one graph, which is applied the A star algorithm on
        ...
            
        ...

        Parameters
        -------------
        mat       : array
            The map
        Returns
        -------------
            graph : dict

    """
    graph = defaultdict(list)
    vertex_num = len(vertices)
    
    for edge in edges:
        graph[vertices[edge[0]]].append(vertices[edge[1]])
        graph[vertices[edge[1]]].append(vertices[edge[0]])
        
    return graph

def load_vertices_from_file(filename: str):
    # list of vertices
    vertices: List = []
    with open(filename, newline='\n') as csvfile:
        v_data = csv.reader(csvfile, delimiter=",")
        next(v_data)
        for row in v_data:
            vertex = (float(row[1]), float(row[2]))
            vertices.append(vertex)
    return vertices

def load_edges_from_file(filename: str):
    edges = []
    with open(filename, newline='\n') as csvfile:
        reader = csv.reader(csvfile, delimiter=",")
        next(reader)
        for row in reader:
            edges.append((int(row[0]), int(row[1])))
    return edges

def plot_graph(vertices, edges, path):

    for v in vertices:
        plt.plot(v[0], v[1], 'r+')

    for e in edges:
        plt.plot([vertices[e[0]][0], vertices[e[1]][0]],
                 [vertices[e[0]][1], vertices[e[1]][1]],
                 "g--")

    for i, v in enumerate(vertices):
        plt.text(v[0] + 0.2, v[1], str(i))
    
    num_edges = len(path)
    for index in range(num_edges-1):
        plt.plot([path[index][0], path[index+1][0]],
                 [path[index][1], path[index+1][1]],
                 "b--")      
    plt.show()
        
# Main function    
if __name__ == "__main__" :
    """
    Part 1: A star with graph
    """
    # Create the graph from the edges and the visibility graph of this map
    
    file_verices = "./map/env_2.csv"
    file_edges= "./map/visibility_graph_env_2.csv"
    if(len(sys.argv)>2):  
        file_verices = sys.argv[1]
        file_edges = sys.argv[2] 
        
    vertices = load_vertices_from_file(file_verices)
    edges = load_edges_from_file(file_edges)
    
    graph = mat2graph(vertices, edges)
    # Get the goal and starting point of the map
    start = vertices[0]
    destination = vertices[-1]

    A_star_obj = A_star(graph, start, destination)

    A_star_obj.compute()
    # Get the optimal path
    optimal_path = A_star_obj.getOptimalPath()
    # Get the score of the optimal path
    optimal_score = A_star_obj.getOptimalScore()
    # Print results
    print("The list of vertices representing the optimal path:")
    print(optimal_path)
    print("The score of the optimal path")
    print(optimal_score)

    plot_graph(vertices, edges, optimal_path)
    """
    Part 2: A star with grid map
    """
    # Load grid map
    file_grid = "./gridmap/map0.png"
    goal_index        = (90,70)
    init_index        = (10,10)
    if(len(sys.argv)>3):  
        file_grid = sys.argv[3] 
        init_index = (int(sys.argv[4]),int(sys.argv[5]))
        print("start:",init_index)
        goal_index = (int(sys.argv[6]) , int(sys.argv[7]))
        
    image = Image.open(file_grid).convert('L')

    grid_map = np.array(image.getdata()).reshape(image.size[0],image.size[1])/255
    # binarize the image
    grid_map[grid_map > 0.5] = 1
    grid_map[grid_map <= 0.5] = 0
    # Invert colors to make 0 -> free and 1 -> occupied
    grid_map = (grid_map * -1) + 1
    # Convert to graph
    graph = gridMat2graph(grid_map)

    # Set the goal and starting point of the map 0
    # goal_index        = (90,70)
    # init_index        = (10,10)

    # Set the goal and starting point of the map 2
    # goal_index        = (139,38)
    # init_index        = (8,31)

    # Set the goal and starting point of the map 3
    # goal_index        = (375,375)
    # init_index        = (50,90)

    # Initialise an object of class A star   
    A_star_obj = A_star(graph, init_index, goal_index)
    # Compute A star
    A_star_obj.compute()
    # Get the optimal path
    optimal_path = A_star_obj.getOptimalPath()
    # Get the score of the optimal path
    optimal_score = A_star_obj.getOptimalScore()
    # Print results
    print("The list of vertices representing the optimal path:")
    print(optimal_path)
    print("The score of the optimal path")
    print(optimal_score)

    # Plot the map and the optimal path
    plt.matshow(grid_map)
    plt.plot([p[1] for p in optimal_path], [p[0] for p in optimal_path], color='red', linewidth=2)
    plt.plot(init_index[1], init_index[0], 'b*', markersize = 4)
    plt.plot(goal_index[1], goal_index[0], 'r*', markersize = 4)
    plt.colorbar() 
    plt.show()
    """
    Part 3: A star algorithm with the PrimsMaze generator
    """
    # Set the size of the map
    size            = 25
    # Set the goal and starting point of the map
    init_index      = (0, 0)
    goal_index      = (size-1, size-1)
    # Init the PrimsMaze generator
    prims_maze_obj  = PrimsMaze(size)
    mat             = prims_maze_obj.create_maze(init_index).tolist()
    # Plot prims maze generator
    plt.figure(figsize=(10, 5))
    plt.subplot(1, 2, 1)
    plt.imshow(mat, interpolation='nearest')
    plt.xticks([]), plt.yticks([])
    # Convert to graph
    graph = gridMat2graph(mat, True)


    # Initialise an object of class A star   
    A_star_obj = A_star(graph, init_index, goal_index)
    # Compute A star
    A_star_obj.compute()
    # Get the optimal path
    optimal_path = A_star_obj.getOptimalPath()
    # Get the score of the optimal path
    optimal_score = A_star_obj.getOptimalScore()


    # Print results
    print("The list of positions representing the optimal path:")
    print(optimal_path)
    print("The score of the optimal path")
    print(optimal_score)


    # Plot the map and the optimal path
    for x, y in optimal_path:
        mat[x][y] = 0.5

    plt.subplot(1, 2, 2)
    plt.imshow(mat)
    plt.show()
    
