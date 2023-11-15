
# import neccesry packges
import numpy as np 
from matplotlib import pyplot as plt 
from PIL import Image 
from math import sqrt
import sys

def is_onboard( position , map ):
    if((position[0] >= 0 and position[1] >= 0) and position[0]<len(map) and position[1]<len(map)):
        return True        
    else:
        return False   
    
def is_not_obstacle( position , map): # Checks a Position is obstacle or Not
    if(map[position] == 1):
        return False
    return True

def isValid( position , map): 
    if is_onboard(position , map ) and map[position] != 1 and  map[position] == 0 :
        return True
    return False

def get_distance(post1 , post2): # returns a distance based the connectivity we select 
    distance =  sqrt((post1[0] - post2[0])**2 + (post1[1] -post2[1])**2)
    return distance
        
def getX(path):# returns the row part of two dimensional array 
    x=[]       # we use this to draw path in a map
    for p in path:
        i,j = p
        x.append(i) 
    return x 
def getY(path):  # returns the colomun part of two dimensional array 
    y=[]         # we use this to draw path in a map
    for p in path:
        i,j = p
        y.append(j)
    return y

def A_Star(start,goal,map,motions):

    #initialization
    h_start = get_distance(start, goal) # huristic distnce the start node 
    g_distance = { start: 0 }  # distance from the start point to the child node
    f_distance = {start:h_start} # total distance calculated 
    open_list =  {start : h_start }
    closed_list = {start:0}
    parent_list = { start : start }
    
    
    # do until the closed list is empty
    while  len(open_list): 
        current_position = list(open_list)[0]   # get the first node from the listed o-list 
        open_list.pop(current_position) # remove from the sorted openlist 
        closed_list[current_position] = parent_list[current_position]   # add to the closed-list the one in the top of the open list
        # found the goal
        if(current_position == goal): # if goal is found return 
            print("goal found",current_position)
            break
                
        np.random.shuffle(motions)  
        # look for the children node      
        for m in motions:  
            successor_position = (current_position[0]+m[0] , current_position[1]+m[1])
            g = g_distance[current_position] + get_distance(successor_position,current_position)
            h = get_distance(successor_position, goal) # get the huristic distnace from the child node to the goal 
            f = g + h   
            
            if(isValid(successor_position, map)): 
                if(open_list.get(successor_position) and open_list.get(successor_position) < f ):
                    # if node is already in the open list with small distance ignore it
                    continue
                if(closed_list.get(successor_position) ):
                    # if node is already in the closed list ignore it
                    continue 
                else:     
                    ## update the closed and open list              
                    g_distance[successor_position] = g
                    f_distance[successor_position] = f
                    open_list[successor_position] = f
                    parent_list[successor_position] = current_position
                    
        open_list = dict(sorted(open_list.items(), key=lambda open_list: open_list[1])) # sort open postion by thier value        
       
       
    ## Searching the path from start to the goal using closed list    
    path = [goal]
    curre_node = goal
    while curre_node != start:
        curre_node = closed_list[curre_node]
        path.append(curre_node)
    path = path[::-1]
    
    return path,f_distance[goal]

def main():
    
    # set default file , start , goal
    file_grid = './gridmap/map0.png'
    start = (10,10)
    goal = (90,70)

    if(len(sys.argv)>1):  
        file_grid = sys.argv[1] 
        start = (int(sys.argv[2]),int(sys.argv[3]))
        goal = (int(sys.argv[4]) , int(sys.argv[5]))
        
      # Load grid map
    image = Image.open(file_grid).convert('L')
    grid_map = np.array(image.getdata()).reshape(image.size[0], image.size[1])/255 
    # binarize the image 
    grid_map[grid_map > 0.5] = 1
    grid_map[grid_map <= 0.5] = 0 
    # Invert colors to make 0 -> free and 1 -> occupied 
    grid_map = (grid_map * -1) + 1 # Show grid map 
    
    
    # using 4 point connectivity 
    motions = [(0,-1),(-1, 0),(0, 1),(1, 0)] # 4-point connectivity 
    path_4_point,path_distance_4_point = A_Star(start,goal,grid_map,motions)
    print("------using 4 poitnt connectivity------")
    print("path_distance:",path_distance_4_point)
    print("path:",path_4_point)
    # using 8 point connectivity 
    
    motions = [(0,-1),(-1, 0),(0, 1),(1, 0),(1,1),(-1,-1),(1,-1),(-1,1) ] # 8-point connectivity 
    path_8_point,path_distance_8_point = A_Star(start,goal,grid_map,motions)
    print("------using 8-point connectivity-------")
    print("path_distance:",path_distance_8_point)
    print("path:",path_8_point)
    
    
    # All Plotes Here 
    fig, axs = plt.subplots(1,2 ,)
    plt.subplots_adjust(left=0.1, right=0.9, top=0.9, bottom=0.1)    
    axs[0].matshow(grid_map)
    axs[0].plot(getY(path_4_point.copy()),getX(path_4_point.copy()),color='red')
    axs[0].scatter(start[1], start[0], s=200,marker='.', c='red', )
    axs[0].scatter(goal[1], goal[0], s=200,marker='*' ,c='red',)
    axs[0].set_title('Path of 4-point conn A* !')
    
    axs[1].matshow(grid_map)
    axs[1].plot(getY(path_8_point.copy()),getX(path_8_point.copy()),color='red')
    axs[1].scatter(start[1], start[0], s=200,marker='.', c='red', )
    axs[1].scatter(goal[1], goal[0], s=200,marker='*' ,c='red',)
    axs[1].set_title('Path of 8-point conn A* !')

    plt.show()
    
main()