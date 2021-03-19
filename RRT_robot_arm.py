# -*- coding: utf-8 -*-
"""
Created on Tue Dec  3 10:43:00 2019
ASEN 5519
Algorithmic Motion Planning
Final Project

@author: Shrivatsan K.
"""

"""
Problem Statement: Take the Wolf, Goat and Cabbage from Start Position (2,0) to Final position (-2,0)
Constraints : Goat and Cabbage cannot be left alone together
              Goat and Wolf cannot be left alone together  

The program uses Djikstra's algorithm (to find the sequence of actions) and RRT (to motion plan for robot arm)
to solve the problem

Input : Graph for Task Planning, Link Lengths of robot arm, Obstacle locations in Workspace
Output : Visual Display of solution to Problem Statement respecting the constraints 
         Wolf is a silver colored square
         Goat is a golden colored square
         Cabbage is a green colored square
         Obstacles are shown in Blue
"""
import matplotlib.pyplot as plt
from random import uniform
from random import randint
from shapely.geometry import Polygon
from shapely.geometry import Point
from shapely.geometry import LineString
import numpy as np
import matplotlib.colors as mcolors


class Poly: #defining all obstacles as objects of this class
        
    def __init__(self, n ):
       
        self.no_of_vertices = n   # user specified number of vertices for an obstacle
        self.vertices = []     # Create a list to store all vertices
        self.color = "royalblue"
    
    def create_poly(self):
        self.poly = Polygon(self.vertices)
    
    def display_poly(self):
        
        poly_x, poly_y = self.poly.exterior.xy  #this method is defined in class Polygon
        plt.fill(poly_x, poly_y, color = self.color)
        
    def is_in_poly(self, line):
        return line.intersects(self.poly)   # binary predicate defined in shapely module

"""
Subroutine: Performs search on graph using Djisktra's algorithm
Input: Task planning graph
Output: Path from Start vertex to Goal vertex
"""
def djisktra(graph):
              
        visited_nodes = []          #list to keeps track of all the nodes that have been visited
        priority_queue = []         #list to keeps track of shortest path
        visited_nodes.append((0,0,0))           #visit the start node
        for i in graph[(0,0,0)]:    #add the neighbors of start   
                f = i[1]  
                priority_queue.append(([(0,0,0),i[0]],f))
                
        priority_queue.sort(key=lambda tup: tup[1])     #sort the nodes according to cost function or priority
        
        while priority_queue:
                
                if(priority_queue[0][0][len(priority_queue[0][0])-1] == (1,1,1)):     #if goal is the first element, then shortest path to goal has been found
                    break
                
                else:            
                    
                    for i in graph[priority_queue[0][0][len(priority_queue[0][0])-1]]:         #add the neighbors of the first node in priority queue
                        if i[0] == (0,0,0):         #fisrt run of this will cause start to be classified as neighbor. Dont add start to the queue
                            continue
                        if i[0] in visited_nodes and i[1] > priority_queue[0][1]:  #if a shorter path has been found for a node that has already been visited, then add it to the queue
                            continue
                        else:
                            f = priority_queue[0][1] + i[1]   #calculating priority
                            visited_nodes.append(i[0])
                            path = []
                            for elements in priority_queue[0][0]:
                                path.append(elements)
                            path.append(i[0])
                            #path.append(f)
                            priority_queue.append((path,f))
                
                    priority_queue.pop(0)
                    priority_queue.sort(key=lambda tup: tup[1])     #sort the nodes according to cost function or priority
        
        return priority_queue[0][0]


"""
Subroutine: Check for collision of Arm with obstacles
Input: Joint angles
Output: Boolean which indicates collision
"""
def check_collision(thetha1, thetha2):
    
    collision_flag = 0  # using a variable to keep track of collision
    
    first_joint_x, end_effector_x, first_joint_y, end_effector_y = link(l1,l2,thetha1, thetha2)
    
    first_link = LineString([(0,0),(first_joint_x,first_joint_y)]) #creating a line object from shapely module
    second_link = LineString([(first_joint_x,first_joint_y), (end_effector_x,end_effector_y)])
    Object = Polygon([(end_effector_x-0.125,end_effector_y-0.125),(end_effector_x+0.125,end_effector_y-0.125),(end_effector_x+0.125,end_effector_y+0.125),(end_effector_x-0.125,end_effector_y+0.125)])
    for o in obstacles:
            
        if (o.is_in_poly(first_link))|(o.is_in_poly(second_link)|o.is_in_poly(Object)):
            collision_flag = 1  #set flag if collision occurs
            
        
    return collision_flag

"""
Subroutine: Find End Effector and joint coordinates
Input: Joint angles and link length
Output: End Effector and Joint coordinates
"""

def link(l1,l2,thetha1, thetha2): 
    
    first_joint_x = l1*np.cos(thetha1) #calculate x-coordinate of first joint
    
    end_effector_x = first_joint_x + l2*np.cos(thetha1+thetha2) #calculate x-coordinate of end effector
    
    first_joint_y = l1*np.sin(thetha1) #calculate y-coordinate of first joint
    
    end_effector_y = first_joint_y + l2*np.sin(thetha1+thetha2)  #calculate y-coordinate ofend effector
    
    return  (first_joint_x, end_effector_x, first_joint_y, end_effector_y)

"""
Subroutine: Random Sample Generator
Input: Goal node
Output: Random sample in collision free space
"""
def sample_config_space(goal):
    
    collision_flag = 1
    
    while(collision_flag):
                
            thetha1_sample = uniform (0,2*np.pi)    # Get a random sample between 0 and 2*pi
            thetha2_sample = uniform (0,2*np.pi)
        
            collision_flag = check_collision(thetha1_sample,thetha2_sample)     # Reject samples in obstacle region
    
    if(randint(0,100)<5):       # Sample goal with 5 percent probability
        return goal
    
    else:
        return(thetha1_sample, thetha2_sample)

"""
Subroutine: Add new branches to tree
Input: Two nodes to be added in the tree
"""
def make_tree(node1, node2):
    if node1 not in tree:
        tree[node1] = []     # add edges in an adjacency list manner with vertices as the keys
    tree[node1].append(node2)    

"""
Subroutine: check if sub path is collision free
Input: two joint angle configurations
Output: Boolean indicating whether subpath between the two configurations is collision free
"""

def check_path(node1, node2):
    col_flag = 0
    t = np.linspace(0,1,10)   # Break the path into 10 waypoints
    for i in t:
        thetha_inc = (1-i)*node1 + i*node2
        col_flag = check_collision(thetha_inc[0],thetha_inc[1])     # Check for collision at each way point
        if(col_flag == 1):
            break
    return col_flag    

"""
Subroutine: Generate RRT to find morion plan from start to goal
Input: goal
Output: Tree with root at start anf goal as one of the leaves
"""

def RRT(goal):
    
    converge = 1
    while(converge == 1):
           
                    
            thetha_sample = sample_config_space(goal) # Get a sample in collision free space
                
            ### Grow Start Side Tree
            
            nearest = np.sqrt(2)*2*np.pi
            
            for element in tree.keys():
                dist = Point(thetha_sample).distance(Point(element))
                if(dist < nearest):         # Find the node in the tree closest to the random sample
                    nearest = dist
                    nearest_node = element
            
            if(nearest<epsilon):
                update_thetha = np.array(thetha_sample) # if within epsilon radius, take step to the random sample
            else:       # else take a step of epsilon
                
                update_thetha = (1-(0.1/nearest))*np.array(nearest_node) + (0.1/nearest)*np.array(thetha_sample)    
                
            
            valid_path = check_path(np.array(nearest_node), update_thetha)  # Check if subpath is collision free
            
            if(valid_path == 0):
                make_tree(nearest_node, (update_thetha[0],update_thetha[1]))        # if subpath is collision free, add node to tree
                make_tree((update_thetha[0],update_thetha[1]), nearest_node)
                d = Point((update_thetha[0],update_thetha[1])).distance(Point(goal))
                print("distance to goal: " ,d)
                if(update_thetha[0] == goal[0]) & (update_thetha[1] == goal[1]):        #converge if goal is reached
                    converge = 0            
    return 
"""
Subroutine: Generate path between start and goal in RRT using Breadth First Search
input: tree generated using RRT
Output: List of parent nodes
"""
def generate_path(graph): 
    queue = []
    visited_nodes = []          #to keep track of visited nodes
    trace_back = {}
    
    queue.append(start)
    visited_nodes.append(start)

    for neighbor in graph[start]:           #add start to queue
        queue.append(neighbor)
        visited_nodes.append(neighbor)
        trace_back[neighbor] = queue[0]
    
    queue.pop(0)    
        
    while queue:                        # Run till queue becomes empty
        
        if goal in queue:               # If goal is in queue, path has been found
            break
        
        for neighbor in graph[queue[0]]:
            if neighbor in visited_nodes:
                continue
            else:
                queue.append(neighbor)          #add nearest neighbors
                visited_nodes.append(neighbor)
                trace_back[neighbor] = queue[0]     #keep track of parent nodes
        queue.pop(0)
    return trace_back
    
"""
Subroutine: Find sequence of states to reach between start from goal in RRT
Input: Node you want to reach, list of parent nodes
output: path from start to goal node 
"""
def find_path(current_node,trace_back):
        path = []
        while(1):
            prev_node = trace_back[current_node]            # Use parent nodes to get to start
            path.append(current_node)       
            if(prev_node == start):
                path.append(prev_node)
                break
            else:
                current_node = prev_node
        return path
    
"""
Subroutine: Combine Task Planning and Motion Planning
Input: Task Planning action sequence, joint angle configurations to reach from start to goal
Output: Sequence of Robot arm configurations and sequence in which objects have to be picked up
"""
def Task_Motion_plan(action_sequence, go_to_goal, go_to_start):

        end_effector_state = 0
        trajectory = []     # Sequence of Robot arm configurations to complete the task plan
        pick_up = []        # Sequence in which objects change state
        
        for i in range(len(action_sequence)-1):
            transition = [0,0,0]
            transition[0] = action_sequence[i][0]^action_sequence[i+1][0]
            transition[1] = action_sequence[i][1]^action_sequence[i+1][1]
            transition[2] = action_sequence[i][2]^action_sequence[i+1][2]
            
            active = transition.index(max(transition)) # Find the object for which the state changes 
            
            if(action_sequence[i][active]==0):  # Object has to be taken to goal
                
                    
                if(end_effector_state == 0):        #Check if end effector is in start
                    trajectory.append(go_to_goal)
                    pick_up.append(active)
                    end_effector_state = 1    
                               
                    continue
                
                if(end_effector_state == 1):        # If end effector is in goal, bring it to start
                                                    # and then take object to goal    
                    trajectory.append(go_to_start)
                    pick_up.append('Nan')
                        
                    trajectory.append(go_to_goal)
                    pick_up.append(active)
                    
                    end_effector_state = 1    
                    continue
            
            if(action_sequence[i][active]==1):      # Object has to be taken to start
                
                
                if(end_effector_state == 0):       #Check if end effector is in start
                                                   # If it is, bring it to goal first and then take object to start  
                     trajectory.append(go_to_goal)
                     pick_up.append('Nan')
                     
                     trajectory.append(go_to_start)
                     pick_up.append(active)
                     
                     end_effector_state = 0
                     continue
                 
                if(end_effector_state == 1):       # Check if end effector is in goal 
                    
                    trajectory.append(go_to_start)
                    pick_up.append(active)
                    
                    end_effector_state = 0    
                    continue
        
        return trajectory, pick_up
        
"""
Subroutine: Visual display of motion planning
Input: Sequence of Robot arm configurations and sequence in which objects have to be picked up
"""
def disp_action(trajectory, pick_up, obstacles, Objects):
        
    for i in range(len(trajectory)):
        
        for coords in trajectory[i]:
             plt.clf()
             for o in obstacles:        # Show workspace
               o.display_poly()
            
             x1,x2,y1,y2 = link(l1,l2,coords[0], coords[1])     # Robot Arm animation
             plt.plot([0,x1],[0,y1],'brown')
             plt.plot([x1,x2],[y1,y2],'brown')
             plt.plot(x2,y2,color = 'brown',marker = 'o')
             
             if(pick_up[i] != 'Nan'):
                Objects[pick_up[i]].vertices = [(x2-0.125,y2-0.125),(x2+0.125,y2-0.125),(x2+0.125,y2+0.125),(x2-0.125,y2+0.125)]
                Objects[pick_up[i]].create_poly()
             
             for object in Objects:     # Object animation
                    object.display_poly()
                
             plt.draw()
             plt.pause(0.0001)

                 
"""
Defining Workspace
"""
obstacles = []      # list of workspace obstacles

o1 = Poly(4)        # Obstacles taken from Homework 2 Ques. 7.a.ii     
o1.vertices = [(-0.25,1.1),(-0.25,2),(0.25,2),(0.25,1.1)]
o1.create_poly()
obstacles.append(o1)

o2 = Poly(4)
o2.vertices =  [(-2,-2),(-2,-1.8),(2,-1.8),(2,-2)]
o2.create_poly()
obstacles.append(o2)

Objects = [] # List of objects, i.e. wolf, goat and cabbage
             # All of them are squares placed at start position       
Wolf = Poly(4)
Wolf.vertices = [(1.875,-0.125),(2.125,-0.125),(2.125,0.125),(1.875,0.125)]
Wolf.color = (0.75,0.75,0.75,0.5) # Wolf is silver in color
Wolf.create_poly()

Objects.append(Wolf)

Goat = Poly(4)
Goat.vertices = [(1.875,-0.125),(2.125,-0.125),(2.125,0.125),(1.875,0.125)]
Goat.color = (1,0.84,0,0.5) # Goat is golden in color
Goat.create_poly()

Objects.append(Goat)

Cabbage = Poly(4)
Cabbage.vertices = [(1.875,-0.125),(2.125,-0.125),(2.125,0.125),(1.875,0.125)]
Cabbage.color = (0,1,0,0.5) # Cabbage is green in color
Cabbage.create_poly()

Objects.append(Cabbage)

"""
Task Planning
"""
Task_Plan = {(0,0,0):[((0,1,0),1)],                       #defining graph
             (0,0,1):[((1,0,1),1),((0,1,1),1)],
             (0,1,0):[((1,1,0),1),((0,0,0),1),((0,1,1),1)],
             (1,0,0):[((1,1,0),1),((1,0,1),1)],
             (0,1,1):[((0,0,1),1),((0,1,0),1)],
             (1,0,1):[((0,0,1),1),((1,1,1),1),((1,0,0),1)],
             (1,1,0):[((0,1,0),1),((1,0,0),1)],
             (1,1,1):[((1,0,1),1)]}

action_sequence = djisktra(Task_Plan)          #Run Djisktra's algorithm on the graph to get the path

"""
Motion Planning
"""
start = (0,0)
goal = (np.pi,0)
l1 =1 
l2 =1
tree = {}
tree[start] = []
epsilon = 0.1
RRT(goal)
trace_back = generate_path(tree)
path = find_path(goal,trace_back)
go_to_start = path.copy()
path.reverse()
go_to_goal = path

"""
Task Planning + Motion Planning
"""
trajectory, pick_up = Task_Motion_plan(action_sequence, go_to_goal, go_to_start)

"""
Final Display
"""
disp_action(trajectory, pick_up, obstacles, Objects)
