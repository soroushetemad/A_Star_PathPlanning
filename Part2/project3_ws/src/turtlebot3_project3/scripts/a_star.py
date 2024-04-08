import numpy as np
import math
import cv2
import queue
import time
from math import dist

# defining the search space(map) and other global variables
ROBOT_RADIUS = 220 
# rpm1 = 10
# rpm2 = 15
map_width = 6000
map_height = 2000
threshold = 100
POINT_SIZE = 5
BUMPER_COLOR = (20, 20, 20)
OBSTACLE_COLOR = (10, 100, 255)
# clearance = 10


# Defining a class for nodes in the graph
class Node:
    def __init__(self, x, y, parent,theta,UL,UR, c2c, c2g, total_cost):
        self.x = x
        self.y = y
        self.parent = parent
        self.theta = theta
        self.UL = UL
        self.UR = UR
        self.c2c = c2c
        self.c2g = c2g
        self.total_cost = total_cost
        
    def __lt__(self,other):
        return self.total_cost < other.total_cost

# Define possible actions and associated cost increments
def cost_fn(Xi, Yi, Thetai, UL, UR, Nodes_list, Path_list, obstacle_frame):
    '''
    Xi, Yi,Thetai: Input point's coordinates
    Xs, Ys: Start point coordinates for plot function
    Xn, Yn, Thetan: End point coordintes
    '''

    # Constants for differential drive motion model
    t = 0
    r = 33   # Wheel radius (mm)
    L = 287  # Wheelbase (mm)
    dt = 0.1 # Time step (s)
    cost = 0
    Xn = Xi  # Initialize new point's x-coordinate
    Yn = Yi  # Initialize new point's y-coordinate
    Thetan = math.pi * Thetai / 180  # Convert orientation from degrees to radians
 
    # Simulating motion over a small time interval (dt)
    while t < 1:
        t = t + dt
        Xs = Xn
        Ys = Yn
        Xn += r*0.5 * (UL + UR) * math.cos(Thetan) * dt
        Yn += r*0.5 * (UL + UR) * math.sin(Thetan) * dt
        Thetan += (r / L) * (UR - UL) * dt
        
        if Validity(Xn, Yn, obstacle_frame):
            c2g = dist((Xs, Ys), (Xn, Yn))
            cost = cost + c2g
            Nodes_list.append((Xn, Yn))  # Append new point to Nodes_list
            Path_list.append((Xs, Ys))   # Append previous point to Path_list
        else:
            return None
    
    Thetan = 180 * (Thetan) / math.pi
    return [Xn, Yn, Thetan, cost, Nodes_list, Path_list]

# Configuring the obstacle space and constructing the obstacles
def Configuration_space(clearance):
    CLEARANCE = clearance
    # Initialize an empty grid for obstacle space
    frame = np.full([map_height, map_width, 3], (255, 255, 255)).astype(np.uint8)
    circle_center = (4200, 800)
    radius = 600

    # Add a black border around the image
    frame[0:CLEARANCE, :] = BUMPER_COLOR  # Top border
    frame[:, 0:CLEARANCE] = BUMPER_COLOR  # Left border
    frame[-CLEARANCE:, :] = BUMPER_COLOR  # Bottom border
    frame[:, -CLEARANCE:] = BUMPER_COLOR  # Right border

    # adding the robot_radius to the clearance 
    frame[CLEARANCE:ROBOT_RADIUS, :] = (255,255,250) # Top border
    frame[:, CLEARANCE:ROBOT_RADIUS] = (255,255,250)  # Left border
    frame[-ROBOT_RADIUS:-CLEARANCE, :] = (255,255,250)  # Bottom border
    frame[:, -ROBOT_RADIUS:-CLEARANCE] = (255,255,250)  # Right border
    
    rectangle1 = np.array([
        [(1500, 0), (1750, 0), (1750, 1000), (1500, 1000)]
    ])
    rect1_clearance = np.array([
        [(1500 - CLEARANCE, 0), (1750 + CLEARANCE, 0), (1750 + CLEARANCE, 1000 + CLEARANCE), (1500 - CLEARANCE, 1000 + CLEARANCE)]
    ])
    rect1_robot = np.array([
        [(1500 - ROBOT_RADIUS, 0), (1750 + ROBOT_RADIUS, 0), (1750 + ROBOT_RADIUS, 1000 + ROBOT_RADIUS), (1500 - ROBOT_RADIUS, 1000 + ROBOT_RADIUS)]
    ])

    rectangle2 = np.array([
        [(2500, 2000), (2750, 2000), (2750, 1000), (2500, 1000)]
    ])
    rect2_clearance = np.array([
        [(2500 - CLEARANCE, 2000), (2750 + CLEARANCE, 2000), (2750+CLEARANCE, 1000-CLEARANCE), (2500-CLEARANCE, 1000-CLEARANCE)]
    ])
    rect2_robot = np.array([
        [(2500 - ROBOT_RADIUS, 2000), (2750 +ROBOT_RADIUS, 2000), (2750 + ROBOT_RADIUS, 1000 - ROBOT_RADIUS), (2500 - ROBOT_RADIUS, 1000 - ROBOT_RADIUS)]
    ])
    cv2.fillPoly(frame, pts=rect1_robot, color=(255, 255, 250))  # White robot radius
    cv2.fillPoly(frame, pts=rect1_clearance, color=(0, 0, 0))  # Black clearance
    cv2.fillPoly(frame, pts=rectangle1, color= OBSTACLE_COLOR)
    cv2.fillPoly(frame, pts=rect2_robot, color=(255, 255, 250))  # White robot radius
    cv2.fillPoly(frame, pts=rect2_clearance, color=(0, 0, 0))  # Black clearance
    cv2.fillPoly(frame, pts=rectangle2, color= OBSTACLE_COLOR)
    cv2.circle(frame, circle_center, radius + CLEARANCE+ ROBOT_RADIUS, (255,255,250), -1)
    cv2.circle(frame, circle_center, radius + CLEARANCE, BUMPER_COLOR, -1)
    cv2.circle(frame, circle_center, radius, OBSTACLE_COLOR, -1)
    obstacle_frame = frame.copy()  # Creating a copy of frame for obstacle_frame
    return obstacle_frame


def Valid_Orient(theta):
    if theta % 30 == 0:
        return True
    elif theta == 0:
        return True
    else:
        return False

def Validity(x, y, obstacle_frame):
    # Check if coordinates are within the boundaries of the obstacle space
    if x < 0 or x >= map_width or y < 0 or y >= map_height or np.array_equal(obstacle_frame[int(y), int(x)], OBSTACLE_COLOR) or np.array_equal(obstacle_frame[int(y), int(x)], BUMPER_COLOR) or np.array_equal(obstacle_frame[int(y), int(x)], (255,255,250)):
        return False
    # If the cell is not occupied by an obstacle or bumper, it's considered valid
    return True

def Check_goal(present, goal):
    distance_to_goal = dist((present.x, present.y), (goal.x, goal.y))
    if distance_to_goal < threshold:
        return True
    else:
        return False
    

def a_star(start, goal, rpm1, rpm2, obstacle_frame):
    start_time = time.time()
    if Check_goal(start, goal):
        return None, 1
    goal_node = goal
    start_node = start

    moves = [[rpm1, 0], 
             [0, rpm1], 
             [rpm1, rpm1], 
             [0, rpm2], 
             [rpm2, 0], 
             [rpm2, rpm2], 
             [rpm1, rpm2],
             [rpm2, rpm1]]
    
    unexplored_nodes = {}  # Dictionary to store all open nodes
    unexplored_nodes[(start_node.x, start_node.y)] = start_node

    explored_nodes = {}  # Dictionary to store all closed nodes
    priority_queue = queue.PriorityQueue()  # Priority queue to store nodes based on their priority
    priority_queue.put((start_node.total_cost, start_node))  # Put the start node into the priority queue

    Nodes_list = []  # Stores all nodes that have been traversed, for visualization purposes.
    Path_list = []  # List to store all nodes that have been traversed, for visualization purposes

    while not priority_queue.empty():
        present_node = priority_queue.get()[1]  # Get the node with the lowest cost from the priority queue
        # all_nodes.append([present_node.x, present_node.y, present_node.theta])

        current_id = (present_node.x, present_node.y)
        if Check_goal(present_node, goal_node):
            goal_node.parent = present_node.parent
            goal_node.total_cost = present_node.total_cost
            print("Goal Node found")
            end_time = time.time()  # End time for measuring time taken
            time_taken = end_time - start_time
            print("Time taken to explore:", time_taken, "seconds")
            return 1, Nodes_list, Path_list

        if current_id in explored_nodes:
            continue
        else:
            explored_nodes[current_id] = present_node

        del unexplored_nodes[current_id]

        for move in moves:
            X1 = cost_fn(present_node.x, present_node.y, present_node.theta, move[0], move[1],
                            Nodes_list, Path_list, obstacle_frame)
            
            if X1 is not None:
                angle = X1[2]
                x = (round(X1[0] * 10) / 10)
                y = (round(X1[1] * 10) / 10)
                th = (round(angle / 15) * 15)
                c2g = dist((x, y), (goal.x, goal.y))

                new_node = Node(x, y, present_node, th, move[0], move[1], present_node.c2c + X1[3], c2g, present_node.c2c + X1[3] + c2g)
                new_node_id = (new_node.x, new_node.y)

                if not Validity(new_node.x, new_node.y, obstacle_frame):
                    continue
                elif new_node_id in explored_nodes:
                    continue

                if new_node_id in unexplored_nodes:
                    if new_node.total_cost < unexplored_nodes[new_node_id].total_cost:
                        unexplored_nodes[new_node_id].total_cost = new_node.total_cost
                        unexplored_nodes[new_node_id].parent = new_node.parent
                else:
                    unexplored_nodes[new_node_id] = new_node

                priority_queue.put((new_node.total_cost, new_node))  # Put the new node into the priority queue
            
            # Explore nodes within a radius of 100 units from the current node
            for coord in unexplored_nodes.copy():
                if dist(coord, current_id) <= 100:
                    explored_nodes[coord] = True
                    del unexplored_nodes[coord]
    return 0, Nodes_list, Path_list

def Backtrack(goal_node):  
    x_path = []
    y_path = []
    theta_path = []
    RPM_Left = []
    RPM_Right = []

    x_path.append(goal_node.x)
    y_path.append(goal_node.y)
    theta_path.append(goal_node.theta)
    RPM_Left.append(goal_node.UL)
    RPM_Right.append(goal_node.UR)
    parent_node = goal_node.parent
   

    while parent_node != -1:
        x_path.append(parent_node.x)
        y_path.append(parent_node.y)
        theta_path.append(parent_node.theta)
        RPM_Left.append(parent_node.UL)
        RPM_Right.append(parent_node.UR)
        parent_node = parent_node.parent
        
    x_path.reverse()
    y_path.reverse()
    theta_path.reverse()
    RPM_Left.reverse()
    RPM_Right.reverse()

    RPM_Left = np.array(RPM_Left)
    RPM_Right = np.array(RPM_Right)
    x = np.asarray(x_path)
    y = np.asarray(y_path)
    theta = np.array(theta_path)
    # print(x,y,theta,RPM_Left, RPM_Right)
    
    return x,y,theta,RPM_Left,RPM_Right
