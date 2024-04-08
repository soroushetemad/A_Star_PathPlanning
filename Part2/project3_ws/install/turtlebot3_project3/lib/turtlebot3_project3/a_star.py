# #!/usr/bin/env python3

# import numpy as np
# import cv2
# import queue
# import time
# from math import dist
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist

# rpm1 = 10
# rpm2 = 15

# ROBOT_RADIUS = 297 
# WHEEL_RADIUS = 33 

# map_width = 6000
# map_height = 2000
# threshold = 100
# POINT_SIZE = 5
# BUMPER_COLOR = (20, 20, 20)
# OBSTACLE_COLOR = (10, 100, 255)

# class Node:
#     def __init__(self, x, y, parent,theta,UL,UR, c2c, c2g, total_cost):
#         self.x = x
#         self.y = y
#         self.parent = parent
#         self.theta = theta
#         self.UL = UL
#         self.UR = UR
#         self.c2c = c2c
#         self.c2g = c2g
#         self.total_cost = total_cost
        
#     def __lt__(self,other):
#         return self.total_cost < other.total_cost

# # Define possible actions and associated cost increments
# def cost_fn(Xi, Yi, Thetai, UL, UR, Nodes_list, Path_list, obstacle_frame):

#     t = 0
#     r = WHEEL_RADIUS #40
#     L = ROBOT_RADIUS #160
#     dt = 0.1
#     cost = 0
#     Xn = Xi
#     Yn = Yi
#     Thetan = 3.14 * Thetai / 180

#     while t < 1:
#         t = t + dt
#         Xs = Xn
#         Ys = Yn
#         Xn += r*0.5 * (UL + UR) * np.cos(Thetan) * dt
#         Yn += r*0.5 * (UL + UR) * np.sin(Thetan) * dt
#         Thetan += (r / L) * (UR - UL) * dt
        
#         if Validity(Xn, Yn, obstacle_frame):
#             c2g = dist((Xs, Ys), (Xn, Yn))
#             cost = cost + c2g
#             Nodes_list.append((Xn, Yn))
#             Path_list.append((Xs, Ys))
#         else:
#             return None
    
#     Thetan = 180 * (Thetan) / 3.14

#     return [Xn, Yn, Thetan, cost, Nodes_list, Path_list]

# def Configuration_space():
#     CLEARANCE = clearance 
#     frame = np.full([map_height, map_width, 3], (255, 255, 255)).astype(np.uint8)
#     circle_center = (4200, 800)
#     radius = 600

#     # Add a black border around the image
#     frame[0:CLEARANCE, :] = BUMPER_COLOR  # Top border
#     frame[:, 0:CLEARANCE] = BUMPER_COLOR  # Left border
#     frame[-CLEARANCE:, :] = BUMPER_COLOR  # Bottom border
#     frame[:, -CLEARANCE:] = BUMPER_COLOR  # Right border

#     rectangle1 = np.array([
#         [(1500, 0), (1750, 0), (1750, 1000), (1500, 1000)]
#     ])
#     rect1_bumper = np.array([
#         [(1500 - CLEARANCE, 0), (1750 + CLEARANCE, 0), (1750 + CLEARANCE, 1000 + CLEARANCE), (1500 - CLEARANCE, 1000 + CLEARANCE)]
#     ])

#     rectangle2 = np.array([
#         [(2500, 2000), (2750, 2000), (2750, 1000), (2500, 1000)]
#     ])
#     rect2_bumper = np.array([
#         [(2500 - CLEARANCE, 2000), (2750 + CLEARANCE, 2000), (2750+CLEARANCE, 1000-CLEARANCE), (2500-CLEARANCE, 1000-CLEARANCE)]
#     ])

#     #cv2.fillPoly(frame, pts=walls_inflated, color=BUMPER_COLOR)
#     cv2.fillPoly(frame, pts=rect1_bumper, color= BUMPER_COLOR)
#     cv2.fillPoly(frame, pts=rectangle1, color= OBSTACLE_COLOR)
#     cv2.fillPoly(frame, pts=rect2_bumper, color= BUMPER_COLOR)
#     cv2.fillPoly(frame, pts=rectangle2, color= OBSTACLE_COLOR)
#     cv2.circle(frame, circle_center, radius + CLEARANCE, BUMPER_COLOR, -1)
#     cv2.circle(frame, circle_center, radius, OBSTACLE_COLOR, -1)
#     obstacle_frame = frame.copy()  # Creating a copy of frame for obstacle_frame
#     return obstacle_frame

# def Valid_Orient(theta):
#     if theta % 30 == 0:
#         return True
#     elif theta == 0:
#         return True
#     else:
#         return False

# def Validity(x, y, obstacle_frame):
#     # Check if coordinates are within the boundaries of the obstacle space
#     if x < 0 or x >= map_width or y < 0 or y >= map_height or np.array_equal(obstacle_frame[int(y), int(x)], OBSTACLE_COLOR) or np.array_equal(obstacle_frame[int(y), int(x)], BUMPER_COLOR):
#         return False
#     # If the cell is not occupied by an obstacle or bumper, it's considered valid
#     return True


# def Check_goal(present, goal):
#     distance_to_goal = dist((present.x, present.y), (goal.x, goal.y))
#     if distance_to_goal < threshold:
#         return True

# def a_star(start, goal, rpm1, rpm2, obstacle_frame):
#     start_time = time.time()
#     if Check_goal(start, goal):
#         return None, 1
#     goal_node = goal
#     start_node = start

#     moves = [[rpm1, 0], 
#              [0, rpm1], 
#              [rpm1, rpm1], 
#              [0, rpm2], 
#              [rpm2, 0], 
#              [rpm2, rpm2], 
#              [rpm1, rpm2],
#              [rpm2, rpm1]]
    
#     unexplored_nodes = {}  # Dictionary to store all open nodes
#     unexplored_nodes[(start_node.x, start_node.y)] = start_node

#     explored_nodes = {}  # Dictionary to store all closed nodes
#     priority_queue = queue.PriorityQueue()  # Priority queue to store nodes based on their priority
#     priority_queue.put((start_node.total_cost, start_node))  # Put the start node into the priority queue

#     Nodes_list = []  # Stores all nodes that have been traversed, for visualization purposes.
#     Path_list = []  # List to store all nodes that have been traversed, for visualization purposes

#     while not priority_queue.empty():
#         present_node = priority_queue.get()[1]  # Get the node with the lowest cost from the priority queue
#         # all_nodes.append([present_node.x, present_node.y, present_node.theta])

#         current_id = (present_node.x, present_node.y)
#         if Check_goal(present_node, goal_node):
#             goal_node.parent = present_node.parent
#             goal_node.total_cost = present_node.total_cost
#             print("Goal Node found")
#             end_time = time.time()  # End time for measuring time taken
#             time_taken = end_time - start_time
#             print("runtime:",time_taken)
#             return 1, Nodes_list, Path_list

#         if current_id in explored_nodes:
#             continue
#         else:
#             explored_nodes[current_id] = present_node

#         del unexplored_nodes[current_id]

#         for move in moves:
#             X1 = cost_fn(present_node.x, present_node.y, present_node.theta, move[0], move[1],
#                             Nodes_list, Path_list, obstacle_frame)
            
#             if X1 is not None:
#                 angle = X1[2]
#                 x = (round(X1[0] * 10) / 10)
#                 y = (round(X1[1] * 10) / 10)
#                 th = (round(angle / 15) * 15)
#                 c2g = dist((x, y), (goal.x, goal.y))

#                 new_node = Node(x, y, present_node, th, move[0], move[1], present_node.c2c + X1[3], c2g, present_node.c2c + X1[3] + c2g)
#                 new_node_id = (new_node.x, new_node.y)

#                 if not Validity(new_node.x, new_node.y, obstacle_frame):
#                     continue
#                 elif new_node_id in explored_nodes:
#                     continue

#                 if new_node_id in unexplored_nodes:
#                     if new_node.total_cost < unexplored_nodes[new_node_id].total_cost:
#                         unexplored_nodes[new_node_id].total_cost = new_node.total_cost
#                         unexplored_nodes[new_node_id].parent = new_node.parent
#                 else:
#                     unexplored_nodes[new_node_id] = new_node

#                 priority_queue.put((new_node.total_cost, new_node))  # Put the new node into the priority queue
            
#             # Explore nodes within a radius of 10 units from the current node
#             for coord in unexplored_nodes.copy():
#                 if dist(coord, current_id) <= 100:
#                     explored_nodes[coord] = True
#                     del unexplored_nodes[coord]
#     return 0, Nodes_list, Path_list

# def Backtrack(goal_node):  
#     x_path = []
#     y_path = []
#     theta_path = []
#     RPM_Left_Wheel = []
#     RPM_Right_Wheel = []

#     x_path.append(goal_node.x)
#     y_path.append(goal_node.y)
#     theta_path.append(goal_node.theta)
#     RPM_Left_Wheel.append(goal_node.UL)
#     RPM_Right_Wheel.append(goal_node.UR)
#     parent_node = goal_node.parent

#     while parent_node != -1:
#         x_path.append(parent_node.x)
#         y_path.append(parent_node.y)
#         theta_path.append(parent_node.theta)
#         RPM_Left_Wheel.append(parent_node.UL)
#         RPM_Right_Wheel.append(parent_node.UR)
#         parent_node = parent_node.parent
        
#     x_path.reverse()
#     y_path.reverse()
#     theta_path.reverse()
#     RPM_Left_Wheel.reverse()
#     RPM_Right_Wheel.reverse()

#     RPM_Left = np.array(RPM_Left_Wheel)
#     RPM_Right = np.array(RPM_Right_Wheel)
#     x = np.asarray(x_path)
#     y = np.asarray(y_path)
#     theta = np.array(theta_path)
#     return x,y,theta,RPM_Left,RPM_Right

# def publisher_node(x_path, y_path, RPM_Left, RPM_Right, theta_path):
#     rclpy.init()
#     node = rclpy.create_node('Astar_publisher')
#     publisher = node.create_publisher(Twist, '/cmd_vel', 10)
#     msg = Twist()

#     count = 0
#     r = node.create_rate(10)

#     for i in range(len(x_path)): 
#         while rclpy.ok():
#             if count == 101:
#                 msg.linear.x = 0
#                 msg.angular.z = 0
#                 publisher.publish(msg)
#                 break
#             else:
#                 # Assuming inputs_to_bot function is defined elsewhere
#                 velocity, angle = inputs_to_bot(x_path[i], y_path[i], theta_path[i], RPM_Left[i], RPM_Right[i])
#                 msg.linear.x = velocity * 10
#                 msg.angular.z = angle * 10
#                 publisher.publish(msg)
#                 count += 1
#                 r.sleep()
#         count = 0

# def inputs_to_bot(Xi, Yi, Thetai, UL, UR):
#     r = WHEEL_RADIUS
#     L = ROBOT_RADIUS
#     dt = 0.1  # Assuming a time step of 0.1 seconds
#     pi = 3.14

#     UL = UL * 2 * pi / 60  # Convert RPM to rad/s
#     UR = UR * 2 * pi / 60

#     thetan = pi * Thetai / 180  # Convert degrees to radians

#     theta_dot = (r / L) * (UR - UL)  # Angular velocity
#     theta_diff = thetan + theta_dot * dt  # Updated orientation

#     x_dot = (r / 2) * (UL + UR) * np.cos(theta_diff)  # Linear velocity in x-direction
#     y_dot = (r / 2) * (UL + UR) * np.sin(theta_diff)  # Linear velocity in y-direction

#     velocity_value = np.sqrt(x_dot ** 2 + y_dot ** 2)  # Magnitude of linear velocity
#     return velocity_value, theta_dot

# def act(args=None):
#     rclpy.init(args=args)
#     publisher_node = publisher_node(x_path, y_path, RPM_Left, RPM_Right, theta_path)
#     print("Spinning publisher node...")
#     rclpy.spin(publisher_node)
#     publisher_node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':

#     s_x, s_y, e_x, e_y = 500,1000,5750,250
#     clearance = 10
#     obstacle_frame = Configuration_space()
#     start_theta = 0
            
#     c2g = dist((s_x,map_height - s_y), (e_x, map_height - e_y))
#     total_cost =  c2g
#     start_node = Node(s_x, map_height - s_y,-1,start_theta,0,0,0,c2g,total_cost)
#     goal_node = Node(e_x, map_height - e_y, -1,0,0,0,c2g,0,total_cost)
#     timer_begin = time.time()
#     found_goal,Nodes_list,Path_list = a_star(start_node, goal_node, rpm1, rpm2, obstacle_frame)
#     timer_end = time.time()
#     print("Time taken to explore:", timer_end - timer_begin, "seconds")

#     if found_goal:
#         # Generate shortest path
#         print("Goal node found!!")
#         x_path,y_path,theta_path,RPM_Left,RPM_Right = Backtrack(goal_node)
#         print("Shortest Path: ", x_path, y_path)
#     else:
#         print("Goal not found.")

#     # Draw explored nodes and paths
#     for i in range(len(Path_list)):
#         # Get the coordinates of the parent node and the present node
#         parent_node = Path_list[i]
#         present_node = Nodes_list[i]
        
#         # Draw a line segment from the parent node to the present node
#         cv2.line(obstacle_frame, (int(parent_node[0]), int(parent_node[1])), (int(present_node[0]), int(present_node[1])), (0, 255, 0), 1)
#         # Draw circles at the present node
#         cv2.circle(obstacle_frame, (int(present_node[0]), int(present_node[1])), POINT_SIZE, (0, 255, 0), -1)


#     # Draw start and end points
#     cv2.circle(obstacle_frame, (s_x, map_height - s_y),20, (0, 255, 255), -1)  # Green circle for start point
#     cv2.circle(obstacle_frame, (e_x, map_height - e_y), 20, (0, 0, 255), -1)  # Red circle for end point

#     for i in range(len(x_path) - 1):
#         # Convert coordinates to integers
#         start_point = (int(x_path[i]), int(y_path[i]))
#         end_point = (int(x_path[i + 1]), int(y_path[i + 1]))
        
#         # Draw line segment
#         cv2.line(obstacle_frame, start_point, end_point, (255, 0, 0), 2)

#     #cv2.imshow("Map with Path", obstacle_frame)
#     cv2.imwrite("FRAME.jpg", obstacle_frame)
#     publisher_node(x_path,y_path,RPM_Left,RPM_Right,theta_path)
#     act()


import numpy as np
import math
import cv2
import queue
import time
from math import dist

# defining the search space(map) and other global variables
ROBOT_RADIUS = 220 
rpm1,rpm2 = 10, 14
map_width = 6000
map_height = 2000
threshold = 100
POINT_SIZE = 5
BUMPER_COLOR = (20, 20, 20)
OBSTACLE_COLOR = (10, 100, 255)
clearance = 10


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
    Thetan = 3.14 * Thetai / 180  # Convert orientation from degrees to radians
 
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
    
    Thetan = 180 * (Thetan) / 3.14
    return [Xn, Yn, Thetan, cost, Nodes_list, Path_list]

# Configuring the obstacle space and constructing the obstacles
def Configuration_space():
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
            
            # Explore nodes within a radius of 10 units from the current node
            for coord in unexplored_nodes.copy():
                if dist(coord, current_id) <= 100:
                    explored_nodes[coord] = True
                    del unexplored_nodes[coord]
    return 0, Nodes_list, Path_list

def Backtrack(goal_node):  
    x_path = []
    y_path = []
    theta_path = []
    RPM_Left_Wheel = []
    RPM_Right_Wheel = []

    x_path.append(goal_node.x)
    y_path.append(goal_node.y)
    theta_path.append(goal_node.theta)
    RPM_Left_Wheel.append(goal_node.UL)
    RPM_Right_Wheel.append(goal_node.UR)
    parent_node = goal_node.parent

    while parent_node != -1:
        x_path.append(parent_node.x)
        y_path.append(parent_node.y)
        theta_path.append(parent_node.theta)
        RPM_Left_Wheel.append(parent_node.UL)
        RPM_Right_Wheel.append(parent_node.UR)
        parent_node = parent_node.parent
        
    x_path.reverse()
    y_path.reverse()
    theta_path.reverse()
    RPM_Left_Wheel.reverse()
    RPM_Right_Wheel.reverse()

    RPM_Left = np.array(RPM_Left_Wheel)
    RPM_Right = np.array(RPM_Right_Wheel)
    x = np.asarray(x_path)
    y = np.asarray(y_path)
    theta = np.array(theta_path)
    return x,y,theta,RPM_Left,RPM_Right


if __name__ == '__main__':
    # Get clearance of the obstacle
    s_x, s_y, e_x, e_y = 500,1000,5750,250
    start_theta = 0
    clearance = 10

    obstacle_frame = Configuration_space()
    c2g = dist((s_x,map_height - s_y), (e_x, map_height - e_y))
    total_cost =  c2g
    start_node = Node(s_x, map_height - s_y,-1,start_theta,0,0,0,c2g,total_cost)
    goal_node = Node(e_x, map_height - e_y, -1,0,0,0,c2g,0,total_cost)
    found_goal,Nodes_list,Path_list = a_star(start_node, goal_node, rpm1, rpm2, obstacle_frame)

    if found_goal:
        # Generate shortest path
        x_path, y_path ,theta,RPM_Left, RPM_Right= Backtrack(goal_node)
        print("total cost:", total_cost)
    else:
        print("Goal not found.")


 
