#!/usr/bin/env python3

import rclpy   # ROS Client Library for Python
import math
import rclpy.node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener
from astar import *

# Define a class for controlling the robot
map_height = 2000

class RobotControl(rclpy.node.Node):

    def __init__(self):
        super().__init__('Robot_Controller_node') # Initialize the ROS node
        # Create a publisher to publish velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    # Method to publish linear and angular velocity commands to the robot
    def publish_velocity(self, velocity_linear, velocity_angular):
        velocity_message = Twist()

        # Set the linear and angular velocities in the message
        velocity_message.linear.x = velocity_linear
        velocity_message.angular.z = velocity_angular
        
         # Publish the velocity message
        self.cmd_vel_pub.publish(velocity_message)

    # Method to convert wheel RPM to angular and linear velocities
    def rpm_to_angular(self, rpm1, rpm2):
        r = 0.3372  # Radius of the wheels
        L = 0.2878  # Wheelbase of the robot
        l_omega = (rpm1 * 2 * math.pi) / 60.0
        r_omega =(rpm2 * 2 * math.pi) / 60.0

        velocity_angular= (r / L) * (l_omega - r_omega) 
        velocity_linear = (r / 2) * (l_omega + r_omega)

        return velocity_angular, velocity_linear


def main(args=None):       # Initialize the ROS client library
    rclpy.init(args=args)
    
    # Create an instance of the RobotControl class
    node = RobotControl()
    robot_radius = 220
    clearance = 6
    obstacle_frame= Configuration_space(clearance)
    
    RPM1 = 10
    RPM2 = 5
    start_theta = 0
    
    # Taking Goal Node coordinates as input from user
    while True:
        goal_coordinates = input("Enter coordinates for Goal Node (x y): ")
        try:
            end_x, end_y = map(int, goal_coordinates.split())
            if not Validity(end_x, end_y, obstacle_frame):
                print("Goal node is out of bounds or within the obstacle. Please enter valid coordinates.")
                continue
            break
        except ValueError:
            print("Invalid input format. Please enter two integers separated by space.")
    s_x, s_y = 500, 1000
    e_x, e_y = end_x, end_y
    
    if start_theta % 30 != 0:
        print("Please enter valid theta values. Theta should be a multiple of 30 degrees.")
        exit()

    print("Processing....!!!!")

    # Define start and goal nodes
    c2g = math.dist((s_x,s_y), (e_x, e_y))
    total_cost =  c2g
    r = robot_radius
    start_node = Node(s_x,s_y,-1,start_theta,0,0,0,c2g,total_cost)
    goal_node = Node(e_x, e_y, -1,0,0,0,c2g,0,total_cost)

    # Find the optimal path using A* algorithm
    found_goal, Nodes_list, Path_list = a_star(start_node, goal_node,RPM1,RPM2, obstacle_frame)

    if found_goal:
        x_path, y_path, theta, rpm1_left, rpm2_right = Backtrack(goal_node)
        total_cost = goal_node.total_cost  # Cost of the optimal path
        print(x_path)
        print("Final path cost:", total_cost)
        for i in range(len(x_path)):
            rpm1 = rpm1_left[i]
            rpm2 = rpm2_right[i]
            th = theta[i]
            velocity_linear, velocity_angular= node.rpm_to_angular(rpm1, rpm2)
            print(f"Step {i+1}: linear_velocity = {velocity_linear}, angular_velocity = {velocity_angular}")
            node.publish_velocity(velocity_linear, velocity_linear)
            time.sleep(1.03)
    
        node.publish_velocity(0.0, 0.0) # Stop the robot when the path execution is complete
    else:
        print("Goal not found!")

    node.destroy_node()   # Destroy the ROS node
    rclpy.shutdown()      # Shutdown the ROS client library

if __name__ == '__main__':
    main()
    
