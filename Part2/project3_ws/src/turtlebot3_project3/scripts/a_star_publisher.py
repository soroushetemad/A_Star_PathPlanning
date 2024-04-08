#!/usr/bin/env python3

#importing all necessary libraries
import rclpy
import math
import rclpy.node
from geometry_msgs.msg import Twist

#importing a_star from astar file
from astar import *


map_height = 2000
# Defining a class for controlling the robot
class RobotControl(rclpy.node.Node):

    def __init__(self):
        # Initialize the ROS node
        super().__init__('Robot_Control_node')
        # Creating a publisher to publish velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    # Method to publish linear and angular velocity commands to the robot
    def publish_velocity(self, linear_vel, angular_vel):
        velocity_message = Twist()

        velocity_message.linear.x = linear_vel
        velocity_message.angular.z = angular_vel
        
        # Publish the twist message
        self.cmd_vel_pub.publish(velocity_message)

    # Method to convert wheel RPM to angular and linear velocities
    def rpm_to_angular(self, rpm1, rpm2):
        r = 0.3372
        L = 0.2878
        omega_left = (rpm1 * 2 * math.pi) / 60.0
        omega_right = (rpm2 * 2 * math.pi) / 60.0

        angular_velocity = (r / L) * (omega_left - omega_right) 
        linear_velocity = (r / 2) * (omega_left + omega_right)

        return linear_velocity, angular_velocity


def main(args=None):
    rclpy.init(args=args)
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
            end_input_x, end_input_y = map(int, goal_coordinates.split())
            if not Validity(end_input_x, end_input_y, obstacle_frame):
                print("Goal node is out of bounds or within the obstacle. Please enter valid coordinates.")
                continue
            break
        except ValueError:
            print("Invalid input format. Please enter two integers separated by space.")
    
    start_x, start_y = 500, 1000
    end_x, end_y = end_input_x, end_input_y
    
    print(".........................................")
    print("Processing.....!!!!")
    print(".........................................")
    
    # Define start and goal nodes
    c2g = math.dist((start_x,start_y), (end_x, end_y))
    total_cost =  c2g
    r = robot_radius
    start_node = Node(start_x, start_y,-1,start_theta,0,0,0,c2g,total_cost)
    goal_node = Node(end_x, end_y, -1,0,0,0,c2g,0,total_cost)

    # Find the optimal path using A* algorithm
    found_goal, Nodes_list, Path_list = a_star(start_node, goal_node,RPM1,RPM2, obstacle_frame)

    if found_goal:
        x_path, y_path, theta, rpm1_left, rpm2_right = Backtrack(goal_node)
        print(x_path)
        total_cost = goal_node.total_cost  # Cost of the optimal path
        print("Final path cost:", total_cost)
        for i in range(len(x_path)):
            rpm1 = rpm1_left[i]
            rpm2 = rpm2_right[i]
            th = theta[i]
            linear_vel, angular_vel = node.rpm_to_angular(rpm1, rpm2)
            print(f"Step {i+1}: linear_vel = {linear_vel}, angular_vel = {angular_vel}")
            node.publish_velocity(linear_vel, angular_vel)
            time.sleep(1.03)
    
        node.publish_velocity(0.0, 0.0) # Stop the robot when the path execution is complete
    else:
        print("Goal not found!")

    node.destroy_node()  # Destroy the ROS node
    rclpy.shutdown()     # Shutdown the ROS client library

if __name__ == '__main__':
    main()