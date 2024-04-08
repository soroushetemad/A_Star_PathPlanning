#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import a_star as ft

class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('velocity_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.1  # 10Hz
        self.timer = self.create_timer(timer_period, self.publish_velocity)

        # Constants for differential drive motion model
        self.r = 33   # Wheel radius (mm)
        self.L = 287  # Wheelbase (mm)

        # Get clearance of the obstacle
        while True:
            CLEARANCE = input("Assign Clearance to the Obstacles: ")
            try:
                CLEARANCE = int(CLEARANCE)
                break
            except ValueError:
                print("Invalid input format. Please enter an integer.")

        self.obstacle_frame = ft.Configuration_space(CLEARANCE)

        # Taking start node coordinates as input from user
        while True:
            start_coordinates = input("Enter coordinates for Start Node (x y): ")
            try:
                s_x, s_y = map(int, start_coordinates.split())
                if not ft.Validity(s_x, s_y, self.obstacle_frame):
                    print("Start node is out of bounds or within the obstacle. Please enter valid coordinates.")
                    continue
                break
            except ValueError:
                print("Invalid input format. Please enter two integers separated by space.")

        # Taking start node orientation as input from user
        while True:
            start_theta = input("Enter Orientation of the robot at start node (multiple of 30): ")
            try:
                start_theta = int(start_theta)
                if not ft.Valid_Orient(start_theta):
                    print("Start orientation has to be a multiple of 30")
                    continue
                break
            except ValueError:
                print("Invalid input format. Please enter an integer.")

        # Taking Goal Node coordinates as input from user
        while True:
            goal_coordinates = input("Enter coordinates for Goal Node (x y): ")
            try:
                e_x, e_y = map(int, goal_coordinates.split())
                if not ft.Validity(e_x, e_y, self.obstacle_frame):
                    print("Goal node is out of bounds or within the obstacle. Please enter valid coordinates.")
                    continue
                break
            except ValueError:
                print("Invalid input format. Please enter two integers separated by space.")

        c2g = ft.dist((s_x, ft.map_height - s_y), (e_x, ft.map_height - e_y))
        total_cost = c2g
        start_node = ft.Node(s_x, ft.map_height - s_y, -1, start_theta, 0, 0, 0, c2g, total_cost)
        goal_node = ft.Node(e_x, ft.map_height - e_y, -1, 0, 0, 0, c2g, 0, total_cost)
        found_goal, Nodes_list, Path_list = ft.a_star(start_node, goal_node, self.obstacle_frame)

        if found_goal:
            self.x_path, self.y_path, _ = ft.Backtrack(goal_node)
            self.path_index = 0
        else:
            self.get_logger().info("Goal not found.")

    def publish_velocity(self):
        if hasattr(self, 'x_path') and hasattr(self, 'y_path'):
            if self.path_index < len(self.x_path) - 1:
                current_node = (self.x_path[self.path_index], self.y_path[self.path_index])
                next_node = (self.x_path[self.path_index + 1], self.y_path[self.path_index + 1])

                # Calculate linear and angular velocities based on the current and next nodes
                rpm1, rpm2 = ft.calculate_rpm(current_node, next_node)

                msg = Twist()
                linear_velocity = self.r * (rpm1 + rpm2) * 0.5 * 0.10472  # Convert to m/s
                angular_velocity = self.r * (rpm2 - rpm1) / self.L  # in rad/s

                msg.linear.x = linear_velocity
                msg.angular.z = angular_velocity

                self.publisher_.publish(msg)
                self.get_logger().info(f'Published linear velocity: {linear_velocity:.2f} m/s, angular velocity: {angular_velocity:.2f} rad/s')

                self.path_index += 1

def main(args=None):
    rclpy.init(args=args)

    velocity_publisher = VelocityPublisher()

    rclpy.spin(velocity_publisher)

    velocity_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()