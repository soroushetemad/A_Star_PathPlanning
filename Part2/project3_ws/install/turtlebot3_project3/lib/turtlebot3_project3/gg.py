#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class KeyboardControlNode(Node):

    def __init__(self):
        super().__init__('keyboard_control_node')

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def run_keyboard_control(self):

        velocity_message = Twist()
        linear_vel=0.5
        angular_vel=0.2

                
        # Publish the twist message
        velocity_message.linear.x = linear_vel
        velocity_message.angular.z = angular_vel
        
        self.cmd_vel_pub.publish(velocity_message)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlNode()
    node.run_keyboard_control()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
