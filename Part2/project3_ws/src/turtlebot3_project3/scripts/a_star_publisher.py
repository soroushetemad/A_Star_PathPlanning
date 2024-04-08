# !/usr/bin/env python3

import math
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_geometry_msgs import TransformStamped
import tf2_ros
from geometry_msgs.msg import Quaternion
import time

from a_star import *

map_height = 2000  # Define map_height here, as it's used in the code

class BotController(object):
    def _init_(self):
        self.node = rclpy.create_node('a_star_publisher')
        self._velocity_publisher = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self._odom_subscriber = self.node.create_subscription(Odometry,'/odom',self.odom_callback,10)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self.node)
        self._velocity_msg = Twist()
        self._robot_x = 0.0
        self._robot_y = 0.0
        self._robot_yaw = 0.0

    def odom_callback(self, msg):
        orientation_q = msg.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        x, y, yaw = Quaternion(orientation_list)
        self._robot_x = msg.pose.position.x
        self._robot_y = msg.pose.position.y
        self._robot_yaw = yaw

    def get_transform(self):
        try:
            trans = self._tf_buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time())
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            rot = trans.transform.rotation
            x, y, yaw = Quaternion([rot.x, rot.y, rot.z, rot.w])
            return x, y, yaw
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return self._robot_x, self._robot_y, self._robot_yaw

    def cmd_vel(self, linear, angular):
        velocity = Twist()
        velocity.linear.x = linear
        velocity.angular.z = angular
        self._velocity_publisher.publish(velocity)

def main(args=None):
    rclpy.init(args=args)
    RPM1 = 10
    RPM2 = 15
    s_x, s_y, e_x, e_y = 500, 1000, 5750, 250
    start_theta = 30
    robot_radius = 22

    c2g = dist((s_x, map_height - s_y), (e_x, map_height - e_y))
    total_cost = c2g
    start_node = Node(s_x, map_height - s_y,-1,start_theta,0,0,0,c2g,total_cost)
    goal_node = Node(e_x, map_height - e_y, -1,0,0,0,c2g,0,total_cost)
    obstacle_frame = Configuration_space()
    found_goal, Nodes_List, Path_List = a_star(start_node, goal_node, RPM1, RPM2, obstacle_frame)

    if found_goal:
        x_path, y_path, theta_path, RPM_Left, RPM_Right, total_cost = Backtrack(goal_node)
    else:
        print("Goal not found")
        return

    print('\nPath found successfully')
    print('\nWaiting to publish cmd_vel messages')
    # rclpy.spin_once()  # Use spin_once to allow subscribers to connect
    time.sleep(10)
    print('\nPublishing messages')

    botcontroller = BotController()
    rate = botcontroller.node.create_rate(10)
    L = 0.287
    dt = 0.1

    for i in range(len(x_path)):
        UL = RPM_Left[i]
        UR = RPM_Right[i]
        theta = theta_path[i]
        pi = math.pi

        UL = UL * 2 * pi / 60
        UR = UR * 2 * pi / 60
        thetan = 3.14 * theta / 180

        theta_dot = (robot_radius / L) * (UR - UL)
        velocity_value = (robot_radius / 2) * (UL + UR)

        Xn, Yn, yaw = botcontroller.get_transform()
        yaw = yaw * 180 / np.pi

        diff = ((thetan - yaw) + 180) % 360 - 180
        print("Velocity value: ", velocity_value, "Theta dot: ", theta_dot, "Diff: ", diff)

        botcontroller.cmd_vel(velocity_value, theta_dot + 0.005 * diff)
        rate.sleep()

    botcontroller.cmd_vel(0, 0)
    print('Successfully reached')

    rclpy.spin(botcontroller.node)
    botcontroller.node.destroy_node()
    rclpy.shutdown()

if __name__ == '_main_':
    main()
    rclpy.spin_once()  # Use spin_once to allow subscribers to connect