# A* Path Planning for TurtleBot3 Waffle

Name: Soroush Etemad
Directory ID: setemad
UID: 116608992

Name: Shreyas Acharya
Directory ID: shrey1s0
UID: 120426643

## Overview
This project focuses on implementing the A* (A-star) algorithm for path planning on a TurtleBot3 Waffle robot navigating through a predefined obstacle space. The A* algorithm is a widely used method for finding the shortest path between nodes in a graph, making it suitable for navigation tasks.

## Dependencies
- ROS2 (Robot Operating System)
- TurtleBot3 packages
- Gazebo 
- Python 

## Installation
1. Install ROS following the instructions provided on the official ROS website.
2. Install the TurtleBot3 packages by following the instructions on the official TurtleBot3 GitHub repository.
3. Install Gazebo for the simulation and visualization.
4. Ensure Python is installed on your system.

## Running Program
1. In one terminal run: ros2 launch turtlebot3_project3 competition_world.launch.py
2. In another terminal run: ros2 run turtlebot3_project3 a_star_publisher.py

Example input: 
   - Start node: x,y, theta = 500, 1000, 0 
   - End node: x,y = 5750, 250
