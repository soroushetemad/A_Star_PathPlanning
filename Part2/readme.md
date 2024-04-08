# A* Path Planning for TurtleBot3 Waffle

Name: Soroush Etemad
Directory ID: setemad
UID: 116608992

Name: Shreyas Acharya
Directory ID: shrey1s0
UID: 120426643

## Overview
This project focuses on implementing the A* algorithm for path planning on a TurtleBot3 Waffle robot navigating through a predefined obstacle space. The A* algorithm is a widely used method for finding the shortest path between nodes in a graph, making it suitable for navigation tasks.

## Dependencies
- ROS2 (Robot Operating System)
- TurtleBot3 packages
- Gazebo 
- Python 
- OpenCV

## Installation
1. Install ROS following the instructions provided on the official ROS website.
2. Install the TurtleBot3 packages by following the instructions on the official TurtleBot3 GitHub repository.
3. Install Gazebo for the simulation and visualization.
4. Ensure Python is installed on your system.
5. Ensure OpenCV is installed form part1 visualization.

# Part 01: 2D Implementation:
## Running Program
1. run the following file:
``` proj3p2_p1_Soroush_Shreyas.py ```
3. Assign Clearance to the Obstacles: 10
4. Enter coordinates for Start Node (x y): 500 1000
5. Enter Orientation of the robot at start node (multiple of 30): 0
6. Enter coordinates for Goal Node (x y): 5750 250
7. Enter rpms of the wheels(rpm1 rpm2): 10 15
8. Code will run and show a live animation
9. Once the animation is finished, a video file (Astar_path2_planning_video.mp4) will be saved automatically.

# Part 02: Gazebo Visualization: 

## Running Program
1. Clone the github repository using following command:
``` git clone https://github.com/soroushetemadA_Star_PathPlanning.git ```

2. Navigate to the workspace folder(turtlebot3_project3).
3. Build the workspace :
``` colcon build ```

4. Source ROS in the new terminal.
5. In same terminal run:
``` ros2 launch turtlebot3_project3 competition_world.launch.py ```
6. Source ROS in new terminal.
6. In same terminal run:
 ```ros2 run turtlebot3_project3 a_star_publisher.py ```


Example input: 
   - Start node: x,y, theta = 500, 1000, 0 
   - End node: x,y = 5670, 560
