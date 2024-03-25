# A* Path Planning

## Overview
This A* Path Planning algorithm is implemented in Python using OpenCV and NumPy libraries. It allows finding the shortest path between a start and a goal node in a given map with obstacles. The algorithm considers a robot with a specified clearance and radius, navigating through a defined configuration space.

## Instructions
To run the code, follow these steps:

1. Ensure you have Python installed on your system. Install the required libraries: `numpy` and `opencv-python`.
2. Run the script `a_star_path_planning.py`.
   
## Possible Inputs
When running the script, you will be prompted to provide the following inputs:

- **Clearance to the Obstacles**: 5
- **Radius of the Robot**: 5
- **Start Node Coordinates**: 200 40
- **Orientation of the Robot at Start Node**: 0 
- **Step Size of the Robot**: 10
- **Goal Node Coordinates**: 1150 250
- **Orientation of the Robot at Goal Node**: 30
  
Another example:
- **Clearance to the Obstacles**: 5
- **Radius of the Robot**: 3
- **Start Node Coordinates**: 10 10
- **Orientation of the Robot at Start Node**: 30
- **Step Size of the Robot**: 10
- **Goal Node Coordinates**: 200 200
- **Orientation of the Robot at Goal Node**: 30

Ensure that the provided inputs are within the boundaries of the map and avoid obstacles.
