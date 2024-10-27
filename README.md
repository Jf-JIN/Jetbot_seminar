# Jetbot_seminar

- [Overview](https://github.com/Jf-JIN/Jetbot_seminar?tab=readme-ov-file#%E7%AE%80%E8%BF%B0)
- [Task Description](https://github.com/Jf-JIN/Jetbot_seminar?tab=readme-ov-file#%E7%AE%80%E8%BF%B0)
- [Project Structure](https://github.com/Jf-JIN/Jetbot_seminar?tab=readme-ov-file#%E7%AE%80%E8%BF%B0)
- [Remaining Issues](https://github.com/Jf-JIN/Jetbot_seminar?tab=readme-ov-file#%E7%AE%80%E8%BF%B0)
## Overview
This project focuses on designing algorithms and controls to enable the Jetbot to autonomously plan its path, move, explore, and build a map as required.

Original project documentation:
-  [Project Seminar 'Robotics and Computational Intelligence' 2024 presented by RIS | Technical University of Darmstadt](https://github.com/NikHoh/jetbot_maze)
-  [Craft a maze from lasercut MDF walls and generate the corresponding AprilTag Bundle YAML](https://github.com/NikHoh/apriltag-maze)

##### Project references：
- [jetbot](https://github.com/NVIDIA-AI-IOT/jetbot)
- [jetbot_ros](https://github.com/dusty-nv/jetbot_ros)
- [apriltag](https://github.com/AprilRobotics/apriltag)
- [apriltag-imgs](https://github.com/AprilRobotics/apriltag-imgs)
- [Adafruit_CircuitPython_MotorKit](https://github.com/adafruit/Adafruit_CircuitPython_MotorKit)

## Task Description
The seminar focuses on three main tasks:

* In a predefined maze, assign a start and end point for the Jetbot, which will autonomously plan its path and reach the designated location.
* In an undefined maze, the Jetbot explores autonomously and generates a maze YAML file.
* In an undefined maze, the Jetbot explores until it locates a specified object (AprilTag with a code above 900), then returns to the starting point.
<div style="display:inline-block;"> <img src="https://github.com/Jf-JIN/Jetbot_seminar/image/jetbot2.gif" alt="gif1" height = "200"> <img src="https://github.com/Jf-JIN/Jetbot_seminar/image/jetbot3.gif" alt="gif2" height = "200"></div>

## Project Structure
The project comprises: client design (frontend UI + some backend algorithms), server design (based on socket communication), and a car system (subdivided below). The car system consists of four main modules: central processing, data acquisition, motor drive control, and pathfinding algorithms.

#### Client Design
The client parses the YAML file, constructs a map class, communicates with the car server, and displays real-time data, including camera, motor, and path information. It also allows for real-time adjustment of PID parameters for debugging and modifies the car’s movement state.

<div style="display:inline-block;"> <img src="https://github.com/Jf-JIN/Jetbot_seminar/image/jetbot1.gif" alt="jetbot1" height = "200"> </div>
Map construction involves analyzing the YAML file, establishing different wall classes based on each wall's center point and orientation, and placing them within a map (manager) class containing two matrices: an object matrix (containing actual wall objects for extracting wall information) and an abstract matrix (a binary matrix for path calculation using A*).

<div style="display:inline-block;"> <img src="https://github.com/Jf-JIN/Jetbot_seminar/image/matrix2.png" alt="matrix2" height = "200"> <img src="https://github.com/Jf-JIN/Jetbot_seminar/image/matrix3.png" alt="matrix3" height = "200"></div>
#### Motor Drive
Primarily responsible for motor control, using PID or other control algorithms, including pose correction. The module receives data packets from the data acquisition unit to determine location and then executes movements as directed by the central processing algorithm.

#### Data Acquisition
Handles camera-based QR code recognition, identification of front-facing AprilTags on both sides, and IMU data processing. The data is packaged into a custom data class, JLocation, which is continuously transmitted to both the central processing and motor control modules.

#### Central Processing
The primary role is to pass the pathfinding results to the motor drive for execution. It also transmits data from the data acquisition module to the client for display in real-time and receives instruction packets from the client.

#### Pathfinding Algorithms
Responsible for path computation and planning, creating the map, and sending suggestions to the central processing unit. The pathfinding module employs two primary algorithms: A* and DFS* (Depth First Search).

<div style="display:inline-block;"> <img src="https://github.com/Jf-JIN/Jetbot_seminar/image/A+Algo.png" alt="A*Algo" height = "200"> </div>
*  A* Algorithm Used in tasks 1 and 3, with two path extension options: four-directional and eight-directional.
<div style="display:inline-block;"> <img src="https://github.com/Jf-JIN/Jetbot_seminar/image/erweitung4.png" alt="erweitung4" height = "200"> <img src="https://github.com/Jf-JIN/Jetbot_seminar/image/erweitung8.png" alt="erweitung8" height = "200"></div>
For a predefined maze, the algorithm results are as shown (using Manhattan distance):

<div style="display:inline-block;"> <img src="https://github.com/Jf-JIN/Jetbot_seminar/image/4-AStar.png" alt="4-AStar" height = "200"> <img src="https://github.com/Jf-JIN/Jetbot_seminar/image/8-AStar.png" alt="8-AStar" height = "200"></div>
The calculated path contains multiple waypoints. These points are then connected to form a route consisting of the start, end, and target actions, as illustrated below.

<div style="display:inline-block;"> <img src="https://github.com/Jf-JIN/Jetbot_seminar/image/nodes.png" alt="nodes" height = "200"> <img src="https://github.com/Jf-JIN/Jetbot_seminar/image/behandelt.png" alt="behandelt" height = "200"></div>
The car then moves according to the path instructions.

<div style="display:inline-block;"> <img src="https://github.com/Jf-JIN/Jetbot_seminar/image/DFS-Algo.png" alt="DFS-Algo" height = "200"> </div>
* DFS Algorithm Mainly used for tasks 2 and 3 to systematically explore each branch (node) in an unknown map. Two key challenges in this task include:
Free Wall Problem: Can cause the car to revisit the same node indefinitely. Recognizing visited nodes is essential to prevent infinite loops.
No Wall Problem: The car reliably detects AprilTags only one unit in front. When no AprilTag is detected ahead, a new node is added as a branch, potentially causing the same issue as the free wall problem.
<div style="display:inline-block;"> <img src="https://github.com/Jf-JIN/Jetbot_seminar/image/freiWand.png" alt="freiWand" height = "200"> <img src="https://github.com/Jf-JIN/Jetbot_seminar/image/keinWand.png" alt="keinWand" height = "200"></div>
* BFS algorithm was not completed in time to compare with DFS.

## Remaining Issues
1. AprilTag Recognition Angle: In task 1, the Jetbot sometimes approaches AprilTags at a slant, leading to positioning errors and potential collisions.
2. Reverse Backtracking in Task 3: Currently, the bot backtracks to the last node in reverse; this could be optimized by using the A* algorithm to calculate the shortest path back to the previous node.