ENPM661 Project 3 Phase 2| A* Algorithm for a mobile robot with non-holonomic constraints, and Gazebo simulation.

Kshitij Aggarwal

119211618

kshitij2

Github repository link: https://github.com/KshitijAggarwal8/ENPM661_Project3_Phase2/tree/master

# Part 1 (A* for non-holonomic): #
________________________________________________________

**The main executable is `a_star_kshitij.py`**

________________________________________________________
**User Inputs:**
- Obstacle Clearance
- Robot Radius
- Robot Step Size
- Start point coordinates (space separated)
- Start point orientation (in multiples of 30)
- Goal point coordinates (space separated)
- Goal point orientation (in multiples of 30)

________________________________________________________
**Libraries used:**
- numpy
- math
- heapq
- time
- cv2
- matplotlib
- os

________________________________________________________
**Output:**
- If the goal is reached: "Found the GOAL!"
- Video generation progress (in %age)
- Total execution time

________________________________________________________
**Output Video:**

The video was recorded for the following inputs:

- Start Point: (100, 20)

- Start orientation: 30

- Goal Point: (420, 380)

- Goal orientation: 30

________________________________________________________


# Part 2 (Gazebo Simulation): #

________________________________________________________
**How to run:**
1. Navigate to the workspace project3_ws, and source the underlay.
2. export the turtlebot model: `export TURTLEBOT3_MODEL-waffle`
3. Launch the competition arena: `ros2 launch turtlebot3_project3 competition_world.launch.py` 
4. Open another terminal, source the underlay, and run the A star node: `ros2 run turtlebot3_project3 astar_node.py`
5. After initialising the inputs, the algorithm will calculate the optimal path and display it in a separate window. Close this window to initiate the Robot's navigation.

________________________________________________________
**User Inputs:**
- Clearance (in m)
- RPM 1 
- RPM 2
- Start point x (in m)
- Start point y (in m)
- Start Orientation
- Goal point x (in m)
- Goal point y (in m)

________________________________________________________
**Output Video:**

The video has been recored for the following input values:

Enter the clearance 0.2

Enter RPM1:1

Enter RPM2:2

Enter start point x coordinate (in metres): 0.5

Enter start point y coordinate (in metres): 1

Enter start orientation in degrees: 0

Enter goal point x coordinate (in metres): 5.75

Enter goal point y coordinate (in metres): 1

Video Link: https://drive.google.com/file/d/1zPUFRwBRJZqdP_vu8PJEW0X8y5Kw2ZH0/view?usp=sharing



________________________________________________________
**Note:**
1. The program does not do well with greater RPM values (suggested values - 1,2)
2. All the dimensions have been kept in metres for consistency
3. Robot might collide at times for a low clearance (suggested clearance between 1.5-2.0)

________________________________________________________
**Libraries used:**
- numpy
- math
- heapq
- time
- rclpy
- geometry_msgs
- sys
