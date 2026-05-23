# ECE276B PR2 Spring 2026

## Overview
In this assignment, you will implement a motion planning algorithm for dynamic goals in several 3-D environments.

### 1. main.py
This file contains examples of how to load and display the environments and how to call a motion planner and plot the planned path. For the dynamic goal scenarios, an additional visualization is provided to illustrate the goal movement and the path planning process. If the planner fails to find a feasible path to a goal within the prescribed time interval, the corresponding goal will be shown in purple in the final plot. Feel free to modify this file to fit your specific goals for the project. In particular, you should certainly replace Line 189 with a call to a function which checks whether the planned path intersects the boundary or any of the blocks in the environment. The values of t1 and t2 can be modified in Lines 160-165.

### 2. Planner.py
This file contains an implementation of a baseline planner. The baseline planner gets stuck in complex environments and is not very careful with collision checking. Modify or replace this file in any way necessary for your own implementation.

### 3. astar.py
This file contains a class defining a node for the A* algorithm as well as an incomplete implementation of A*. Feel free to continue implementing the A* algorithm here or start over with your own approach.

### 4. maps
This folder contains 7 test environments described via a rectangular outer boundary and a list of rectangular obstacles. The start and goal points for each environment are specified in main.py.



